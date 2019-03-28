#! /usr/bin/env python
import rospy
import math
import time
import threading
import socket
import sys
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from tello_position_controller.msg import GoTo
from std_srvs.srv import Trigger


host = ''
port = 9000
locaddr = (host,port)
prev_error = [0.00,0.00,0.00]
goal = GoTo()
telemetry = GoTo()
goal.x = 7.00
goal.y = 0.00
goal.z = 1.00
goal.yaw = 0.00
accum = [0.00,0.00,0.00]
gains = [[600,600,0.15], [600, 600, 0.15], [135, 20, 0.13], [30, 0, 0.00]]
data = None
takeoff_flag = False
output = [0,0,0,0]


# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

tello_address = ('192.168.10.1', 8889)

sock.bind(locaddr)

def recv():
    global data
    while True:
        try:
            data, server = sock.recvfrom(1518)
            print(data.decode(encoding="utf-8"))
        except Exception:
            print ('\nExit . . .\n')
            break


print ('command mode ready, beginning take-off')


#recvThread create
recvThread = threading.Thread(target=recv)
recvThread.start()



def msg(string):
    msg = string
    msg.encode(encoding="utf-8")
    sent = sock.sendto(msg, tello_address)
    if(sent == 0):
        print("socket machine broke")
    return



def exit_land():
    msg('rc 0 0 0 0')
    msg('land')
    sock.close()
    print "shutdown time!"
    quit()
    sys.exit()


def limiter(inp,min,max):
    if(inp > max):
        inp = max
    elif(inp < min):
        inp = min
    return inp

def goal(GoTo_msg):
    global goal
    goal=GoTo_msg
    rospy.loginfo("Goal changed to %s %s %s %s",goal.x,goal.y,goal.z,goal.yaw)
    return


def position_get(odometry_msg):
    # print(rospy.get_time())
    global prev_error,goal,accum, output, telemetry
    x = odometry_msg.pose.pose.position.x
    y = odometry_msg.pose.pose.position.y
    z = odometry_msg.pose.pose.position.z

    # quaternion to euler
    quaternion = (odometry_msg.pose.pose.orientation.x,
        odometry_msg.pose.pose.orientation.y,
        odometry_msg.pose.pose.orientation.z,
        odometry_msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    # rotation matrix from world to robot frame
    error_x = float((goal.x-x)*math.cos(yaw) + (goal.y-y)*math.sin(yaw))
    error_y = float(-(goal.x-x)*math.sin(yaw) + (goal.y-y)*math.cos(yaw))
    error_z = float(goal.z - z)
    error_yaw = float(goal.yaw-yaw)
    vel_x = float(odometry_msg.twist.twist.linear.x*math.cos(yaw) + odometry_msg.twist.twist.linear.y*math.sin(yaw))
    vel_y = float(-odometry_msg.twist.twist.linear.x * math.sin(yaw) + odometry_msg.twist.twist.linear.y * math.cos(yaw))

    if takeoff_flag is True:
        # x position control PID
        accum[0] = accum[0] + error_x
        output[0]=int((gains[0][0] * (error_x)) + gains[0][1] * (-vel_x) + gains[0][2] * accum[0])
        output[0] = limiter(output[0],-100,100)
        accum[0] = limiter(accum[0],-450,450)


        # y position control PID
        accum[1] = accum[1] + error_y
        output[1]=-int((gains[1][0] * (error_y)) + gains[1][1] * (-vel_y) + gains[1][2] * accum[1])
        output[1] = limiter(output[1],-100,100)
        accum[1] = limiter(accum[1],-450,450)
        # print(error_y - prev_error[1])

        # z position control PID
        accum[2] = accum[2] + error_z
        output[2] = int(gains[2][0]*error_z - gains[2][1]*(odometry_msg.twist.twist.linear.z) + gains[2][2]*accum[2])
        output[2] = limiter(output[2],-30,30)
        accum[2] = limiter(accum[2],-450,450)

        # yaw controller PID
        output[3] = -int(gains[3][0]*error_yaw)
        output[3] = limiter(output[3],-100,100)
        # print(yaw)

        #commanding tello


        #print statements
        # print("%s,%s,%s %s" % (output[0],output[1],output[2],output[3]))
        # print("%s,%s" %(error_x - prev_error[0], vel_x))
        # print("%s,%s" % (error_y - prev_error[1], vel_y))
        print("%s,%s,%s" % (accum[0],accum[1],accum[2]))
        datastream=rospy.Publisher('/tello/datastream', GoTo, queue_size=0)
        telemetry.x,telemetry.y,telemetry.z,telemetry.yaw = error_y,error_y - prev_error[1],gains[1][2]*accum[1],vel_y
        datastream.publish(telemetry)
        #saving last state
        prev_error[0]=error_x
        prev_error[1]=error_y

    # print(rospy.get_time())




def takeoff_srv(trig):
    global takeoff_flag
    if battery_check():
        takeoff_flag = True
        msg('takeoff')
        rospy.loginfo("takeoff requested")
        time.sleep(1.0)
        if(data == 'ok'):
            return {'success': True,'message' : "Taking off"}
        elif(data == 'error'):
            return {'success': False,'message' : "Error Taking off"}
    else:
        rospy.logerr("Takeoff flag is false")
        return {'success': False, 'message': "Error Taking off"}

def land_srv(trig):
    global takeoff_flag
    takeoff_flag = False
    rospy.loginfo("landing requested")
    msg('rc 0 0 0 0')
    msg('land')
    return {'success': True,'message' : "Initiating landing"}

def emergency_srv(trig):
    global takeoff_flag
    takeoff_flag = False
    rospy.loginfo("emergency requested")
    msg('emergency')
    return {'success': True, 'message': "Emergency"}

def startup():
    msg('command')
    time.sleep(1.0)
    msg('speed 100')
    time.sleep(1.0)
    msg('battery?')
    time.sleep(1.0)
    battery = int(data)
    if(battery > 10):
        rospy.loginfo("Battery is %s",data)
        rospy.loginfo("Ready to takeoff")
    else:
        rospy.logerr("Battery is %s",data)
        rospy.logerr("Cannot takeoff")
    return

def battery_check():
    msg('battery?')
    time.sleep(0.1)
    battery=int(data)
    if (battery > 10):
        rospy.loginfo("Battery is %s", data)
        rospy.loginfo("Ready to takeoff")
        return True
    else:
        rospy.logerr("Battery is %s", data)
        rospy.logerr("Cannot takeoff")
        return False




def tello_position():
    global output
    rospy.init_node('tello_position', anonymous=True)
    rospy.Subscriber("/vicon/tello/odom", Odometry, position_get)
    takeoff=rospy.Service('/tello/takeoff', Trigger, takeoff_srv)
    land=rospy.Service('/tello/land', Trigger, land_srv)
    emergency=rospy.Service('/tello/emergency', Trigger, emergency_srv)
    rospy.Subscriber("/tello/GoTo", GoTo, goal)
    freq = 40  # hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        msg('rc %s %s %s %s' % (output[1], output[0], output[2], output[3]))
        rate.sleep()

    rospy.spin()
    rospy.on_shutdown(exit_land)


if __name__ == '__main__':
    goal.x=7.00
    goal.y=0.00
    goal.z=1.00
    goal.yaw=0.00
    startup()
    tello_position()
