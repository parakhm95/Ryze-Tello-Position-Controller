#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
import sys
import time
import threading 
import socket
import sys
import platform
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

host = ''
port = 9000
locaddr = (host,port)
error_x_prev = 0.00
error_y_prev = 0.00
error_z_prev = 0.00
goal = [7.00,0.00,1.00]
accum_x = 0.00
accum_y = 0.00
accum_z = 0.00


# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

tello_address = ('192.168.10.1', 8889)

sock.bind(locaddr)

def recv():
    count = 0
    while True: 
        try:
            data, server = sock.recvfrom(1518)
            print(data.decode(encoding="utf-8"))
        except Exception:
            print ('\nExit . . .\n')
            break


print ('\r\n\r\nTello Python3 Demo.\r\n')

print ('Tello: command takeoff land flip forward back left right \r\n       up down cw ccw speed speed?\r\n')

print ('end -- quit demo.\r\n')


#recvThread create
recvThread = threading.Thread(target=recv)
recvThread.start()

      

def msg(string):
    msg = string
    msg.encode(encoding="utf-8")
    sent = sock.sendto(msg, tello_address)

# msg = 'command'
# print ('working')
# msg = msg.encode(encoding="utf-8") 
# sent = sock.sendto(msg, tello_address)

msg('command')
msg('takeoff')
msg('speed 100')
# msg = 'takeoff'
# print ('working')
# msg = msg.encode(encoding="utf-8") 
# sent = sock.sendto(msg, tello_address)
# print ('done')

def nanocall(joy_msg):
    global goal
    if joy_msg.buttons[16]==1:
        goal[2] = 1.5
    if joy_msg.buttons[17]==1:
        goal[2] = 0.5
    # if joy_msg.buttons[17]==1:
    #     goal[0] = -5.5

def exit_land():
    msg('land')
    sock.close()
    print "shutdown time!"
    quit()
    sys.exit()



def position_get(posestamped_msg):
    global error_x_prev, error_y_prev,error_z_prev,goal,accum_x,accum_y,accum_z
    x = posestamped_msg.pose.pose.position.x
    y = posestamped_msg.pose.pose.position.y
    z = posestamped_msg.pose.pose.position.z

    # quaternion to euler
    quaternion = (posestamped_msg.pose.pose.orientation.x,
        posestamped_msg.pose.pose.orientation.y,
        posestamped_msg.pose.pose.orientation.z,
        posestamped_msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    #rotation matrix from world to robot frame
    error_x = float((goal[0]-x)*math.cos(yaw) + (goal[1]-y)*math.sin(yaw))
    error_y = float(-(goal[0]-x)*math.sin(yaw) + (goal[1]-y)*math.cos(yaw))
    error_z = float(goal[2] - z)

    # x position control PID
    accum_x = accum_x + error_x
    output_x = int((45*(error_x)) + 3700*(error_x - error_x_prev) + 0.0*accum_x)
    if(output_x > 100):
        output_x = 100
    elif(output_x < -100):
        output_x = -100
    if(accum_x > 300):
        accum_x = 300
    elif(accum_x < -300):
        accum_x = -300    

    # y position control PID
    accum_y = accum_y + error_y
    output_y = -int((45*(error_y)) + 3700*(error_y - error_y_prev) + 0.0*accum_y)
    if(output_y > 100):
        output_y = 100
    elif(output_y < -100):
        output_y = -100
    if(accum_y > 300):
        accum_y = 300
    elif(accum_y < -300):
        accum_y = -300

    #z position control PID
    accum_z = accum_z + error_z
    output_z = int(105*error_z - 20*(posestamped_msg.twist.twist.linear.z) + 0.05*accum_z)
    if (output_z > 100):
        output_z=100
    elif (output_z < -100):
        output_z= -100
    if (accum_z > 300):
        accum_z=300
    elif (accum_z < -300):
        accum_z=-300

    #commanding tello
    msg('rc %s %s %s 0' % (output_y,output_x,output_z))
    error_x_prev = error_x
    error_y_prev = error_y
    print("%s,%s,%s" % (output_x,output_y,output_z))
    

def tello_position():
    rospy.init_node('tello_position', anonymous=True)   
    rospy.Subscriber("/vicon/tello/odom", Odometry, position_get)
    rospy.Subscriber("/nanokontrol2", Joy, nanocall)
    freq = 100  # hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()    
    rospy.on_shutdown(exit_land)

# if __name__ == '__main__':
#     try:
#         tello_position()
#     except KeyboardInterrupt:
#         print('Interrupted')
#         try:
#             sys.exit(0)
#         except SystemExit:
#             os._exit(0)


if __name__ == '__main__':
    tello_position()
