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

host = ''
port = 9000
locaddr = (host,port)
lastpose = PoseStamped()
goal = [-4.00,0.00]
accum_x = 0.00
accum_y = 0.00


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
        goal[0] = -2.5
    if joy_msg.buttons[17]==1:
        goal[0] = -5.5
    if joy_msg.buttons[17]==1:
        goal[0] = -5.5

def exit_land():
    msg('land')
    sock.close()
    print "shutdown time!"
    quit()
    sys.exit()



def position_get(posestamped_msg):
    global lastpose,goal,accum_x,accum_y
    x = posestamped_msg.pose.position.x
    y = posestamped_msg.pose.position.y

    # quaternion to euler
    quaternion = (posestamped_msg.pose.orientation.x,
        posestamped_msg.pose.orientation.y,
        posestamped_msg.pose.orientation.z,
        posestamped_msg.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    #rotation matrix from world to robot frame
    goal_x = float(goal[0]*math.cos(yaw) + goal[1]*math.sin(yaw))
    goal_y = float(-goal[0]*math.sin(yaw) + goal[1]*math.cos(yaw))

    # x position control PID
    currenterror =  float(goal_x - x)
    accum_x = accum_x + currenterror
    output_x = int((50*(currenterror)) + -80*(x - lastpose.pose.position.x) + 0.0*accum_x)
    if(output_x > 100):
        output_x = 100
    elif(output_x < -100):
        output_x = -100
    if(accum_x > 300):
        accum_x = 300
    elif(accum_x < -300):
        accum_x = -300
    print currenterror

    # y position control PID
    currenterror =  float(goal_y - y)
    accum_y = accum_y + currenterror
    output_y = -int((50*(currenterror)) + -80*(y - lastpose.pose.position.y) + 0.0*accum_y)
    if(output_y > 100):
        output_y = 100
    elif(output_y < -100):
        output_y = -100
    if(accum_y > 300):
        accum_y = 300
    elif(accum_y < -300):
        accum_y = -300
    #commanding tello
    msg('rc %s %s 0 0' % (output_y,output_x))
    lastpose = posestamped_msg
    

def tello_position():
    rospy.init_node('tello_position', anonymous=True)   
    rospy.Subscriber("/vicon/tello/pose", PoseStamped, position_get)
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