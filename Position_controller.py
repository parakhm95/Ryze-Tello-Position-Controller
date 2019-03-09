import rospy
import math
from geometry_msgs.msg import PoseStamped
import sys
import time
import threading 
import socket
import sys
import platform

host = ''
port = 9000
locaddr = (host,port)
lastpose = 0 


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
# msg = 'takeoff'
# print ('working')
# msg = msg.encode(encoding="utf-8") 
# sent = sock.sendto(msg, tello_address)
# print ('done')

def exit_land():
   msg = 'land'
   print ('working')
   msg = msg.encode(encoding="utf-8") 
   sent = sock.sendto(msg, tello_address)
   print ('done')
   print ('\n . . .\n')
   sock.close()
   print "shutdown time!"



def position_get(posestamped_msg):
    global lastpose
    currentpose = posestamped_msg.pose.position.x
    currenterror = 6 - currentpose
    output = int((-75*(currenterror)) + -40*(lastpose - currentpose))
    
    if(output > 100):
        output = 100
    elif(output < -100):
        output = -100
    print output
    msg('rc 0 %s 0 0' % (output))
    lastpose = posestamped_msg.pose.position.x
    
    # print(navsat_msg.longitude)



def tello_position():
    rospy.init_node('tello_position', anonymous=True)   
    rospy.Subscriber("/vicon/tello/pose", PoseStamped, position_get)
    freq = 100  # hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        rate.sleep()
        # print("running")

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
