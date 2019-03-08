#
# Tello Python3 Control Demo 
#
# http://www.ryzerobotics.com/
#
# 1/1/2018

import threading 
import socket
import sys
import time
import platform  

host = ''
port = 9000
locaddr = (host,port) 


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

 

   #  python_version = str(platform.python_version())
   #  version_init_num = int(python_version.partition('.')[0]) 
   # # print (version_init_num)
   #  if version_init_num == 3:
   #      msg = input("");
   #  elif version_init_num == 2:
   #      msg = raw_input("");
    
   #  if not msg:
   #      break  
   #  if 'end' in msg:
   #      print ('...')
   #      sock.close()  
   #      break
    # Send data
msg = 'command'
print ('working')
msg = msg.encode(encoding="utf-8") 
sent = sock.sendto(msg, tello_address)


msg = 'takeoff'
print ('working')
msg = msg.encode(encoding="utf-8") 
sent = sock.sendto(msg, tello_address)
print ('done')

while True:
    try:

        msg = 'rc -100 0 0 0'
        print ('working')
        msg = msg.encode(encoding="utf-8") 
        sent = sock.sendto(msg, tello_address)
        print ('done')
    
        time.sleep(0.6)
        
        msg = 'rc 100 0 0 0'
        print ('working')
        msg = msg.encode(encoding="utf-8") 
        sent = sock.sendto(msg, tello_address)
        print ('done')
    
        time.sleep(0.6)
    except KeyboardInterrupt:
        msg = 'land'
        print ('working')
        msg = msg.encode(encoding="utf-8") 
        sent = sock.sendto(msg, tello_address)
        print ('done')
        print ('\n . . .\n')
        sock.close()  
        break