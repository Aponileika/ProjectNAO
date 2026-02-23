import sys
sys.path.append(r"C:\Users\malte\projects\nao\pynaoqi-python2.7-2.8.6.23-win64-vs2015-20191127_152649\lib")

from naoqi import ALProxy
import vision_definitions
import threading
import time
import tempfile
import sys
import os
import socket
import json
import errno
import struct
import zlib



# Replace with your robot's IP
IP = "192.168.0.123"
PORT = 9559

################################################################
 
#Create proxy to ALVideoDevice
#camProxy = ALProxy("ALVideoDevice", IP, PORT)

# Subscribe to the camera
#resolution = vision_definitions.kQVGA  # 320x240
#colorSpace = vision_definitions.kRGBColorSpace
#fps = 30
#nameId = camProxy.subscribe("python_client", resolution, colorSpace, fps)

#########################################################

motion = ALProxy("ALMotion", IP, PORT)
posture = ALProxy("ALRobotPosture", IP, PORT)

####################################################
HOST = "127.0.0.1"
HOST_PORT_1 = 5000
#HOST_PORT_2 = 9000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, HOST_PORT_1))
s.listen(1)
print(1)
conn, addr = s.accept()
print(2)

#try:
#    v = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#    v.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#    v.connect((HOST, HOST_PORT_2))
#    socket_connected = True
#except socket.error as e:
#    print("Failed to connect to server:", e)
#    socket_connected = False


########################################################


#def video():
#   while True:
#        # Get image
#        imageContainer = camProxy.getImageRemote(nameId)#

        # Unpack image data
#        width = imageContainer[0]
#        height = imageContainer[1]
#        payload = imageContainer[6]
#        imageFormat = imageContainer[2]  # e.g., 'RGB'

#        fmt_bytes = imageFormat.encode('ascii')
#        fmt_len = len(fmt_bytes)

#        payload = zlib.compress(payload)
#        comp_flag = 1

#        header =struct.pack('!B', fmt_len) + fmt_bytes
#        header += struct.pack('!II B I', width, height, comp_flag, len(payload))

#        full_message = header + payload

#        try:
#            v.sendall(struct.pack('!I', len(full_message)))
#            v.sendall(full_message)
#        except socket.error as e:
#            if getattr(e, 'errno', None) == errno.EPIPE:
#                print('Peer closed connection')
#            else:
#                print('Socket error sending frame:', e)
#            break



def walk():
    buffer = ''
    while True:
        try:
            print(4)
            chunk = conn.recv(1024).decode('utf-8')

            print(5)

            line = buffer
            if not line: 
                continue
            try:
                msg = json.loads(line)
                print(msg)
                            
                x1 = float(msg.get('x', 0))
                y1 = float(msg.get('y', 0))
                theta = float(msg.get('theta', 0))

                print(x1)
                print(y1)
                print(theta)
                

            except json.JSONDecodeError as e:
                print("JSON decode error:", e)
                continue
            #print(x1, y1)

            if x1**2 < 0.2:
                x1 = 0
            if y1**2 < 0.2:
                y1 = 0
            if theta**2 < 0.2:
                theta = 0

            #if x1 == 0 and y1 == 0 and x2 == 0:
            #    continue

            motion.moveTo(y1, -x1, theta)

                
        except Exception as e:
            print("Error:", e)
            conn.close()
            break


#def quit(camProxy, nameId):

#    camProxy.unsubscribe(nameId)

def main():

    try:
        motion.wakeUp()
        #posture.goToPosture("Sit", 0.5)
        posture.goToPosture("StandInit", 1)
        motion.wbEnable(True)
    except Exception as e:
        print("Error:", e)
        
    print(3)
    walk_t = threading.Thread(target=walk)
    #video_t = threading.Thread(target=video)

    walk_t.daemon = True
    #video_t.daemon = True

    walk_t.start()
    #video_t.start()

    walk_t.join()
    #video_t.join()

main()

