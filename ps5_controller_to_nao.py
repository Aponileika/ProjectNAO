import pygame
import socket
import json
import time

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise Exception("No joystick detected. Please connect your PS5 controller.")
joystick = pygame.joystick.Joystick(0)
joystick.init()

HOST = '127.0.0.1'  # Change if mov.py is running on another machine
PORT = 5000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Connecting to {}:{} ...".format(HOST, PORT))
s.connect((HOST, PORT))
print("Connected. Sending controller data...")

try:
    while True:
        pygame.event.pump()
        # PS5 controller axes may differ, adjust if needed
        x1 = joystick.get_axis(0)  # Left stick horizontal
        y1 = -joystick.get_axis(1) # Left stick vertical (invert for natural forward)
        x2 = joystick.get_axis(2)  # Right stick horizontal
        y2 = -joystick.get_axis(3) # Right stick vertical (if needed)
        msg = json.dumps({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2}) + '\n'
        s.sendall(msg.encode('utf-8'))
        time.sleep(0.05)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    s.close()
    pygame.quit()
