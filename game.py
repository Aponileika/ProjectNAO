import socket
import json
import pygame
import sys


HOST = '127.0.0.1'
PORT = 5000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

def send_joystick_data(x, y, theta):
    data = json.dumps({'x': x, 'y': y, 'theta': theta})
    try:
        s.sendall(data.encode('utf-8'))
    except socket.error as e:
        print("Socket error:", e)

pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
print(f"Joysticks found: {joystick_count}")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Connected to:", joystick.get_name())

clock = pygame.time.Clock()
while True:
    pygame.event.pump()  # Process internal events

    left_x = joystick.get_axis(0)  # Left stick horizontal
    left_y = joystick.get_axis(1)  # Left stick vertical
    right_x = joystick.get_axis(3) # Right stick horizontal
    right_y = joystick.get_axis(4) # Right stick vertical

    

    print("Left Stick:", left_x, left_y)
    #print("Right Stick:", right_x, right_y)
    send_joystick_data(left_x, left_y, right_y)
    clock.tick(100)  # Limit to 1 FPS

    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN and joystick.get_button(0):  # Button A
            print("Exit button pressed.")
            s.close()
            sys.exit()
            break
