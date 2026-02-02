import socket
import json


HOST = '127.0.0.1'
PORT = 5000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

def send_message(message):
    """
    Function to send messages

    * The argument to the json.dumps MUST be a dictionary
    * The dictionary MUST have a 'command' key
    * The rest of the arguments needed to decode the message, such as position data,
      is sent with appropiate keys
    """

    if message == "q":
        data = json.dumps({'command': 'quit'})
    elif message == "sit":
        data = json.dumps({'command': 'sit'})
    elif message == "stand":
        data = json.dumps({'command': 'stand'})

    try:
        s.sendall(data.encode())
    except socket.error as e:
        print("Socket error:", e)

while True:

    message = input()
    send_message(message)
    if message == "q":
        s.close()
        break
