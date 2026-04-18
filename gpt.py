from openai import OpenAI
#import pc_main
import re
import json
import socket
import time
client = OpenAI(api_key="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")


HOST = '127.0.0.1'
PORT = 5000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

with open("prompt_text_curr.txt", "r") as f:
    prompt_text = f.read()

messages = []
temp_mess = []
messages.append({"role": "system", "content": prompt_text})
#pc_main.conn()

data = json.dumps({'command' : 'stand'})
try:
    s.sendall(data.encode())
except socket.error as e:
    print("Socket error:", e)

############################



def walk(dest):
    data = json.dumps({'command' : 'walk', 'x': 1.0, 'y': 0.0, 'theta': 0.0})
    print(data)
    try:
        s.sendall(data.encode())
    except socket.error as e:
        print("Socket error:", e)
    print("now walking")
    time.sleep(10)
    data = json.dumps({'command' : 'walk', 'x': 0.0, 'y': 0.0, 'theta': 0.0})
    try:
        s.sendall(data.encode())
    except socket.error as e:
        print("Socket error:", e)



def get_list(list):
    print(list)
    with open(f'{list[0]}.json', "r") as f:
        data = json.load(f)
        messages.append({"role": "system", "content": f'{list} contains: {data}'})
        response = client.chat.completions.create(
            model="gpt-4.1",
            messages = messages
        )
        return data


def new_message():

    user_input = input("You: ")

    # Add user message
    messages.append({"role": "user", "content": user_input})

    # Send to API
    response = client.chat.completions.create(
        model="gpt-4.1",
        messages = messages
    )

    # Get assistant reply
    reply = response.choices[0].message.content or ""
    print("Assistant:", reply)

    # Add assistant reply to history
    messages.append({"role": "assistant", "content": reply})

    return reply

def to_action(reply):
    matches = re.findall(r'(\w+)\(([^)]*)\)', reply)

    A = []
    for head, inside in matches:
        parts = [head] + [p.strip() for p in inside.split(",")]
        A.append(parts)

    print(A)

    #for i in A:
    i = A[0]
    print(f'{i[0]} ( {i[1:]})')
    if len(i[0]) > 1:
        result = globals()[i[0]](i[1:])
    else:
        result = globals()[i[0]]()

    messages.append({"role": "assistant", "content": str(i) + "has ben run and the result whas" + str(result) + ", adapt the solution to this new information"})
    response = client.chat.completions.create(
        model="gpt-4.1",
        messages = messages
    )

    reply = response.choices[0].message.content or ""
    if reply != None:
        to_action(reply)

    

#to_action('"get_list(items)", "get_list(places)"')
while 1:
    reply = new_message()
    to_action(reply)