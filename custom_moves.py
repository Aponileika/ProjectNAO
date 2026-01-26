"""
File for custom moves

Each function is a custom move, do make more :D
Pass on the parent (self in nao_main) as argument to access robot movement and posture
"""


import json
import sys

#Add SDK
with open('config.json', 'r') as f:
    config = json.load(f)
sys.path.append(config["filepath"])
from naoqi import ALProxy


def kick(parent):
    #EXAMPLE FUNCTION; NOT ACTUAL KICK ( ...yet ;] )
    parent.posture.goToPosture("StandInit", 1)
    pass