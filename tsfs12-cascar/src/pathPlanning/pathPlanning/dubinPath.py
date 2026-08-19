import numpy as np
import random
import json
from math import *

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

    
def dubinsPath(startPos, startAngle, endPos, endAngle, debug=False):
    
    spacing = 0.1 #Distance between each point in path
    L = 0.285
    deltaMax = np.pi/6
    multiplierR = 1.2 #Increase R for more robust turn
    R = multiplierR * L / np.tan(deltaMax)

    startEndVector = endPos - startPos
    startEndNorm = startEndVector /np.linalg.norm(startEndVector)

    perp = rot(np.array([np.cos(startAngle), np.sin(startAngle)]))
    circleStartCCW = startPos + R * perp
    circleStartCW = startPos - R * perp

    perp = rot(np.array([np.cos(endAngle), np.sin(endAngle)]))
    circleEndCCW = endPos + R * perp
    circleEndCW = endPos - R * perp

    LSL = {"startTang": circleStartCCW + R*rot(-startEndNorm), "endTang": circleEndCCW + R*rot(-startEndNorm), 
        "startPos":startPos, "endPos":endPos, "startCircle": circleStartCCW, "endCircle":circleEndCCW,
        "startDir":1, "endDir":1}
    LSL["length"] = getPathLength(LSL, R)

    RSR = {"startTang": circleStartCW - R*rot(-startEndNorm), "endTang": circleEndCW - R*rot(-startEndNorm),
        "startPos":startPos, "endPos":endPos, "startCircle": circleStartCW, "endCircle":circleEndCW,
        "startDir":-1, "endDir":-1}
    RSR["length"] = getPathLength(RSR, R)
    
    #LSR
    LSR = False
    D = np.linalg.norm(circleEndCW - circleStartCCW)
    if D >= 2*R:
        #Find internal tangents
        phi = atan2(circleEndCW[1]-circleStartCCW[1], circleEndCW[0]-circleStartCCW[0])
        alpha = np.arccos(2*R / D)
        theta = phi - alpha

        LSR = {"startTang":circleStartCCW+R*np.array([np.cos(theta), np.sin(theta)]),
            "endTang":  circleEndCW-R*np.array([np.cos(theta), np.sin(theta)]),
            "startPos":startPos, "endPos":endPos,
            "startCircle": circleStartCCW, "endCircle":circleEndCW,
            "startDir":1, "endDir":-1}
        LSR['length'] = getPathLength(LSR, R)
    
    #RSL
    RSL = False
    D = np.linalg.norm(circleEndCCW - circleStartCW)
    if D >= 2*R:
        #Find internal tangents
        phi = atan2(circleEndCCW[1]-circleStartCW[1], circleEndCCW[0]-circleStartCW[0])
        alpha = np.arccos(2*R / D)
        theta = phi + alpha

        RSL = {"startTang":circleStartCW+R*np.array([np.cos(theta), np.sin(theta)]),
            "endTang":  circleEndCCW-R*np.array([np.cos(theta), np.sin(theta)]),
            "startPos":startPos, "endPos":endPos,
            "startCircle": circleStartCW, "endCircle":circleEndCCW,
            "startDir":-1, "endDir":1}  
        RSL['length'] = getPathLength(RSL, R)

    shortestPath = {'length':np.inf}
    paths = [LSL, RSR, LSR, RSL]
    for path in paths:
        if path:
            if path['length'] < shortestPath['length']:
                shortestPath = path

    dubinPath = createPath(shortestPath, R, spacing)
    
    if debug:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim(-6,6)
        ax.set_ylim(-6,6)

        ax.quiver(startPos[0], startPos[1], np.cos(startAngle), np.sin(startAngle), color='blue')
        ax.quiver(endPos[0], endPos[1], np.cos(endAngle), np.sin(endAngle), color='red')

        perp = rot(np.array([np.cos(startAngle), np.sin(startAngle)]))
        circle = Circle(circleStartCCW,R,fill=False,linewidth=1)
        ax.add_patch(circle)
        circle = Circle(circleStartCW,R,fill=False,linewidth=1)
        ax.add_patch(circle)

        perp = rot(np.array([np.cos(endAngle), np.sin(endAngle)]))
        circle = Circle(circleEndCCW,R,fill=False,linewidth=1)
        ax.add_patch(circle)
        circle = Circle(circleEndCW,R,fill=False,linewidth=1)
        ax.add_patch(circle)

        ax.plot([ LSL["startTang"][0], LSL["endTang"][0]], [LSL["startTang"][1], LSL["endTang"][1]], linestyle="--", color='orange')
        ax.plot([ RSR["startTang"][0], RSR["endTang"][0]], [RSR["startTang"][1], RSR["endTang"][1]], linestyle="--", color='lightblue')
        if LSR:
            ax.plot([ LSR["startTang"][0], LSR["endTang"][0]], [LSR["startTang"][1], LSR["endTang"][1]], linestyle="--")
        if RSL:
            ax.plot([ RSL["startTang"][0], RSL["endTang"][0]], [RSL["startTang"][1], RSL["endTang"][1]], linestyle="--")

        x = [p[0] for p in dubinPath]
        y = [p[1] for p in dubinPath]
        plt.plot(x, y, linestyle="-", color='green')

        fig.show()
        input()

    return dubinPath


def createPath(path, R, spacing):
    sampledPath = []

    #Sample first arc
    start_angle = atan2(path['startPos'][1]-path['startCircle'][1], path['startPos'][0]-path['startCircle'][0])
    end_angle = atan2(path['startTang'][1]-path['startCircle'][1], path['startTang'][0]-path['startCircle'][0])
    sampledPath.extend(sampleArc(start_angle, end_angle, path['startDir'], R, path["startCircle"], spacing))

    #Sample tangent
    sampledPath.extend(sampleTangent(path, spacing))

    #Sample second arc
    start_angle = atan2(path['endTang'][1]-path['endCircle'][1], path['endTang'][0]-path['endCircle'][0])
    end_angle = atan2(path['endPos'][1]-path['endCircle'][1], path['endPos'][0]-path['endCircle'][0])
    sampledPath.extend(sampleArc(start_angle, end_angle, path['endDir'], R, path["endCircle"], spacing))

    #Add distances and angles between points
    sampledPath = getPathAngle(sampledPath)

    return sampledPath

def sampleArc(startAngle, endAngle, dir, R, center, spacing):
    arcLength = R * mod2pi(dir * (endAngle - startAngle))
    sampleCount = round(arcLength / spacing)

    dTheta = arcLength / (R * sampleCount)

    sampledPath = []
    for i in range(sampleCount):
        sampledPath.append({"x": center[0] + R*np.cos(startAngle + dir*i*dTheta),
                            "y": center[1] + R*np.sin(startAngle + dir*i*dTheta) })

    return sampledPath


def sampleTangent(path, spacing):
    tangentLength = np.sqrt( (path['endTang'][0]-path['startTang'][0])**2 + (path['endTang'][1]-path['startTang'][1])**2 )
    sampleCount = round(tangentLength / spacing)

    dx = (path['endTang'][0]-path['startTang'][0]) / sampleCount
    dy = (path['endTang'][1]-path['startTang'][1]) / sampleCount
    
    sampledPath = []
    for i in range(sampleCount):
        sampledPath.append({'x':path["startTang"][0] + dx*i,
                            'y':path["startTang"][1] + dy*i })

    return sampledPath


def getPathAngle(path):
    distance = 0
    for i in range(len(path) - 1):
        start_node = path[i]
        next_node = path[i+1]

        angle = atan2(next_node["y"] - start_node["y"],
                    next_node["x"] - start_node["x"])
        
        distance += sqrt( (next_node['x'] - start_node['x'])**2 + (next_node['x'] - start_node['x'])**2 )
        
        next_node["distance"] = distance
        next_node["theta"] = angle

    return path


def getPathLength(path, R):
    tangentLength = np.sqrt( (path['endTang'][0]-path['startTang'][0])**2 + (path['endTang'][1]-path['startTang'][1])**2 )

    start_angle = atan2(path['startPos'][1]-path['startCircle'][1], path['startPos'][0]-path['startCircle'][0])
    end_angle = atan2(path['startTang'][1]-path['startCircle'][1], path['startTang'][0]-path['startCircle'][0])
    arcLengthStart = R * mod2pi(path['startDir'] * (end_angle - start_angle))

    start_angle = atan2(path['endTang'][1]-path['endCircle'][1], path['endTang'][0]-path['endCircle'][0])
    end_angle = atan2(path['endPos'][1]-path['endCircle'][1], path['endPos'][0]-path['endCircle'][0])
    arcLengthEnd = R * mod2pi(path['endDir'] * (end_angle - start_angle))

    return arcLengthStart + tangentLength + arcLengthEnd


def rot(vector):
    return np.array([-vector[1], vector[0]])


def mod2pi(theta):
    return theta % (2*np.pi)


if __name__ == "__main__":
    startPos = np.array([random.randint(-4,4), random.randint(-4,4)])
    startAngle = random.uniform(0,2*np.pi)

    endPos = np.array([random.randint(-4,4), random.randint(-4,4)])
    endAngle = random.uniform(0,2*np.pi)

    dubinsPath(startPos, startAngle, endPos, endAngle, debug = True)