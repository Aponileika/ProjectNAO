import numpy as np
import random
import json
from math import *

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

    
def dubinsPath(startPos, startAngle, endPos, endAngle, getDistance=True, debug=False):
    
    spacing = 0.2 #Distance between each point in path
    L = 0.285
    deltaMax = np.pi/6
    multiplierR = 1.0 #Increase R for more robust turn
    R = multiplierR * L / np.tan(deltaMax)
    zeroVec = np.array([0, 0])

    perp = rot(np.array([np.cos(startAngle), np.sin(startAngle)]))
    circleStartCCW = startPos + R * perp
    circleStartCW = startPos - R * perp

    perp = rot(np.array([np.cos(endAngle), np.sin(endAngle)]))
    circleEndCCW = endPos + R * perp
    circleEndCW = endPos - R * perp

    
    if (circleEndCCW == circleStartCCW).all():
        LSL = False
    else:
        circleVector = circleEndCCW - circleStartCCW
        circleNorm = circleVector / np.linalg.norm(circleVector)
        LSL = {
            "startTang": circleStartCCW + R*rot(-circleNorm),
            "endTang": circleEndCCW + R*rot(-circleNorm),
            "startPos": startPos,
            "endPos": endPos,
            "startCircle": circleStartCCW,
            "endCircle": circleEndCCW,
            "startDir": 1,
            "endDir": 1
        }
        LSL["length"] = getPathLength(LSL, R)

    
    if (circleEndCW == circleStartCW).all():
        RSR = False
    else:
        circleVector = circleEndCW - circleStartCW
        circleNorm = circleVector / np.linalg.norm(circleVector)
        RSR = {
            "startTang": circleStartCW + R*rot(circleNorm),
            "endTang": circleEndCW + R*rot(circleNorm),
            "startPos": startPos,
            "endPos": endPos,
            "startCircle": circleStartCW,
            "endCircle": circleEndCW,
            "startDir": -1,
            "endDir": -1
        }
        RSR["length"] = getPathLength(RSR, R)
    
    #LSR
    LSR = False
    Dvec = circleEndCW - circleStartCCW
    D2 = np.dot(Dvec, Dvec)
    if D2 >= (2*R)**2:
        D = sqrt(D2)

        #Find internal tangents
        phi = atan2(circleEndCW[1]-circleStartCCW[1], circleEndCW[0]-circleStartCCW[0])
        alpha = np.arccos(2*R / D)
        theta = phi - alpha

        LSR = {"startTang":circleStartCCW+R*np.array([cos(theta), sin(theta)]),
            "endTang":  circleEndCW-R*np.array([cos(theta), sin(theta)]),
            "startPos":startPos, "endPos":endPos,
            "startCircle": circleStartCCW, "endCircle":circleEndCW,
            "startDir":1, "endDir":-1}
        LSR['length'] = getPathLength(LSR, R)
    
    #RSL
    RSL = False
    Dvec = circleEndCCW - circleStartCW
    D2 = np.dot(Dvec, Dvec)
    if D2 >= (2*R)**2:
        D = sqrt(D2)

        #Find internal tangents
        phi = atan2(circleEndCCW[1]-circleStartCW[1], circleEndCCW[0]-circleStartCW[0])
        alpha = np.arccos(2*R / D)
        theta = phi + alpha

        RSL = {"startTang":circleStartCW+R*np.array([cos(theta), sin(theta)]),
            "endTang":  circleEndCCW-R*np.array([cos(theta), sin(theta)]),
            "startPos":startPos, "endPos":endPos,
            "startCircle": circleStartCW, "endCircle":circleEndCCW,
            "startDir":-1, "endDir":1}  
        RSL['length'] = getPathLength(RSL, R)

    paths = [LSL, RSR, LSR, RSL]
    validPaths = []
    pathLengths = []
    for path in paths:
        if path:
            validPaths.append(path)
            pathLengths.append(path['length'])

    for i in range(len(validPaths)):
        validPaths[i] = createPath(validPaths[i], R, spacing, getDistance)
        validPaths[i][0] = (validPaths[i][0][0], validPaths[i][0][1], startAngle)
    
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

    if not dubinsPath:
        return None

    return validPaths, pathLengths


def createPath(path, R, spacing, getDistance=True):
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

    #Get the distances
    if getDistance:
        sampledPath = getPathDistance(sampledPath)

    return sampledPath

def sampleArc(startAngle, endAngle, dir, R, center, spacing):
    arcLength = R * mod2pi(dir * (endAngle - startAngle))
    sampleCount = max(1, round(arcLength / spacing))
    dTheta = arcLength / (R * sampleCount)

    sampledPath = []
    for i in range(sampleCount+1):
        circleAngle = startAngle + dir*i*dTheta
        sampledPath.append( (center[0] + R*cos(circleAngle),
                             center[1] + R*sin(circleAngle),
                             mod2pi(circleAngle + dir*pi/2) ) )

    return sampledPath


def sampleTangent(path, spacing):
    dx = path['endTang'][0]-path['startTang'][0]
    dy = path['endTang'][1]-path['startTang'][1]

    tangentLength = np.sqrt( dx**2 + dy**2 )
    sampleCount = max(1, round(tangentLength / spacing))

    dx /= sampleCount
    dy /= sampleCount
    theta = atan2(dy,dx)
    
    sampledPath = []
    for i in range(1, sampleCount+1):
        sampledPath.append( (path["startTang"][0] + dx*i,
                             path["startTang"][1] + dy*i,
                             theta) )

    return sampledPath


def getPathDistance(path):
    distance = 0
    for i in range(len(path) - 1):
        start_node = path[i]
        next_node = path[i+1]
        
        distance += sqrt( (next_node[0] - start_node[0])**2 + (next_node[1] - start_node[1])**2 )
        path[i+1] = (next_node[0], next_node[1], next_node[2], distance)

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