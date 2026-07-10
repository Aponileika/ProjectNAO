import numpy as np
import random
import json
from math import *

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class dubinsPath(Node):
    def __init__(self, Ts = 0.05):
        super().__init__("path_planning")
        self.get_logger().info("Path Planning Node Started")

        self.path_publsiher = self.create_publisher(String, "planned_path", 1)

        self.pos_sub = self.create_subscription(
            Pose2D,
            "pose",
            self.pos_callback,
            10
        )

        self.Ts = Ts

        self.x = 0
        self.y = 0
        self.theta = 0

        self.plannedPath = []

        ##FOR TEST PURPOSES; REPLACE WITH ACTION
        self.xGoal = 4
        self.yGoal = 4
        self.thetaGoal = -np.pi/2

        self.path_planned = False


    def pos_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        #FOR TEST PURPOSES; REPLACE WITH ACTION
        if not self.path_planned:
            self.plennedPath = self.dubinsPath(np.array([self.x, self.y]), self.theta,
                                               np.array([self.xGoal, self.yGoal]), self.thetaGoal)

            msg = String()
            data = self.plannedPath
            msg.data = json.dumps(data)
            self.path_publsiher.publish(msg)

            self.path_planned = True
            self.get_logger().info("PATH PLANNED")


    def dubinsPath(self, startPos, startAngle, endPos, endAngle, debug=False):
        
        spacing = 0.05 #Distance between each point in path
        L = 0.285
        deltaMax = np.pi/4
        R = L / np.tan(deltaMax)

        startEndVector = endPos - startPos
        startEndNorm = startEndVector /np.linalg.norm(startEndVector)

        perp = self.rot(np.array([np.cos(startAngle), np.sin(startAngle)]))
        circleStartCCW = startPos + R * perp
        circleStartCW = startPos - R * perp

        perp = self.rot(np.array([np.cos(endAngle), np.sin(endAngle)]))
        circleEndCCW = endPos + R * perp
        circleEndCW = endPos - R * perp

        LSL = {"startTang": circleStartCCW + R*self.rot(-startEndNorm), "endTang": circleEndCCW + R*self.rot(-startEndNorm), 
            "startPos":startPos, "endPos":endPos, "startCircle": circleStartCCW, "endCircle":circleEndCCW,
            "startDir":1, "endDir":1}
        LSL["length"] = self.getPathLength(LSL, R)

        RSR = {"startTang": circleStartCW - R*self.rot(-startEndNorm), "endTang": circleEndCW - R*self.rot(-startEndNorm),
            "startPos":startPos, "endPos":endPos, "startCircle": circleStartCW, "endCircle":circleEndCW,
            "startDir":-1, "endDir":-1}
        RSR["length"] = self.getPathLength(RSR, R)
        
        #LSR
        LSR = False
        D = np.linalg.norm(circleEndCW - circleStartCCW)
        if D >= 2*R:
            #Find internal tangents
            phi = np.atan2(circleEndCW[1]-circleStartCCW[1], circleEndCW[0]-circleStartCCW[0])
            alpha = np.arccos(2*R / D)
            theta = phi - alpha

            LSR = {"startTang":circleStartCCW+R*np.array([np.cos(theta), np.sin(theta)]),
                "endTang":  circleEndCW-R*np.array([np.cos(theta), np.sin(theta)]),
                "startPos":startPos, "endPos":endPos,
                "startCircle": circleStartCCW, "endCircle":circleEndCW,
                "startDir":1, "endDir":-1}
            LSR['length'] = self.getPathLength(LSR, R)
        
        #RSL
        RSL = False
        D = np.linalg.norm(circleEndCCW - circleStartCW)
        if D >= 2*R:
            #Find internal tangents
            phi = np.atan2(circleEndCCW[1]-circleStartCW[1], circleEndCCW[0]-circleStartCW[0])
            alpha = np.arccos(2*R / D)
            theta = phi + alpha

            RSL = {"startTang":circleStartCW+R*np.array([np.cos(theta), np.sin(theta)]),
                "endTang":  circleEndCCW-R*np.array([np.cos(theta), np.sin(theta)]),
                "startPos":startPos, "endPos":endPos,
                "startCircle": circleStartCW, "endCircle":circleEndCCW,
                "startDir":-1, "endDir":1}  
            RSL['length'] = self.getPathLength(RSL, R)

        shortestPath = {'length':np.inf}
        paths = [LSL, RSR, LSR, RSL]
        for path in paths:
            if path:
                if path['length'] < shortestPath['length']:
                    shortestPath = path

        dubinPath = self.createPath(shortestPath, R, spacing)
        
        if debug:
            fig, ax = plt.subplots()
            ax.set_aspect('equal')
            ax.set_xlim(-6,6)
            ax.set_ylim(-6,6)

            ax.quiver(startPos[0], startPos[1], np.cos(startAngle), np.sin(startAngle), color='blue')
            ax.quiver(endPos[0], endPos[1], np.cos(endAngle), np.sin(endAngle), color='red')

            perp = self.rot(np.array([np.cos(startAngle), np.sin(startAngle)]))
            circle = Circle(circleStartCCW,R,fill=False,linewidth=1)
            ax.add_patch(circle)
            circle = Circle(circleStartCW,R,fill=False,linewidth=1)
            ax.add_patch(circle)

            perp = self.rot(np.array([np.cos(endAngle), np.sin(endAngle)]))
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


    def createPath(self, path, R, spacing):
        sampledPath = []

        #Sample first arc
        print("arc1")
        start_angle = np.atan2(path['startPos'][1]-path['startCircle'][1], path['startPos'][0]-path['startCircle'][0])
        end_angle = np.atan2(path['startTang'][1]-path['startCircle'][1], path['startTang'][0]-path['startCircle'][0])
        print(self.sampleArc(start_angle, end_angle, path['startDir'], R, path["startCircle"], spacing))
        sampledPath.extend(self.sampleArc(start_angle, end_angle, path['startDir'], R, path["startCircle"], spacing))

        #Sample tangent
        print("tang")
        sampledPath.extend(self.sampleTangent(path, spacing))

        #Sample second arc
        print("arc2")
        start_angle = np.atan2(path['endTang'][1]-path['endCircle'][1], path['endTang'][0]-path['endCircle'][0])
        end_angle = np.atan2(path['endPos'][1]-path['endCircle'][1], path['endPos'][0]-path['endCircle'][0])
        sampledPath.extend(self.sampleArc(start_angle, end_angle, path['endDir'], R, path["endCircle"], spacing))

        return sampledPath

    def sampleArc(self, startAngle, endAngle, dir, R, center, spacing):
        arcLength = R * self.mod2pi(dir * (endAngle - startAngle))
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


    def getPathAngle(self, path):
        distance = 0
        for i in range(len(path) - 1):
            start_node = path[i]
            next_node = path[i+1]

            angle = atan2(next_node["y"] - start_node["y"],
                        next_node["x"] - start_node["x"])
            
            distance += sqrt( (next_node['x'] - start_node['x'])**2 + (next_node['x'] - start_node['x'])**2 )
            
            next_node["distance"] = distance
            next_node["theta"] = angle


    def getPathLength(self, path, R):
        tangentLength = np.sqrt( (path['endTang'][0]-path['startTang'][0])**2 + (path['endTang'][1]-path['startTang'][1])**2 )

        start_angle = np.atan2(path['startPos'][1]-path['startCircle'][1], path['startPos'][0]-path['startCircle'][0])
        end_angle = np.atan2(path['startTang'][1]-path['startCircle'][1], path['startTang'][0]-path['startCircle'][0])
        arcLengthStart = R * self.mod2pi(path['startDir'] * (end_angle - start_angle))

        start_angle = np.atan2(path['endTang'][1]-path['endCircle'][1], path['endTang'][0]-path['endCircle'][0])
        end_angle = np.atan2(path['endPos'][1]-path['endCircle'][1], path['endPos'][0]-path['endCircle'][0])
        arcLengthEnd = R * self.mod2pi(path['endDir'] * (end_angle - start_angle))

        print(2*R*np.pi)
        print(arcLengthStart, tangentLength, arcLengthEnd)

        return arcLengthStart + tangentLength + arcLengthEnd


    def rot(self, vector):
        return np.array([-vector[1], vector[0]])


    def mod2pi(self, theta):
        return theta % (2*np.pi)


def main(args=None):
    rclpy.init(args=args)

    node = dubinsPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    startPos = np.array([random.randint(-4,4), random.randint(-4,4)])
    startAngle = random.uniform(0,2*np.pi)

    endPos = np.array([random.randint(-4,4), random.randint(-4,4)])
    endAngle = random.uniform(0,2*np.pi)

    dubinsPath(startPos, startAngle, endPos, endAngle, debug = True)