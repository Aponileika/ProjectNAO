import numpy as np
import heapq
from scipy.ndimage import binary_dilation
from collections import deque
from math import sin, cos, sqrt

from .dubinPath import *


class Node():
    def __init__(self, x, y, theta, steering=0, parent=None, g=0.0, h=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.steering = steering

        self.parent = parent
        self.g = g
        self.h = h

    @property
    def f(self):
        return self.g + self.h


class HybridAStar():
    def __init__(self):
        self.Vmax = 1
        self.deltaMax = np.pi/6
        N = 5
        self.steeringAngles = np.linspace(-self.deltaMax, self.deltaMax, N)

        self.mapGrid = None
        self.distanceGrid = None
        self.width = None
        self.height = None

        self.xy_resolution = 0.05
        self.theta_resolution = np.deg2rad(30)
        self.map_resolution = 0.05

        self.xy_tolerance = 0.1
        self.theta_tolerance = np.deg2rad(10)

        self.L = 0.285
        self.track = 0.15
        self.radius = 1.2*max(self.L, self.track)/2

        self.propogationDistance = 0.2
        self.propogationInterval = 0.04
        self.dThetas = np.tan(self.steeringAngles) / self.L


    def inflate_map(self, grid):
        """Inflate obstacles by the radius of the circular vehicle footprint."""

        # Convert the physical radius from meters to map cells.
        cells = int(np.ceil(self.radius / self.map_resolution)) + 1

        # Construct a circular binary mask for the vehicle footprint.
        y, x = np.ogrid[-cells:cells + 1, -cells:cells + 1]
        footprint = x**2 + y**2 <= cells**2

        # Dilate every obstacle using the circular footprint.
        return binary_dilation(grid != 0, structure=footprint)


    def createDistanceGrid(self, goal):
        distanceGrid = np.full((self.height, self.width), np.inf)
        gx, gy = self.worldToGrid(goal.x, goal.y)

        queue = deque([(gx, gy)])
        distanceGrid[gy, gx] = 0

        while queue:
            x, y = queue.popleft()
            distance = distanceGrid[y, x] + self.xy_resolution

            for nx, ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                    continue
                if self.mapGrid[ny, nx]:
                    continue
                if distanceGrid[ny, nx] != np.inf:
                    continue

                distanceGrid[ny, nx] = distance
                queue.append((nx, ny))

        return distanceGrid


    @staticmethod
    def wrap_angle(theta):
        """Wrap an angle to [-pi, pi]."""
        return (theta + np.pi) % (2 * np.pi) - np.pi


    def worldToGrid(self, x, y):
        gx = int(x / self.xy_resolution)
        gy = int(y / self.xy_resolution)

        return gx, gy


    def stateKey(self, x, y, theta):
        x_index = int(round(x / self.xy_resolution))
        y_index = int(round(y / self.xy_resolution))
        theta_index = int(round(self.wrap_angle(theta) / self.theta_resolution))

        return x_index, y_index, theta_index


    def isColliding(self, trajectory):
        for point in trajectory:
            x = point[0]
            y = point[1]
            gx, gy = self.worldToGrid(x, y)
            if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
                return True
            if self.mapGrid[gy,gx]:
                return True
        return False


    def heuristic(self, startNode, goalNode):
        # gx, gy  = self.worldToGrid(startNode.x, goalNode.y)
        # return self.distanceGrid[gy, gx]**2
        return (startNode.y - goalNode.y)**2 + (startNode.x - goalNode.x)**2


    def reconstruct_path(self, node):
        """Follow parent pointers from the goal back to the start."""
        path = []

        while node is not None:
            path.append((node.x, node.y, node.theta))
            node = node.parent

        return path
        return path[::-1]


    def propogate(self, node, dTheta):
        x = node.x
        y = node.y
        theta = node.theta 

        dir = -1

        result = (x, y, theta)
        travelled = 0
        while abs(travelled) < self.propogationDistance:
            ds = dir * min(self.propogationDistance-travelled, dir * self.propogationInterval)
            x += ds * cos(theta)
            y += ds * sin(theta)

            gx, gy = self.worldToGrid(x,y)
            if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
                return None
            if self.mapGrid[gy, gx]:
                return None

            theta += ds * dTheta

            result = (x,y,theta)
            travelled += ds

        result = (result[0], result[1], self.wrap_angle(result[2]))
        return result


    def isFinished(self, node, goal):
        ex = abs(node.x - goal.x)
        ey = abs(node.y - goal.y)
        etheta = abs(self.wrap_angle(node.theta-goal.theta))

        if ex < self.xy_tolerance and ey < self.xy_tolerance and etheta < self.theta_tolerance:
            return True
        return False


    def getShortestDubin(self, paths, pathLengths, lowerLimit):
        colliding = []
        for dubin in paths:
            colliding.append(self.isColliding(dubin))

        shortestPath = None
        shortestLength = lowerLimit * self.propogationDistance
        for dubin, pathLength, collision in zip(paths, pathLengths, colliding):
            if not collision and pathLength < shortestLength:
                shortestLength = pathLength
                shortestPath = dubin

        return shortestPath


    def removeDupes(self, trajectory):
        i = 0
        while i < len(trajectory)-1:
            if trajectory[i] == trajectory[i+1]:
                trajectory.pop(i+1)
            else:
                i += 1
        return trajectory


    def addDistance(self, trajectory):
        dist = 0
        trajectory[0] = (trajectory[0][0], trajectory[0][1], trajectory[0][2], dist)
        for i in range(1, len(trajectory)):
            curr = trajectory[i]
            past = trajectory[i-1]
            dist += sqrt( (curr[0]-past[0])**2 + (curr[1]-past[1])**2 )

            trajectory[i] = (curr[0], curr[1], curr[2], dist)

        return trajectory


    def addDubinPaths(self, path, goalNode):
        """
        Function that looks for shortcuts in the form of dubin paths.
        """

        # Add the start manually, since the search might be off
        # Won't make it perfect, but might make it better if a dubin path is found
        path[0] = (goalNode.x, goalNode.y, self.wrap_angle(goalNode.theta+np.pi))

        lower = 0
        upper = len(path) - 1

        maxIter = len(path)
        iterations = 0

        while lower + 1 < upper and iterations < len(path):
            iterations += 1
            if iterations >= maxIter:
                print("WARNING ; MAX ITERATIONS REACHED")
                print(f"Lower: {lower}, Upper: {upper}")   

            # Calculate upper path
            upperDubin = None
            staticPoint = path[upper]
            for i in range(lower, upper - 1):
                dynamicPoint = path[i]
                dubins, dubinLengths = dubinsPath(np.array([dynamicPoint[0], dynamicPoint[1]]), dynamicPoint[2], 
                                             np.array([staticPoint[0], staticPoint[1]]), staticPoint[2], getDistance=False)
                shortestPath = self.getShortestDubin(dubins, dubinLengths, upper - i)

                if shortestPath != None:
                    upperDubin = shortestPath
                    upperIndex = i
                    break

            # Calculate lower path
            lowerDubin = None
            staticPoint = path[lower]
            for i in range(upper, lower + 1, -1):
                dynamicPoint = path[i]
                dubins, dubinLengths = dubinsPath(np.array([staticPoint[0], staticPoint[1]]), staticPoint[2], 
                                             np.array([dynamicPoint[0], dynamicPoint[1]]), dynamicPoint[2], getDistance=False)
                shortestPath = self.getShortestDubin(dubins, dubinLengths, i - lower)

                if shortestPath != None:
                    lowerDubin = shortestPath
                    lowerIndex = i
                    break              

            if lowerDubin is None:
                lower += 1

            # Add the one that shortens the path most, or the one that exists, provided they are shortcuts
            if upperDubin != None and lowerDubin != None:
                if (upper - upperIndex) - len(upperDubin) >= (lowerIndex - lower) - len(lowerDubin):
                    path = path[:upperIndex] + upperDubin[:-1] + path[upper:]
                    upper = upperIndex
                else:
                    path = path[:lower] + lowerDubin[:-1] + path[lowerIndex:]
                    lower = len(lowerDubin)
                    upper += len(lowerDubin) - lowerIndex - 1

            elif upperDubin != None and lowerDubin == None:
                path = path[:upperIndex] + upperDubin[:-1] + path[upper:]
                upper = upperIndex
            elif upperDubin == None and lowerDubin != None:
                path = path[:lower] + lowerDubin[:-1] + path[lowerIndex:]
                lower = len(lowerDubin)
                upper += len(lowerDubin) - lowerIndex - 1

        return path
        

    def search(self, start, goal, map=None, xlim=None, ylim=None):
        if map is None:
            validPaths, validLengths = dubinsPath( (start[0], start[1]), start[2], (goal[0], goal[1]), goal[2], getDistance=True)
            shortestPath = None
            shortestPathLength = np.inf
            for path, length in zip(validPaths, validLengths):
                if length < shortestPathLength:
                    shortestPath = path
                    shortestPathLength = length

            return shortestPath

        minX = xlim[0]
        minY = ylim[0]

        startNode = Node(start[0] - minX, start[1] - minY, self.wrap_angle(start[2]))
        goalNode = Node(goal[0] - minX, goal[1] - minY, self.wrap_angle(goal[2]))

        startNode, goalNode = goalNode, startNode
        startNode.theta = self.wrap_angle(startNode.theta + np.pi)
        goalNode.theta = self.wrap_angle(goalNode.theta + np.pi)

        self.mapGrid = self.inflate_map(map)
        self.height, self.width = map.shape

        if self.isColliding([(startNode.x, startNode.y)]) or self.isColliding([(goalNode.x, goalNode.y)]):
            print("Start or Goal is inside object")
            return None

        # self.distanceGrid = np.copy(self.mapGrid)
        # self.distanceGrid = self.createDistanceGrid(goalNode)
        # print("Distance map done")

        startNode.h = self.heuristic(startNode, goalNode)
        openSet = []
        counter = 0
        heapq.heappush(openSet, (startNode.f, counter, startNode))


        nx = self.width
        ny = self.height
        ntheta = int(round(2*np.pi / self.theta_resolution))
        best_gs = np.full((nx, ny, ntheta), np.inf)

        xi, yi, ti = self.stateKey(startNode.x, startNode.y, startNode.theta)
        best_gs[xi, yi, ti] = startNode.g

        iterations = 0
        maxIterations = 1e6

        while openSet:
            iterations += 1
            if iterations > maxIterations:
                print("MAX ITERATIONS REACHED")
                return None

            _, _, current = heapq.heappop(openSet)
            key = self.stateKey(current.x, current.y, current.theta)
            if current.g > best_gs[key]:
                continue

            if self.isFinished(current, goalNode):
                print("Path found, searching for shortcuts")
                path = self.reconstruct_path(current)
                for i in range(len(path)):
                    point = path[i]
                    path[i] = (point[0], point[1], self.wrap_angle(point[2] + np.pi)) #Since the search is backwards, add turn every point around.

                path = self.addDubinPaths(path, goalNode)
                path = self.removeDupes(path)
                path = self.addDistance(path)
                for i in range(len(path)):
                    point = path[i]
                    path[i] = (point[0]+xlim[0], point[1]+ylim[0], point[2], point[3])
                return path

            for dTheta, steeringAngle in zip(self.dThetas, self.steeringAngles):
                newPoint = self.propogate(current, dTheta)
                if newPoint is None:
                    continue

                newX, newY, newTheta = newPoint
                xi, yi, ti = self.stateKey(newX, newY, newTheta)

                newg = current.g + self.propogationDistance**2
                if newg >= best_gs[xi, yi, ti]:
                    continue
                best_gs[xi, yi, ti] = newg

                newNode =  Node(newX, newY, newTheta, steeringAngle, parent=current, g=newg)
                newNode.h = self.heuristic(newNode, goalNode)
                heapq.heappush(
                    openSet,
                    (newNode.f, counter, newNode)
                )

                counter += 1
                

if __name__ == "__main__":
    import random
    import matplotlib.pyplot as plt
    import time
    import threading

    def add_rectangle_obstacle(grid, obstacle, xlim, ylim, resolution):
        """Add a rectangular obstacle defined in world coordinates to the occupancy grid."""
        x_min, x_max, y_min, y_max = obstacle

        x0 = int((x_min - xlim[0]) / resolution)
        x1 = int((x_max - xlim[0]) / resolution)
        y0 = int((y_min - ylim[0]) / resolution)
        y1 = int((y_max - ylim[0]) / resolution)

        x0 = max(0, x0)
        x1 = min(grid.shape[1], x1)
        y0 = max(0, y0)
        y1 = min(grid.shape[0], y1)

        grid[y0:y1, x0:x1] = 1
        return grid

    xlim = (-5,5)
    ylim = xlim
    xy_resolution = 0.05

    size = int((xlim[1]-xlim[0])/xy_resolution)
    map = np.zeros((size,size))

    obstacles = [
        (-0.5, 0.5, -3, 3),
        (-3, -2, -3, -2),
        (2,3,2,3),
        (-3,-2,2,3),
        (2,3,-3,-2)
    ]
    for obstacle in obstacles:
        map = add_rectangle_obstacle(map, obstacle, xlim, ylim, xy_resolution)

        hybridAStar = HybridAStar()

    fig, ax = plt.subplots()
    plt.show(block=False)

    while True:
        ax.clear()

        start = (random.uniform(-4, -1), random.uniform(-4, 4), random.uniform(-np.pi, np.pi))
        goal = (random.uniform(1, 4), random.uniform(-4, 4), random.uniform(-np.pi, np.pi))

        #start = (-3, 0, 0)
        #goal = (3, 0, np.pi)

        print("Search started")
        t0 = time.time()
        path = hybridAStar.search(start, goal, map, xlim, ylim)
        print("Search finished.")

        if path is not None:
            for point in path:
                pass
                #print((float(point[0]), float(point[1]), 180/np.pi * float(point[2])))
            print(f"    Time:  {time.time() - t0}")
            print(f"    Start: {start}")
            print(f"    Goal:  {goal}")
            print(f"    Final: {float(path[-1][0] + xlim[0]), float(path[-1][1] + ylim[0]), float(path[-1][2])}")

            for i in range(1, len(path)):
                if path[i] == path[i - 1]:
                    print("    WARNING: DUPLICATES")
                    print((float(path[i][0]+xlim[0]), float(path[i][1]+ylim[0])))

        for x_min, x_max, y_min, y_max in obstacles:
            ax.fill(
                [x_min, x_max, x_max, x_min],
                [y_min, y_min, y_max, y_max],
                alpha=0.5
            )

        if path is not None:
            path = np.array(path)

            ax.plot(
                path[:, 0] + xlim[0],
                path[:, 1] + ylim[0],
                "-o",
                markersize=3,
                label="Hybrid A* path"
            )

        ax.quiver(start[0], start[1], np.cos(start[2]), np.sin(start[2]),
                  angles="xy", scale_units="xy", scale=1, color="green", label="Start" )
        ax.quiver(goal[0], goal[1], np.cos(goal[2]), np.sin(goal[2]),
                  angles="xy", scale_units="xy", scale=1, color="red", label="Goal")
        
        ax.scatter(start[0], start[1], color="green")
        ax.scatter(goal[0], goal[1], color="red")

        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid()
        ax.legend()

        plt.pause(0.01)

        threading.Thread(target=input, daemon=True).start()
        while threading.active_count() > 1:
            plt.pause(0.01)
        