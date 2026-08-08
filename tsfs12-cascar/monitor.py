import json
import threading
import socket
import time
import matplotlib.pyplot as plt
import numpy as np


class Monitor():
    def __init__(self, Ts):
        self.Ts = Ts

        self.currEstX = 0
        self.currEstY = 0
        self.currEstTheta = 0

        self.pastEstX = []
        self.pastEstY = []
        self.pastEstTheta = []

        self.currTrueX = 0
        self.currTrueY = 0
        self.currTrueTheta = 0

        self.pastTrueX = []
        self.pastTrueY = []
        self.pastTrueTheta = []

        self.path = []

        self.fig, self.ax = plt.subplots()

        # Trajectories
        self.est_path, = self.ax.plot([], [], "b-", label="Estimated")
        self.true_path, = self.ax.plot([], [], "g-", label="True")
        self.plan_path, = self.ax.plot([], [], "k--", label="Planned")

        # Current positions
        self.est_point, = self.ax.plot([], [], "bo")
        self.true_point, = self.ax.plot([], [], "go")

        # Heading arrows
        self.est_arrow = self.ax.quiver(
            [], [], [], [],
            color="b",
            angles="xy",
            scale_units="xy",
            scale=1
        )

        self.true_arrow = self.ax.quiver(
            [], [], [], [],
            color="g",
            angles="xy",
            scale_units="xy",
            scale=1
        )

        self.ax.set_aspect("equal")
        self.ax.grid(True)
        self.ax.legend()

        plt.ion()
        plt.show()
    
        self.cascarIP = "192.168.0.112"
        self.cascarPort = 5001
        self.cascarSocket = None
        self.connectCascar()

        # self.qualisysIP = ""
        # self.qualisysPort = 0
        # self.qualisysSocket = None
        # self.connectQualisys()

        self.socketLock = threading.Lock()

        threading.Thread(target=self.cascarData, daemon=True).start()
        #threading.Thread(target=self.qualisysData, daemon=True).start()
        threading.Thread(target=self.requestData, daemon=True).start()
        threading.Thread(target=self.updatePlot, daemon=True).start()


    def connectCascar(self):
        self.cascarSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cascarSocket.connect((self.cascarIP, self.cascarPort))
        print("CasCar connected")

    def connectQualisys(self):
        self.qualisysSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.qualisysSocket.connect((self.qualisysIP, self.qualisysPort))
        print("Qualisys connected")


    def updatePlot(self):

        arrow_length = 0.25

        while True:

            # Estimated trajectory
            self.est_path.set_data(self.pastEstX, self.pastEstY)

            # True trajectory
            self.true_path.set_data(self.pastTrueX, self.pastTrueY)

            # Planned path
            if self.path:
                xs = [p[0] for p in self.path]
                ys = [p[1] for p in self.path]
                self.plan_path.set_data(xs, ys)

            # Current positions
            self.est_point.set_data([self.currEstX], [self.currEstY])
            self.true_point.set_data([self.currTrueX], [self.currTrueY])

            # Estimated heading
            self.est_arrow.set_offsets(
                [[self.currEstX, self.currEstY]]
            )

            self.est_arrow.set_UVC(
                arrow_length * np.cos(self.currEstTheta),
                arrow_length * np.sin(self.currEstTheta)
            )

            # True heading
            self.true_arrow.set_offsets(
                [[self.currTrueX, self.currTrueY]]
            )

            self.true_arrow.set_UVC(
                arrow_length * np.cos(self.currTrueTheta),
                arrow_length * np.sin(self.currTrueTheta)
            )

            # Keep axes fitted to data
            xs = self.pastEstX + self.pastTrueX
            ys = self.pastEstY + self.pastTrueY

            if self.path:
                xs += [p[0] for p in self.path]
                ys += [p[1] for p in self.path]

            if xs:
                margin = 0.5
                self.ax.set_xlim(min(xs)-margin, max(xs)+margin)
                self.ax.set_ylim(min(ys)-margin, max(ys)+margin)

            self.fig.canvas.draw_idle()
            plt.pause(self.Ts)


    def cascarData(self):
        buffer = ""
        while True:
            data = self.cascarSocket.recv(1024)
            if not data:
                break
            buffer += data.decode()

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                msg = json.loads(line)
                if "pose" in msg:
                    pose = msg['pose']
                    x, y, theta = pose['x'], pose['y'], pose['theta']

                    self.currEstX = x
                    self.currEstY = y
                    self.currEstTheta = theta

                    self.pastEstX.append(x)
                    self.pastEstY.append(y)
                    self.pastEstTheta.append(theta)

                if "path" in msg:
                    self.path = msg['path']


    def qualisysData(self, debug=False):
        buffer = ""
        while True:
            data = self.qualisysSocket.recv(1024)

            if not data:
                break
            buffer += data.decode()

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                values = line.split()

                if not values:
                    continue

                if values[0] == "POSE":
                    self.currTrueX = float(values[1]) / 1000
                    self.currTrueY = float(values[2]) / 1000
                    self.currTrueTheta = np.deg2rad(float(values[6]))

                    self.pastTrueX.append(self.currTrueX)
                    self.pastTrueY.append(self.currTrueY)
                    self.pastTrueTheta.append(self.currTrueTheta)

                    if debug:
                        print(
                            f"QTM: x={self.currTrueX:.2f}, "
                            f"y={self.currTrueY:.2f}, "
                            f"theta={self.currTrueTheta:.2f}"
                        )


    def requestData(self):
        while True:
            command = input("Command: ").lower()

            if command == "getpath" or command == "path":
                command = {
                    "command":"getPath"
                }
            else:
                command = None

            if command != None:
                msg = json.dumps(command) + "\n"

                with self.socketLock:
                    self.cascarSocket.sendall(
                        msg.encode()
                    )


if __name__ == "__main__":
    Ts = 1
    monitor = Monitor(Ts)

    while True:
        time.sleep(1)