import json
import threading
import socket
import time
from math import *

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cascar_msgs.msg import CarCommand, CarMeasurement
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class Diagnostics(Node):
    def __init__(self, Ts: float, HOST:str, PORT:int):
        super().__init__("diagnostics")

        self.get_logger().info("Diagnostics Node started")

        self.pos_sub = self.create_subscription(
            Pose2D,
            "pose",
            self.pos_callback,
            10
        )

        self.path_sub = self.create_subscription(
            String,
            "planned_path",
            self.path_callback,
            10
        )

        self.Ts = Ts

        self.lock = threading.Lock()
        self.socketLock = threading.Lock()

        self.HOST = HOST
        self.PORT = PORT
        self.poseBuffer = {}
        self.pathBuffer = {}

        threading.Thread(
            target=self.server_loop,
            daemon=True
        ).start()


    def pos_callback(self, msg):
        with self.lock:
            self.poseBuffer['x']=msg.x
            self.poseBuffer['y']=msg.y
            self.poseBuffer['theta']=msg.theta

    
    def path_callback(self, msg):
        with self.lock:
            self.pathBuffer['path']=json.loads(msg.data)
        

    def server_loop(self):

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.HOST, self.PORT))
        server.listen(1)

        self.get_logger().info(f"Listening on {self.HOST}:{self.PORT}")

        while True:

            conn, addr = server.accept()
            #Start listner thread
            threading.Thread(
                target=self.receive_loop,
                args=(conn,),
                daemon=True
            ).start()        

            self.get_logger().info(f"Connected by {addr}")

            try:
                while rclpy.ok():
                    with self.lock:
                        if not self.poseBuffer:
                            packet = None
                        else:
                            packet = json.dumps({'pose':self.poseBuffer.copy()})
                    if packet is not None:
                        with self.socketLock:
                            conn.sendall(packet.encode() + b"\n")

                    time.sleep(self.Ts)

            except Exception as e:
                self.get_logger().warning(f"Disconnected ({e})")

            finally:
                conn.close()


    def receive_loop(self, conn):
        buffer = ""

        while rclpy.ok():
            try:
                data = conn.recv(1024)
            except ConnectionError:
                break
            if not data:
                break

            buffer += data.decode()
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                command = json.loads(line)

                if command["command"] == "getPath":
                    with self.lock:
                        path = self.pathBuffer.get("path", None)

                    packet = json.dumps({"path":path})
                    with self.socketLock:
                        conn.sendall(packet.encode() + b"\n")

        
def main():
    Ts = 1/20
    HOST = "0.0.0.0"
    PORT = 5000

    rclpy.init()

    node = Diagnostics(Ts, HOST, PORT)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()