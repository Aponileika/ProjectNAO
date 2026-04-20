import math

import pygame
import rclpy
from pygame.locals import K_DOWN, K_LEFT, K_RIGHT, K_UP, QUIT, K_a, K_d, K_s, K_w
from rclpy.node import Node

from cascar_msgs.msg import CarCommand, ControlRequest


class KeyboardControl(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_control")
        self.get_logger().info("Keyboard Control Node Started")
        self._steer_cache: float = 0.0
        self._speed_cache: float = 0.0

        self.max_speed: float = 100.0
        self.max_steer: float = 95.0

        # Publishers
        self.sim_publisher_ = self.create_publisher(ControlRequest, "control", 1)
        self.car_publisher_ = self.create_publisher(CarCommand, "car_command", 1)

        # Initialize pygame
        self.fps: int = 60
        pygame.init()
        pygame.display.set_mode((400, 50))
        pygame.display.set_caption("Keyboard Controller")

    @property
    def speed(self) -> float:
        return self._speed_cache

    @property
    def steer(self) -> float:
        return self._steer_cache

    def timer_callback(self) -> None:
        car_msg = CarCommand()
        car_msg.speed = self.speed * self.max_speed
        car_msg.steer = self.steer * self.max_steer

        self.car_publisher_.publish(car_msg)

        sim_msg = ControlRequest()
        sim_msg.ax = self.speed * 1.0
        sim_msg.df = self.steer * math.pi / 4
        self.sim_publisher_.publish(sim_msg)

    def parse_vehicle_keys(
        self,
        keys: pygame.key.ScancodeWrapper,
        milliseconds: int,
    ) -> None:
        speed_increment = 1e-3 * milliseconds
        steer_increment = 2e-3 * milliseconds

        if keys[K_UP] or keys[K_w]:
            if self._speed_cache < 0:
                self._speed_cache = 0
            else:
                self._speed_cache += speed_increment

        elif keys[K_DOWN] or keys[K_s]:
            if self._speed_cache > 0:
                self._speed_cache = 0
            else:
                self._speed_cache -= speed_increment

        else:
            # get sign
            self._speed_cache *= 0.9
            self._speed_cache = (
                0.0
                if abs(self._speed_cache) < speed_increment
                else self._speed_cache
            )

        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment

        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment

        else:
            self._steer_cache *= 0.9
            self._steer_cache = (
                0.0
                if abs(self._steer_cache) < steer_increment
                else self._steer_cache
            )

        self._steer_cache = min(1.0, max(-1.0, self._steer_cache))
        self._speed_cache = min(1.0, max(-1.0, self._speed_cache))

    def get_controls(self) -> tuple[float, float]:
        return self.speed, self.steer

    def run(self) -> None:
        clock = pygame.time.Clock()
        running = True
        while rclpy.ok() and running:
            milliseconds = clock.tick(self.fps)

            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                    self.get_logger().info("Shutting down")
                    break

            keys = pygame.key.get_pressed()
            self.parse_vehicle_keys(keys, milliseconds)
            self.timer_callback()

            # Clear screen (optional for visualization)
            pygame.display.get_surface().fill((0, 0, 0))
            pygame.display.flip()

        pygame.quit()


def main(args=None) -> None:
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    try:
        keyboard_control.run()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
