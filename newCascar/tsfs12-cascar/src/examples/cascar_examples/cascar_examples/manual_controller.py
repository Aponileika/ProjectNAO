"""Manual keyboard-controller example node."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from cascar_interfaces.msg import ActuatorCommand


@dataclass
class VehicleController:
    """Convert held keys into normalized throttle and steering effort.

    The legacy speed field represents throttle effort, not a speed in m/s.
    """

    speed_rate: float = 1.0
    steer_rate: float = 2.0
    speed_return_rate: float = 1.5
    steer_return_rate: float = 4.0
    speed: float = 0.0
    steer: float = 0.0
    _pressed: set[str] = field(default_factory=set)

    def press(self, key: str) -> None:
        """Record a key press."""
        self._pressed.add(key.lower())

    def release(self, key: str) -> None:
        """Record a key release."""
        self._pressed.discard(key.lower())

    def stop(self) -> None:
        """Immediately stop and discard all held inputs."""
        self._pressed.clear()
        self.speed = 0.0
        self.steer = 0.0

    @staticmethod
    def _towards_zero(value: float, amount: float) -> float:
        if value > 0.0:
            return max(0.0, value - amount)
        return min(0.0, value + amount)

    def update(self, seconds: float) -> tuple[float, float]:
        """Advance the controls and return throttle and steering effort."""
        seconds = max(0.0, min(seconds, 0.25))
        forward = "w" in self._pressed or "up" in self._pressed
        reverse = "s" in self._pressed or "down" in self._pressed
        left = "a" in self._pressed or "left" in self._pressed
        right = "d" in self._pressed or "right" in self._pressed

        speed_direction = int(forward) - int(reverse)
        steer_direction = int(left) - int(right)

        if speed_direction:
            if self.speed * speed_direction < 0.0:
                self.speed = 0.0
            self.speed += speed_direction * self.speed_rate * seconds
        else:
            self.speed = self._towards_zero(
                self.speed,
                self.speed_return_rate * seconds,
            )

        if steer_direction:
            if self.steer * steer_direction < 0.0:
                self.steer = 0.0
            self.steer += steer_direction * self.steer_rate * seconds
        else:
            self.steer = self._towards_zero(
                self.steer,
                self.steer_return_rate * seconds,
            )

        self.speed = max(-1.0, min(1.0, self.speed))
        self.steer = max(-1.0, min(1.0, self.steer))
        return self.speed, self.steer


class KeyboardControl(Node):
    """Publish normalized vehicle commands from keyboard input."""

    def __init__(self) -> None:
        super().__init__("keyboard_control")
        self.publish_rate: int = 60
        self.controller = VehicleController(
            speed_rate=1.0,
            steer_rate=2.0,
            speed_return_rate=1.5,
            steer_return_rate=4.0,
        )
        self.command_publisher = self.create_publisher(
            ActuatorCommand,
            "actuator_command",
            1,
        )
        self._running = True
        self._last_update = time.monotonic()
        self._root: tk.Tk | None = None

    def publish_controls(self) -> None:
        """Publish the current normalized throttle and steering controls."""
        message = ActuatorCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.throttle = self.controller.speed
        message.steering = self.controller.steer
        self.command_publisher.publish(message)

    def _on_press(self, event: tk.Event[tk.Misc]) -> None:
        key = event.keysym.lower()
        if key == "space":
            self.controller.stop()
            self.publish_controls()
        elif key == "escape":
            self.close()
        else:
            self.controller.press(key)

    def _on_release(self, event: tk.Event[tk.Misc]) -> None:
        self.controller.release(event.keysym.lower())

    def _on_focus_out(self, _event: tk.Event[tk.Misc]) -> None:
        """Stop when the controller window no longer has keyboard focus."""
        self.controller.stop()
        self.publish_controls()

    def _tick(self) -> None:
        if not self._running or not rclpy.ok():
            self.close()
            return
        now = time.monotonic()
        self.controller.update(now - self._last_update)
        self._last_update = now
        self.publish_controls()
        rclpy.spin_once(self, timeout_sec=0.0)
        delay_ms = max(1, round(1000.0 / self.publish_rate))
        root = self._root
        if root is not None:
            root.after(delay_ms, self._tick)

    def close(self) -> None:
        """Stop the vehicle before closing the controller window."""
        if not self._running:
            return
        self._running = False
        self.controller.stop()
        self.publish_controls()
        self.get_logger().info("Manual control stopped")
        if self._root is not None:
            self._root.quit()

    def run(self) -> None:
        """Create the keyboard window and run its event loop."""
        try:
            root = tk.Tk()
        except tk.TclError as error:
            msg = (
                "Could not open the manual-control window. Ensure a graphical "
                "desktop/display is available."
            )
            raise RuntimeError(msg) from error

        self._root = root
        root.title("CASCAR Manual Control")
        root.resizable(width=False, height=False)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<KeyPress>", self._on_press)
        root.bind("<KeyRelease>", self._on_release)
        root.bind("<FocusOut>", self._on_focus_out)

        tk.Label(
            root,
            text="W / ↑  forward     S / ↓  reverse\n"
            "A / ←  left           D / →  right\n\n"
            "SPACE  emergency stop     ESC  close",
            font=("TkDefaultFont", 12),
            justify="left",
            padx=24,
            pady=18,
        ).pack()
        root.focus_force()
        self.get_logger().info("Manual control started; keep its window focused")
        root.after(0, self._tick)
        root.mainloop()
        root.destroy()


def main(args: list[str] | None = None) -> None:
    """Run the manual control ROS node."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    keyboard_control = KeyboardControl()
    try:
        keyboard_control.run()
    except KeyboardInterrupt:
        keyboard_control.close()
    finally:
        keyboard_control.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
