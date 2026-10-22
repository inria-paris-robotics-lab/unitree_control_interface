#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_control_interface_py import Go2ControlInterface


class MyApp(
    Node,
):
    def __init__(self):
        Node.__init__(self, "my_app")
        self.robot_if = Go2ControlInterface(self)
        self.robot_if.register_callback(self._sensor_reading_callback)

        # The robot will move by itself to the q_start configuration and wait for your first command
        self.q_base = [0.0, 0.96, -1.8385, 0.0, 0.96, -1.8385, 0.0, 1.007, -1.8385, 0.0, 1.007, -1.8385]
        self.robot_if.start_async(self.q_base)

    def _sensor_reading_callback(self, t, q, dq, ddq):
        # Reading timestamp, positions, velocities, accelerations
        # (Should be received at 500Hz approx.)
        # Sending commands
        q_des = self.q_base
        v_des = [0.0] * 12
        tau_des = [0.0] * 12
        kp = [100] * 12
        kd = [0.0] * 12

        # Call this once you app is ready to send command. (In this case can be sent directly)
        if self.robot_if.can_be_unlocked():
            # The robot will stay in position control at q_start config until you call that routine
            # The 1.0 argument will make the interface transition smoothly from the position control to your commands over a 1.0s duration
            self.robot_if.unlock(transition_duration=1.0)

        # This flag is True once both the robot reached the start configuration and self.robot_if.unlock() has been called.
        if self.robot_if.can_be_controlled():
            self.robot_if.send_command(q_des, v_des, tau_des, kp, kd)  # Will crash if called when robot is not ready.


def main(args=None):
    rclpy.init(args=args)
    node = MyApp()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
