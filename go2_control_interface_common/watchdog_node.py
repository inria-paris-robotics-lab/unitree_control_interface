#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from go2_control_interface_py.robot_interface import Go2RobotInterface
from unitree_go.msg import LowCmd
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class WatchDogNode(Node, Go2RobotInterface):
    """
    The watchdog has 3 states :
     state | is_stopped | is_waiting | description
    -------|------------|------------|------------
       A   |     0      |     1      | The watchdog is armed, ready to start, but not actually checking
       B   |     0      |     0      | The watchdog is running, check joints bounds and timeout
       C   |     1      |     -      | The watchdog spam stops commands

    The transitions are as follow:
    A -> B : if a msg is received on /lowcmd
    B -> C : if the joint bounds or the timeout is exceeded
    any -> C : if a False is received on /watchdog/arm
    any -> A : if a True is received on /watchdog/arm

    The topics are published as follow :
    A or B -> is_safe set to True, no command sent to the robot
    C -> is_safe set to False, damping commands spammed to the robot
    """

    def __init__(self):
        Node.__init__(self, "watchdog")
        Go2RobotInterface.__init__(self, self, joints_filter_fq_default=200)

        # Watchdog timer parameters
        self.freq = self.declare_parameter("freq", 100).value
        self.n_fail = self.declare_parameter("n_fail", 2).value
        self.ignore_joint_limits = self.node.declare_parameter("ignore_joint_limits", False).value

        # Safety values
        self.q_max = self.declare_parameter("q_max", [0.0]).value
        self.q_min = self.declare_parameter("q_min", [0.0]).value
        self.dq_max = self.declare_parameter("dq_max", -1.0).value
        assert len(self.q_max) == 12, "Parameter q_max should be length 12"
        assert len(self.q_min) == 12, "Parameter q_min should be length 12"
        assert self.dq_max >= 0.0, "Parameter dq_max should be non negative"

        # Watchdog timer logic
        self.cnt = 0
        self.is_stopped = False
        self.is_waiting = False

        self.lowcmd_subscription = self.create_subscription(LowCmd, "/lowcmd", self.__cmd_cb, 10)
        self.start_subscription = self.create_subscription(Bool, "/watchdog/arm", self.__arm_disarm_cb, 10)
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)

        if not self.ignore_joint_limits:
            self.register_callback(self.__state_cb)

        self._is_safe_publisher = self.create_publisher(
            Bool, "/watchdog/is_safe", QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )

    def __arm_disarm_cb(self, msg):
        # Acting as an e-stop
        if not msg.data:
            self._stop_robot("E-stop pressed.")
        else:
            self._arm_watchdog()

    def __cmd_cb(self, msg):
        if self.is_waiting:
            self.get_logger().warning("First command received on /lowcmd, watchdog running")

        self.is_waiting = False
        self.cnt = 0  # Reset timeout

    def __state_cb(self, t, q, dq, ddq):
        # Joint bounds
        q_max_bound = [q_i > self.q_max[i] for i, q_i in enumerate(q)]
        q_min_bound = [q_i < self.q_min[i] for i, q_i in enumerate(q)]
        dq_max_bound = [abs(dq_i) > self.dq_max for dq_i in dq]

        if any(q_max_bound):
            self._stop_robot(
                f"Watch-dog detect joint {[i for i, b in enumerate(q_max_bound) if b]} out of bounds. (max q)"
            )
        if any(q_min_bound):
            self._stop_robot(
                f"Watch-dog detect joint {[i for i, b in enumerate(q_min_bound) if b]} out of bounds. (min q)"
            )
        if any(dq_max_bound):
            self._stop_robot(
                f"Watch-dog detect joint {[i for i, b in enumerate(dq_max_bound) if b]} out of bounds. (max dq)"
            )
        # TODO: Add check on tau (look at cmd ??)

    def timer_callback(self):
        # If stopped, spam damping command
        if self.is_stopped:
            self._send_kill_cmd()
            return

        if self.is_waiting:
            # No check needs to be done
            return

        # Timeout
        self.cnt += 1
        if self.cnt >= self.n_fail:
            self._stop_robot("Watch-dog timer reached.")

    def _arm_watchdog(self):
        # Arming the watchdog
        self.cnt = 0
        self.is_waiting = True
        self.is_stopped = False
        self.get_logger().warning("Watch-dog armed, waiting for /lowcmd")

        # Send info to other nodes
        is_safe_msg = Bool()
        is_safe_msg.data = True
        self._is_safe_publisher.publish(is_safe_msg)

    def _stop_robot(self, msg_str):
        self._send_kill_cmd()  # ASAP
        if not self.is_stopped:
            self.get_logger().error(msg_str + " Stopping robot.")
        self.is_stopped = True
        self.is_waiting = False

    def _send_kill_cmd(self):
        self._send_command([0.0] * 12, [0.0] * 12, [0.0] * 12, [0.0] * 12, [1.0] * 12, scaling=False)
        # Send info to other nodes
        is_safe_msg = Bool()
        is_safe_msg.data = False
        self._is_safe_publisher.publish(is_safe_msg)


def main(args=None):
    rclpy.init(args=args)
    watch_dog_node = WatchDogNode()

    rclpy.spin(watch_dog_node)

    watch_dog_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
