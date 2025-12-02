import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor

from typing import List, Callable
from unitree_hg.msg import LowCmd, LowState
from std_msgs.msg import Bool
from unitree_sdk2py.utils.crc import CRC
import threading


class Go2RobotInterface:
    # TODO: Populate this array programmatically
    # fmt: off
    __urdf_to_unitree_index = (
        0, 1, 2, 3,  4,  5,         # left leg
        6, 7, 8, 9, 10, 11,         # right leg
        12,                         # waist (here the two locked DoF - 13 and 14 - are skipped)
        15, 16 ,17, 18 ,19, 20, 21, # left arm
        22, 23, 24, 25, 26, 27, 28  # right arm
    )
    # fmt: on

    N_DOF = 27

    def __init__(self, node: Node, *, joints_filter_fq_default=-1.0):
        self.is_ready = False
        self.is_safe = False

        self.node = node

        self._watchdog_publisher = self.node.create_publisher(Bool, "/watchdog/arm", 10)
        self._watchdog_subscription = self.node.create_subscription(Bool, "/watchdog/is_safe", self.__safety_cb, 10)

        self._cmd_publisher = self.node.create_publisher(LowCmd, "lowcmd", 10)
        self._state_subscription = self.node.create_subscription(LowState, "lowstate", self.__state_cb, 10)

        self.scaling_glob = self.node.declare_parameter("scaling_glob", 1.0).value
        self.scaling_gain = self.node.declare_parameter("scaling_gain", 1.0).value
        self.scaling_ff = self.node.declare_parameter("scaling_ff", 1.0).value

        self.last_state_tqva = None
        self.filter_fq = self.node.declare_parameter(
            "joints_filter_fq",
            joints_filter_fq_default,
            ParameterDescriptor(description="Characteristic frequency of the filters on q, dq, ddq"),
        ).value  # By default no filter
        self.robot_fq = self.node.declare_parameter(
            "robot_fq",
            500.0,
            ParameterDescriptor(description="Frequency at which the robot state messages are published"),
        ).value  # 500Hz for the Go2

        if self.filter_fq > self.robot_fq:
            node.get_logger().error(
                "Go2RobotInterface: Joint filter freq higher than robot sampling freq, stopping node ! %f > %f"
                % (self.filter_fq, self.robot_fq)
            )
            assert False, "Go2RobotInterface: Joint filter freq higher than robot sampling freq, stopping node !"

        if self.filter_fq > 0:
            node.get_logger().info("Go2RobotInterface: Joint filter frequency set to %f Hz." % self.filter_fq)
        else:
            node.get_logger().info("Go2RobotInterface: Joint filtering disabled.")

        self.crc = CRC()
        self.user_cb = None
        # TODO: Add a callback to joint_states and verify that robots is within safety bounds

    def register_callback(self, callback: Callable[[float, List[float], List[float], List[float]], None]):
        self.user_cb = callback

    def start_async(self, q_start: List[float], *, goto_config: Bool = True):
        thread = threading.Thread(target=self._start_routine, args=(q_start, goto_config), daemon=True)
        thread.start()

    def _start_routine(self, q_start: List[float], goto_config=True):
        # Arm watchdog
        arm_watchdog_msg = Bool()
        arm_watchdog_msg.data = True
        self._watchdog_publisher.publish(arm_watchdog_msg)

        self.node.get_logger().info("Go2RobotInterface: Waiting for watchdog to be armed...")
        while not self.is_safe and rclpy.ok():
            self.node.get_clock().sleep_for(Duration(seconds=0.1))

        if goto_config:
            self.node.get_logger().info("Go2RobotInterface: Going to start configuration...")
            self._go_to_configuration__(q_start, 5.0)
            self.node.get_logger().info("Go2RobotInterface: Start configuration reached.")
        else:
            self.node.get_logger().info("Go2RobotInterface: Skipping start configuration, set command to zero")
            zeros = [0.0] * self.N_DOF
            self._send_command(zeros, zeros, zeros, zeros, zeros, 0.0)
        self.is_ready = True

    def send_command(self, q: List[float], v: List[float], tau: List[float], kp: List[float], kd: List[float]):
        assert self.is_ready, (
            "Go2RobotInterface not start-ed, call start_async(q_start) first and wait for Go2RobotInterface.is_ready flag to be True"
        )
        assert self.is_safe, "Soft e-stop sent by watchdog, ignoring command"
        self._send_command(q, v, tau, kp, kd)

    def _send_command(
        self, q: List[float], v: List[float], tau: List[float], kp: List[float], kd: List[float], scaling: Bool = True
    ):
        assert len(q) == self.N_DOF, "Wrong configuration size"
        assert len(v) == self.N_DOF, "Wrong configuration size"
        assert len(tau) == self.N_DOF, "Wrong configuration size"
        assert len(kp) == self.N_DOF, "Wrong configuration size"
        assert len(kd) == self.N_DOF, "Wrong configuration size"

        msg = LowCmd()

        # Init header
        msg.mode_pr = 0  # Parallel mechanism (ankle and waist) control mode (default 0) 0:PR, 1:AB
        msg.mode_machine = 6  # G1 Type：4：23-Dof;5:29-Dof;6:27-Dof(29Dof Fitted at the waist)

        # Scaling
        k_ratio = self.scaling_gain * self.scaling_glob if scaling else 1.0
        ff_ratio = self.scaling_ff * self.scaling_glob if scaling else 1.0
        for i_urdf, i_unitree in enumerate(self.__urdf_to_unitree_index):
            msg.motor_cmd[i_unitree].mode = 0x01  # Set toque mode
            msg.motor_cmd[i_unitree].q = q[i_urdf]
            msg.motor_cmd[i_unitree].dq = v[i_urdf]
            msg.motor_cmd[i_unitree].tau = ff_ratio * tau[i_urdf]
            msg.motor_cmd[i_unitree].kp = k_ratio * kp[i_urdf]
            msg.motor_cmd[i_unitree].kd = k_ratio * kd[i_urdf]

        # Compute CRC here
        # TODO: Cleaner CRC computation
        msg.crc = self.crc._CRC__Crc32(self.crc._CRC__PackHGLowCmd(msg))
        self._cmd_publisher.publish(msg)

    def __state_cb(self, msg: LowState):
        t = self.node.get_clock().now().nanoseconds / 1.0e9
        q_urdf = [msg.motor_state[i_unitree].q for i_unitree in self.__urdf_to_unitree_index]
        v_urdf = [msg.motor_state[i_unitree].dq for i_unitree in self.__urdf_to_unitree_index]
        # a_urdf = [msg.motor_state[i].ddq for i in self.__urdf_to_unitree_index] # Not populated by unitree

        last_tqva = self.last_state_tqva
        if last_tqva is None or self.filter_fq <= 0.0:
            # No filtering to do on first point
            v_prev = last_tqva[2] if last_tqva is not None else [0.0] * self.N_DOF
            a_finite_diff = [
                self.robot_fq * (v_urdf[i] - v_prev[i]) for i in range(self.N_DOF)
            ]  # Do that operation first to have the previous v

            self.last_state_tqva = t, q_urdf, v_urdf, a_finite_diff
        else:
            t_prev, q_prev, v_prev, a_prev = last_tqva
            # Filtered derivative (https://fr.mathworks.com/help/sps/ref/filteredderivativediscreteorcontinuous.html#dself.N_DOF6e104759)
            a_filter = [
                self.filter_fq * (v_urdf[i] - v_prev[i]) for i in range(self.N_DOF)
            ]  # Do that operation first to have the previous v

            alpha = self.filter_fq / self.robot_fq
            q_filter = [(1 - alpha) * q_prev[i] + alpha * q_urdf[i] for i in range(self.N_DOF)]
            v_filter = [(1 - alpha) * v_prev[i] + alpha * v_urdf[i] for i in range(self.N_DOF)]

            self.last_state_tqva = t, q_filter, v_filter, a_filter

        if self.user_cb is not None:
            self.user_cb(*self.last_state_tqva)

    def _go_to_configuration__(self, q: List[float], duration: float):
        for i in range(5):
            if self.last_state_tqva is not None:
                break
            self.node.get_clock().sleep_for(Duration(seconds=0.5))  # Wait for first configuration to be received
        else:
            assert False, "Robot state not received in time for initialization of interface."

        q_goal = q
        _, q_start, _, _ = self.last_state_tqva

        t_start = self.node.get_clock().now()
        while rclpy.ok():
            t = self.node.get_clock().now()
            ratio = (t - t_start).nanoseconds / (duration * 1e9)
            ratio = min(ratio, 1)

            q_des = [q_start[i] + (q_goal[i] - q_start[i]) * ratio for i in range(self.N_DOF)]
            self._send_command(q_des, [0.0] * self.N_DOF, [0.0] * self.N_DOF, [50.0] * self.N_DOF, [1.0] * self.N_DOF)

            if ratio == 1:
                break

    def __safety_cb(self, msg):
        self.is_safe = msg.data
