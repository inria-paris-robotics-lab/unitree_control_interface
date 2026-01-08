from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto

from typing import List, Callable
from unitree_go.msg import LowCmd, LowState
from std_msgs.msg import Bool
from unitree_sdk2py.utils.crc import CRC


class Go2RobotInterface:
    # TODO: Populate this array programmatically
    # fmt: off
    __urdf_to_unitree_index = [
            3,  4,  5,
            0,  1,  2,
            9, 10, 11,
            6,  7,  8,
        ] # re-ordering joints
    # fmt: on

    # State callback variable
    user_cb = None
    start_routine = None

    N_DOF = 12

    def __init__(self, node: Node, *, joints_filter_fq_default=-1.0):
        self._is_ready = False
        self._is_safe = False

        self.node = node

        self._watchdog_publisher = self.node.create_publisher(Bool, "/watchdog/arm", 10)
        self._watchdog_subscription = self.node.create_subscription(Bool, "/watchdog/is_safe", self.__safety_cb, 10)

        self._cmd_publisher = self.node.create_publisher(LowCmd, "lowcmd", 10)
        self._state_subscription = self.node.create_subscription(LowState, "lowstate", self.__state_cb, 10)

        self.scaling_gain = self.node.declare_parameter("scaling_gain", 1.0).value
        self.scaling_ff = self.node.declare_parameter("scaling_ff", 1.0).value

        self.transition_start_t = 0.0
        self.transition_duration = 0.0

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
        self.start_routine = GoToStartRoutine(self)

    def register_callback(self, callback: Callable[[float, List[float], List[float], List[float]], None]):
        self.user_cb = callback

    def start_async(self, q_start: List[float], *, goto_config: Bool = True):
        self.start_routine.start(q_start, goto_config=goto_config)

    def can_be_unlocked(self):
        return self.start_routine.is_waiting_completion()

    def can_be_controlled(self):
        return self._is_ready

    def unlock(self, *, transition_duration=0.0):
        """
        Unlock the robot after `start_async` is called with `goto_config` set to True.
        The robot will stay (in position control) at the start configuration, until this `unlock` method is called.
        Consequently, the `can_be_controlled()` flag will be set ot True only after this method is called (even if `goto_config` is set to False)
        """
        assert self.can_be_unlocked(), "Go2RobotInterface: Robot not ready, unable to unlock joint yet."
        self.start_routine.stop()

        self.transition_duration = transition_duration
        self.transition_start_t = self.node.get_clock().now().nanoseconds / 1.0e9

        self._is_ready = True
        self.node.get_logger().info("Go2RobotInterface: Unlocking robot.")

    def send_command(self, q: List[float], v: List[float], tau: List[float], kp: List[float], kd: List[float]):
        assert self.can_be_controlled(), (
            "Go2RobotInterface not start-ed, call start_async(q_start) first and wait for Go2RobotInterface.can_be_controlled() to return True"
        )
        if self.transition_start_t is not None:
            t = self.node.get_clock().now().nanoseconds / 1.0e9
            ratio = (t - self.transition_start_t) / self.transition_duration
            if ratio > 1.0:
                ratio = 1.0
                # Transition over disable it
                self.transition_start_t = None

            q_cmd = [0.0] * self.N_DOF
            v_cmd = [0.0] * self.N_DOF
            tau_cmd = [0.0] * self.N_DOF
            kp_cmd = [0.0] * self.N_DOF
            kd_cmd = [0.0] * self.N_DOF

            for i in range(self.N_DOF):
                q_cmd[i] = q[i] * ratio + self.start_routine.q_cmd[i] * (1.0 - ratio)
                v_cmd[i] = v[i] * ratio + self.start_routine.v_cmd[i] * (1.0 - ratio)
                tau_cmd[i] = tau[i] * ratio + self.start_routine.tau_cmd[i] * (1.0 - ratio)
                kp_cmd[i] = kp[i] * ratio + self.start_routine.kp_cmd[i] * (1.0 - ratio)
                kd_cmd[i] = kd[i] * ratio + self.start_routine.kd_cmd[i] * (1.0 - ratio)

            self._send_command(q_cmd, v_cmd, tau_cmd, kp_cmd, kd_cmd)
        else:
            self._send_command(q, v, tau, kp, kd)

    def _send_command(
        self,
        q: List[float],
        v: List[float],
        tau: List[float],
        kp: List[float],
        kd: List[float],
        *,
        scaling: Bool = True,
        skip_safety: Bool = False,
    ):
        assert len(q) == self.N_DOF, "Wrong configuration size"
        assert len(v) == self.N_DOF, "Wrong configuration size"
        assert len(tau) == self.N_DOF, "Wrong configuration size"
        assert len(kp) == self.N_DOF, "Wrong configuration size"
        assert len(kd) == self.N_DOF, "Wrong configuration size"
        assert skip_safety or self._is_safe, "Soft e-stop sent by watchdog, ignoring command"

        msg = LowCmd()

        # Init header
        msg.head = 0xFE, 0xEF

        # Unused fields
        msg.level_flag = 0
        msg.frame_reserve = 0
        msg.sn = 0, 0
        msg.bandwidth = 0
        msg.fan = 0, 0
        msg.reserve = 0
        msg.led = [0] * 12

        # battery
        msg.bms_cmd.off = 0
        msg.bms_cmd.reserve = 0, 0, 0

        # Version
        msg.sn = 0, 0

        # Gpio
        msg.gpio = 0

        # Scaling
        k_ratio = self.scaling_gain if scaling else 1.0
        ff_ratio = self.scaling_ff if scaling else 1.0
        for i_urdf, i_unitree in enumerate(self.__urdf_to_unitree_index):
            msg.motor_cmd[i_unitree].mode = 0x01  # Set toque mode
            msg.motor_cmd[i_unitree].q = q[i_urdf]
            msg.motor_cmd[i_unitree].dq = v[i_urdf]
            msg.motor_cmd[i_unitree].tau = ff_ratio * tau[i_urdf]
            msg.motor_cmd[i_unitree].kp = k_ratio * kp[i_urdf]
            msg.motor_cmd[i_unitree].kd = k_ratio * kd[i_urdf]

        # Compute CRC here
        # TODO: Cleaner CRC computation
        msg.crc = self.crc._CRC__Crc32(self.crc._CRC__PackLowCmd(msg))
        self._cmd_publisher.publish(msg)

    def __state_cb(self, msg: LowState):
        t = self.node.get_clock().now().nanoseconds / 1.0e9

        # Convert data from unitree order to pinocchio order
        q_urdf = [msg.motor_state[i_unitree].q for i_unitree in self.__urdf_to_unitree_index]
        v_urdf = [msg.motor_state[i_unitree].dq for i_unitree in self.__urdf_to_unitree_index]
        # a_urdf = [msg.motor_state[i].ddq for i in self.__urdf_to_unitree_index] # Not populated by unitree

        # Filter velocity and acceleration is needed
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

        # State machine
        self.start_routine.update(t, q_urdf)

        # Call user callback
        if self.user_cb is not None:
            self.user_cb(*self.last_state_tqva)

    def __safety_cb(self, msg):
        self._is_safe = msg.data


class GoToStartRoutine:
    class State(Enum):
        IDLE = auto()
        ARM_WATCHDOG = auto()
        WAIT_WATCHDOG = auto()
        INTERPOLATING = auto()
        HOLD = auto()

    def __init__(self, robot_interface):
        self.robot = robot_interface

        # Initialize state machine
        self.state = GoToStartRoutine.State.IDLE
        self.t_start = None
        self.q_start = None
        self.q_goal = None

        # Buffers for commands
        self.q_cmd = [0.0] * self.robot.N_DOF
        self.v_cmd = [0.0] * self.robot.N_DOF
        self.tau_cmd = [0.0] * self.robot.N_DOF
        self.kp_cmd = [0.0] * self.robot.N_DOF
        self.kd_cmd = [0.0] * self.robot.N_DOF

        # Gains for position control
        self.kp = [150.0] * self.robot.N_DOF
        self.kd = [1.0] * self.robot.N_DOF

    def start(self, q_goal: list[float], *, duration: float = 5.0, goto_config=True):
        # Set goal related values
        self.q_goal = q_goal
        self.duration = duration
        self.goto_config = goto_config

        # Arm watchdog
        msg = Bool()
        msg.data = True
        self.robot._watchdog_publisher.publish(msg)
        self.state = GoToStartRoutine.State.WAIT_WATCHDOG

        # Now wait for update to be called
        self.robot.node.get_logger().info("Go2RobotInterface: Waiting for first state msg...")

    def update(self, t_now: float, q_now: list[float]):
        """Called from inside __state_cb : progresses the routine."""

        match self.state:
            case GoToStartRoutine.State.IDLE:
                pass

            case GoToStartRoutine.State.WAIT_WATCHDOG:
                # Waiting for watchdog to be armed
                if not self.robot._is_safe:
                    self.robot.node.get_logger().info(
                        "Go2RobotInterface: Waiting for watchdog to be armed...", once=True
                    )
                    return

                # Watchdog armed
                self.robot.node.get_logger().info("Go2RobotInterface: Watchdog to be armed !", once=True)
                if self.goto_config:
                    self.t_start = t_now
                    self.q_start = q_now[:]
                    self.state = GoToStartRoutine.State.INTERPOLATING
                else:
                    # Skip config – just send zeros once
                    self.q_cmd = [0.0] * self.robot.N_DOF
                    self.state = GoToStartRoutine.State.HOLD
                    self.robot.node.get_logger().info("Go2RobotInterface: Skipped start configuration, finished setup.")

            case GoToStartRoutine.State.INTERPOLATING:
                self.robot.node.get_logger().info("Go2RobotInterface: Going to start configuration.", once=True)

                # First increase the gain progressively (to prevent any harsh discontinuity in the motor)
                duration_k = 0.1 * self.duration
                ratio_k = (t_now - self.t_start) / duration_k
                ratio_k = min(ratio_k, 1.0)

                # Then interpolate the configuration
                duration_q = self.duration - duration_k
                ratio_q = (t_now - self.t_start - duration_k) / duration_q
                ratio_q = min(max(ratio_q, 0.0), 1.0)  # clamp between 0. and 1.

                # Linear interpolation of joint positions
                for i in range(self.robot.N_DOF):
                    self.q_cmd[i] = self.q_start[i] + (self.q_goal[i] - self.q_start[i]) * ratio_q
                    self.kp_cmd[i] = self.kp[i] * ratio_k
                    self.kd_cmd[i] = self.kd[i] * ratio_k

                if ratio_q >= 1.0:
                    # Interpolation done, hold the last value until release
                    self.robot.node.get_logger().info("Go2RobotInterface: Start configuration reached.", once=True)
                    self.state = GoToStartRoutine.State.HOLD

            case GoToStartRoutine.State.HOLD:
                self.robot.node.get_logger().info(
                    "Go2RobotInterface: Holding current command until unlocking.", once=True
                )

        # Send command to robot when necessary
        if self.state == GoToStartRoutine.State.INTERPOLATING or self.state == GoToStartRoutine.State.HOLD:
            self.robot._send_command(self.q_cmd, self.v_cmd, self.tau_cmd, self.kp_cmd, self.kd_cmd)

    def is_waiting_completion(self) -> bool:
        return self.state == GoToStartRoutine.State.HOLD

    def stop(self):
        """Disable the routine by setting it to idle mode"""
        self.state = GoToStartRoutine.State.IDLE
