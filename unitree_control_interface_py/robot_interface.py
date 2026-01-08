from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto

from typing import List, Callable
from std_msgs.msg import Bool
from abc import ABC, abstractmethod


class UnitreeControlInterface(ABC):
    ## To be re-implemented for each robots
    @property
    @abstractmethod
    def _urdf_to_unitree_index_array(self) -> List[int]:
        """
        Index of the joints in the unitree message, listed in the urdf order
        """
        pass

    @property
    @abstractmethod
    def N_DOF(self) -> int:
        """
        Number of actuated degrees of freedom (thus free-flyer should be excluded)
        """
        pass

    @property
    @abstractmethod
    def ROBOT_FQ(self) -> float:
        """
        Control frequency of the robot (e.g 500.0Hz for the Go2, 1kHz for the G1, ...)
        """
        pass

    @property
    @abstractmethod
    def Kp_static(self) -> List[int]:
        """
        Default kp gains to control the robot in position (for going to start configuration)
        """
        pass

    @property
    @abstractmethod
    def Kd_static(self) -> List[int]:
        """
        Default kd gains to control the robot in position (for going to start configuration)
        """
        pass

    @abstractmethod
    def get_msgs_type(self):
        """
        Returns the state and command message types to control the robot
        """
        pass

    @abstractmethod
    def make_cmd_msg(self):
        """
        Create an empty command message, with all the fields pre-filled
        """
        pass

    @abstractmethod
    def compute_cmd_crc(self, msg) -> int:
        """
        Compute the crc of a filled command message
        """
        pass

    ## Below here the code is generic for both Go2 and G1 robots

    # State callback variable
    user_cb = None
    start_routine = None

    def __init__(self, node: Node, *, joints_filter_fq_default=-1.0):
        self._is_ready = False
        self._is_safe = False

        self.node = node

        self._watchdog_publisher = self.node.create_publisher(Bool, "/watchdog/arm", 10)
        self._watchdog_subscription = self.node.create_subscription(Bool, "/watchdog/is_safe", self.__safety_cb, 10)

        state_msg_type, command_msg_type = self.get_msgs_type()
        self._cmd_publisher = self.node.create_publisher(command_msg_type, "lowcmd", 10)
        self._state_subscription = self.node.create_subscription(state_msg_type, "lowstate", self.__state_cb, 10)

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

        if self.filter_fq > self.ROBOT_FQ:
            node.get_logger().error(
                "UnitreeControlInterface: Joint filter freq higher than robot sampling freq, stopping node ! %f > %f"
                % (self.filter_fq, self.ROBOT_FQ)
            )
            assert False, "UnitreeControlInterface: Joint filter freq higher than robot sampling freq, stopping node !"

        if self.filter_fq > 0:
            node.get_logger().info("UnitreeControlInterface: Joint filter frequency set to %f Hz." % self.filter_fq)
        else:
            node.get_logger().info("UnitreeControlInterface: Joint filtering disabled.")

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
        assert self.can_be_unlocked(), "UnitreeControlInterface: Robot not ready, unable to unlock joint yet."
        self.start_routine.stop()

        self.transition_duration = transition_duration
        self.transition_start_t = self.node.get_clock().now().nanoseconds / 1.0e9

        self._is_ready = True
        self.node.get_logger().info("UnitreeControlInterface: Unlocking robot.")

    def send_command(self, q: List[float], v: List[float], tau: List[float], kp: List[float], kd: List[float]):
        assert self.can_be_controlled(), (
            "UnitreeControlInterface not start-ed, call start_async(q_start) first and wait for UnitreeControlInterface.can_be_controlled() to return True"
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

        msg = self.make_cmd_msg()

        # Scaling
        k_ratio = self.scaling_gain if scaling else 1.0
        ff_ratio = self.scaling_ff if scaling else 1.0
        for i_urdf, i_unitree in enumerate(self._urdf_to_unitree_index_array):
            msg.motor_cmd[i_unitree].mode = 0x01  # Set toque mode
            msg.motor_cmd[i_unitree].q = q[i_urdf]
            msg.motor_cmd[i_unitree].dq = v[i_urdf]
            msg.motor_cmd[i_unitree].tau = ff_ratio * tau[i_urdf]
            msg.motor_cmd[i_unitree].kp = k_ratio * kp[i_urdf]
            msg.motor_cmd[i_unitree].kd = k_ratio * kd[i_urdf]

        msg.crc = self.compute_cmd_crc(msg)

        self._cmd_publisher.publish(msg)

    def __state_cb(self, msg):
        t = self.node.get_clock().now().nanoseconds / 1.0e9

        # Convert data from unitree order to pinocchio order
        q_urdf = [msg.motor_state[i_unitree].q for i_unitree in self._urdf_to_unitree_index_array]
        v_urdf = [msg.motor_state[i_unitree].dq for i_unitree in self._urdf_to_unitree_index_array]
        # a_urdf = [msg.motor_state[i].ddq for i in self._urdf_to_unitree_index_array] # Not populated by unitree

        # Filter velocity and acceleration is needed
        last_tqva = self.last_state_tqva
        if last_tqva is None or self.filter_fq <= 0.0:
            # No filtering to do on first point
            v_prev = last_tqva[2] if last_tqva is not None else [0.0] * self.N_DOF
            a_finite_diff = [
                self.ROBOT_FQ * (v_urdf[i] - v_prev[i]) for i in range(self.N_DOF)
            ]  # Do that operation first to have the previous v

            self.last_state_tqva = t, q_urdf, v_urdf, a_finite_diff
        else:
            t_prev, q_prev, v_prev, a_prev = last_tqva
            # Filtered derivative (https://fr.mathworks.com/help/sps/ref/filteredderivativediscreteorcontinuous.html#dself.N_DOF6e104759)
            a_filter = [
                self.filter_fq * (v_urdf[i] - v_prev[i]) for i in range(self.N_DOF)
            ]  # Do that operation first to have the previous v

            alpha = self.filter_fq / self.ROBOT_FQ
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
        self.kp = self.robot.Kp_static
        self.kd = self.robot.Kd_static

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
        self.robot.node.get_logger().info("UnitreeControlInterface: Waiting for first state msg...")

    def update(self, t_now: float, q_now: list[float]):
        """Called from inside __state_cb : progresses the routine."""

        match self.state:
            case GoToStartRoutine.State.IDLE:
                pass

            case GoToStartRoutine.State.WAIT_WATCHDOG:
                # Waiting for watchdog to be armed
                if not self.robot._is_safe:
                    self.robot.node.get_logger().info(
                        "UnitreeControlInterface: Waiting for watchdog to be armed...", once=True
                    )
                    return

                # Watchdog armed
                self.robot.node.get_logger().info("UnitreeControlInterface: Watchdog to be armed !", once=True)
                if self.goto_config:
                    self.t_start = t_now
                    self.q_start = q_now[:]
                    self.state = GoToStartRoutine.State.INTERPOLATING
                else:
                    # Skip config – just send zeros once
                    self.q_cmd = [0.0] * self.robot.N_DOF
                    self.state = GoToStartRoutine.State.HOLD
                    self.robot.node.get_logger().info(
                        "UnitreeControlInterface: Skipped start configuration, finished setup."
                    )

            case GoToStartRoutine.State.INTERPOLATING:
                self.robot.node.get_logger().info("UnitreeControlInterface: Going to start configuration.", once=True)

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
                    self.robot.node.get_logger().info(
                        "UnitreeControlInterface: Start configuration reached.", once=True
                    )
                    self.state = GoToStartRoutine.State.HOLD

            case GoToStartRoutine.State.HOLD:
                self.robot.node.get_logger().info(
                    "UnitreeControlInterface: Holding current command until unlocking.", once=True
                )

        # Send command to robot when necessary
        if self.state == GoToStartRoutine.State.INTERPOLATING or self.state == GoToStartRoutine.State.HOLD:
            self.robot._send_command(self.q_cmd, self.v_cmd, self.tau_cmd, self.kp_cmd, self.kd_cmd)

    def is_waiting_completion(self) -> bool:
        return self.state == GoToStartRoutine.State.HOLD

    def stop(self):
        """Disable the routine by setting it to idle mode"""
        self.state = GoToStartRoutine.State.IDLE
