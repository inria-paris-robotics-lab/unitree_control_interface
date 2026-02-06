from .robot_interface import UnitreeControlInterface
from unitree_go.msg import LowCmd, LowState
from typing import List
from unitree_sdk2py.utils.crc import CRC


class Go2ControlInterface(UnitreeControlInterface):
    @property
    def _urdf_to_unitree_index_array(self) -> List[int]:
        return [
            3,
            4,
            5,
            0,
            1,
            2,
            9,
            10,
            11,
            6,
            7,
            8,
        ]

    @property
    def N_DOF(self) -> int:
        """
        Number of actuated degrees of freedom (thus free-flyer should be excluded)
        """
        return 12

    @property
    def ROBOT_FQ(self) -> float:
        """
        Control frequency of the robot (e.g 500.0Hz for the Go2, 1kHz for the G1, ...)
        """
        return 500.0

    @property
    def Kp_static(self) -> List[int]:
        """
        Default kp gains to control the robot in position (for going to start configuration)
        """
        return [150.0] * self.N_DOF

    @property
    def Kd_static(self) -> List[int]:
        """
        Default kd gains to control the robot in position (for going to start configuration)
        """
        return [1.0] * self.N_DOF

    def get_msgs_type(self):
        """
        Returns the state and command message types to control the robot
        """
        return LowState, LowCmd

    def make_cmd_msg(self):
        """
        Create an empty command message, with all the fields pre-filled
        """
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

        return msg

    def compute_cmd_crc(self, msg) -> int:
        """
        Compute the crc of a filled command message
        """
        return self.crc._CRC__Crc32(self.crc._CRC__PackLowCmd(msg))

    def __init__(self, node, *, joints_filter_fq_default=-1):
        super().__init__(node, joints_filter_fq_default=joints_filter_fq_default)
        self.crc = CRC()
