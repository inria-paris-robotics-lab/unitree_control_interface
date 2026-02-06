from .robot_interface import UnitreeControlInterface
from unitree_hg.msg import LowCmd, LowState
from typing import List
from unitree_sdk2py.utils.crc import CRC


class G1ControlInterface(UnitreeControlInterface):
    @property
    def _urdf_to_unitree_index_array(self) -> List[int]:
        return (
            0,
            1,
            2,
            3,
            4,
            5,  # left leg
            6,
            7,
            8,
            9,
            10,
            11,  # right leg
            12,  # waist (here the two locked DoF - 13 and 14 - are skipped)
            15,
            16,
            17,
            18,
            19,
            20,
            21,  # left arm
            22,
            23,
            24,
            25,
            26,
            27,
            28,  # right arm
        )

    @property
    def N_DOF(self) -> int:
        """
        Number of actuated degrees of freedom (thus free-flyer should be excluded)
        """
        return 27

    @property
    def ROBOT_FQ(self) -> float:
        """
        Control frequency of the robot (e.g 500.0Hz for the Go2, 1kHz for the G1, ...)
        """
        return 1000.0

    @property
    def Kp_static(self) -> List[int]:
        """
        Default kp gains to control the robot in position (for going to start configuration)
        """
        return [75.0] * self.N_DOF

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

        msg.mode_pr = 0  # Parallel mechanism (ankle and waist) control mode (default 0) 0:PR, 1:AB
        msg.mode_machine = 6  # G1 Type：4：23-Dof;5:29-Dof;6:27-Dof(29Dof Fitted at the waist)

        return msg

    def compute_cmd_crc(self, msg) -> int:
        """
        Compute the crc of a filled command message
        """
        return self.crc._CRC__Crc32(self.crc._CRC__PackHGLowCmd(msg))

    def __init__(self, node, *, joints_filter_fq_default=-1):
        super().__init__(node, joints_filter_fq_default=joints_filter_fq_default)
        self.crc = CRC()
