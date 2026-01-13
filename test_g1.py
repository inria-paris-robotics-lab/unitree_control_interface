import rclpy
from rclpy.node import Node
from go2_control_interface_py.robot_interface import Go2RobotInterface
from go2_description.loader import loadG1
from std_msgs.msg import Empty
import pinocchio as pin

# from pinocchio.visualize import MeshcatVisualizer
import numpy as np
from simple_mpc import (
    RobotModelHandler,
    RobotDataHandler,
    KinodynamicsID,
    KinodynamicsIDSettings,
)


class MyApp(
    Node,
):
    def __init__(self):
        Node.__init__(self, "my_app2")

        self.pin_robot_wrapper = loadG1()

        self.robot_if = Go2RobotInterface(self, joints_filter_fq_default=150.0)
        self.robot_if.register_callback(self._sensor_reading_callback)

        self._state_subscription = self.create_subscription(Empty, "unlock_ctrl", self.__unlock_cb, 1)

        # self.viz = MeshcatVisualizer(self.pin_robot_wrapper.model, self.pin_robot_wrapper.collision_model, self.pin_robot_wrapper.visual_model)
        # self.viz.initViewer(open=False, zmq_url="tcp://127.0.0.1:6000")
        # self.viz.loadViewerModel()
        # self.viz.display(np.array([0,0,0, 0,0,0,1] + [0.] * 27))

        # Create inverse dynamic
        base_frame_name = "torso_link"
        foot_points = np.array(
            [[0.12, 0.03, -0.035], [-0.06, 0.03, -0.035], [-0.06, -0.03, -0.035], [0.12, -0.03, -0.035]]
        )
        self.model_handler = RobotModelHandler(self.pin_robot_wrapper.model, "slightly_forward", base_frame_name)
        self.model_handler.addQuadFoot("left_ankle_roll_link", base_frame_name, foot_points)
        self.model_handler.addQuadFoot("right_ankle_roll_link", base_frame_name, foot_points)
        self.data_handler = RobotDataHandler(self.model_handler)

        kino_ID_settings = KinodynamicsIDSettings()
        kino_ID_settings.kp_base = 7.0
        kino_ID_settings.kp_posture = 50.0
        kino_ID_settings.kp_contact = 10.0
        kino_ID_settings.w_base = 100.0
        kino_ID_settings.w_posture = 10.0
        kino_ID_settings.w_contact_force = 0.001
        kino_ID_settings.w_contact_motion = 1.0

        self.kino_ID = KinodynamicsID(self.model_handler, 1.0 / 500.0, kino_ID_settings)

        self.nq = self.model_handler.getModel().nq
        self.nv = self.model_handler.getModel().nv

        self.kp = [7.5] * (self.nv - 6)
        self.kd = [0.1] * (self.nv - 6)

        # The robot will move by itself to the q_start configuration and wait for you first command
        self.robot_if.start_async(self.model_handler.getReferenceState()[7 : self.nq])

    def _compute_base_pose_vel(self, q, dq):
        model = self.model_handler.getModel()
        data = self.data_handler.getData()
        base_frame_id = self.model_handler.getBaseFrameId()
        feet_frame_ids = self.model_handler.getFeetFrameIds()

        q_full = pin.neutral(model)
        q_full[7:] = q
        dq_full = np.zeros(model.nv)
        dq_full[6:] = dq

        pin.forwardKinematics(model, data, q_full, dq_full)
        pin.updateFramePlacements(model, data)

        oMfeet = [data.oMf[foot_frame_id] for foot_frame_id in feet_frame_ids]
        oMbase = data.oMf[base_frame_id]

        feetMbase = [oMfoot.actInv(oMbase) for oMfoot in oMfeet]

        feetavgMbase = pin.exp(
            sum([pin.log(footMbase) for footMbase in feetMbase], start=pin.Motion()) / len(feetMbase)
        )

        w_v_feet = [
            pin.getFrameVelocity(model, data, foot_frame_id, pin.ReferenceFrame.WORLD)
            for foot_frame_id in feet_frame_ids
        ]
        w_v_base = pin.getFrameVelocity(model, data, base_frame_id, pin.ReferenceFrame.WORLD)
        w_v_feet_base = w_v_base - sum(w_v_feet, start=pin.Motion()) / len(w_v_feet)
        v_feet_base = oMbase.actInv(w_v_feet_base)

        return pin.SE3ToXYZQUAT(feetavgMbase), v_feet_base.vector

    def _sensor_reading_callback(self, t, q, dq, ddq):
        # Reading timestamp, positions, velocities, accelerations
        # (Should be received at 500Hz approx.)
        # self.viz.display(np.array([0,0,0, 0,0,0,1] + q))

        # Sending commands
        base_pose, base_vel = self._compute_base_pose_vel(q, dq)
        q_meas = np.concatenate((base_pose, np.array(q)))
        v_meas = np.concatenate((base_vel, np.array(dq)))

        if self.robot_if.can_be_controlled():
            tau_cmd = self.kino_ID.solve(t, q_meas, np.zeros_like(v_meas))

            dt = 0.001
            a_next = self.kino_ID.getAccelerations()
            v_next = v_meas + a_next * dt
            q_next = pin.integrate(self.pin_robot_wrapper.model, q_meas, dt * (v_next + v_meas) / 2.0)

            self.robot_if.send_command(q_next[7:], v_next[6:], tau_cmd, self.kp, self.kd)

    def __unlock_cb(self, msg):
        self.robot_if.unlock(transition_duration=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = MyApp()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
