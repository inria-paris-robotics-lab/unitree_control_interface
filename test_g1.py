import rclpy
from rclpy.node import Node
from go2_control_interface_py.robot_interface import Go2RobotInterface
from go2_description.loader import loadG1
import pinocchio as pin
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

        self.robot_if = Go2RobotInterface(self)
        self.robot_if.register_callback(self._sensor_reading_callback)

        # self.viz = MeshcatVisualizer(self.pin_robot_wrapper.model, self.pin_robot_wrapper.collision_model, self.pin_robot_wrapper.visual_model)
        # self.viz.initViewer(open=True)
        # self.viz.loadViewerModel()
        # self.viz.display(np.array([0,0,0, 0,0,0,1] + [0.] * 27))

        # Create inverse dynamic
        base_frame_name = "torso_link"
        foot_points = np.array(
            [[0.12, 0.03, -0.035], [-0.06, 0.03, -0.035], [-0.06, -0.03, -0.035], [0.12, -0.03, -0.035]]
        )
        self.model_handler = RobotModelHandler(self.pin_robot_wrapper.model, "half_sitting", base_frame_name)
        self.model_handler.addQuadFoot("left_ankle_roll_link", base_frame_name, foot_points)
        self.model_handler.addQuadFoot("right_ankle_roll_link", base_frame_name, foot_points)
        self.data_handler = RobotDataHandler(self.model_handler)

        kino_ID_settings = KinodynamicsIDSettings()
        kino_ID_settings.kp_base = 7.0
        kino_ID_settings.kp_posture = 100.0
        kino_ID_settings.kp_contact = 10.0
        kino_ID_settings.w_base = 100.0
        kino_ID_settings.w_posture = 100.0  # 1.0
        kino_ID_settings.w_contact_force = 0.001
        kino_ID_settings.w_contact_motion = 1.0

        self.kino_ID = KinodynamicsID(self.model_handler, 1.0 / 500.0, kino_ID_settings)

        self.nq = self.model_handler.getModel().nq
        self.nv = self.model_handler.getModel().nv

        # The robot will move by itself to the q_start configuration and wait for you first command
        self.robot_if.start_async(self.model_handler.getReferenceState()[7 : self.nq])

    def _compute_base_pose_vel(self, q, dq):
        model = self.model_handler.getModel()
        data = self.data_handler.getData()
        base_frame_id = model.getFrameId("torso_link")
        foot_frame_id = model.getFrameId("left_ankle_roll_link")

        q_full = pin.neutral(model)
        q_full[7:] = q
        dq_full = np.zeros(model.nv)
        dq_full[6:] = dq

        pin.forwardKinematics(model, data, q_full, dq_full)
        pin.updateFramePlacements(model, data)

        oMfoot = data.oMf[foot_frame_id]
        oMbase = data.oMf[base_frame_id]

        footMbase = oMfoot.actInv(oMbase)

        w_v_foot = pin.getFrameVelocity(model, data, foot_frame_id, pin.ReferenceFrame.WORLD)
        w_v_base = pin.getFrameVelocity(model, data, base_frame_id, pin.ReferenceFrame.WORLD)
        w_v_foot_base = w_v_base - w_v_foot
        v_foot_base = oMbase.actInv(w_v_foot_base)

        return pin.SE3ToXYZQUAT(footMbase), v_foot_base.vector

    def _sensor_reading_callback(self, t, q, dq, ddq):
        # Reading timestamp, positions, velocities, accelerations
        # (Should be received at 500Hz approx.)
        # self.viz.display(np.array([0,0,0, 0,0,0,1] + q))

        # Sending commands
        q_des = self.model_handler.getReferenceState()[7 : self.nq]
        v_des = [0.0] * (self.nv - 6)
        kp = ([150.0, 150.0, 50.0, 50.0] + [50.0] * 2) * 2 + [50.0] + ([50.0, 50.0, 50.0, 50.0] + [50.0] * 3) * 2
        kd = ([2.0, 2.0, 1.0, 1.0] + [1.0] * 2) * 2 + [1.0] + ([1.0, 1.0, 1.0, 1.0] + [1.0] * 3) * 2

        base_pose, base_vel = self._compute_base_pose_vel(q, dq)

        tau_cmd = self.kino_ID.solve(
            t, np.concatenate((base_pose, np.array(q))), np.concatenate((base_vel, np.array(dq)))
        )
        if self.robot_if.is_ready:
            self.robot_if.send_command(q_des, v_des, tau_cmd, kp, kd)


def main(args=None):
    rclpy.init(args=args)
    node = MyApp()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
