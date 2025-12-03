import rclpy
from rclpy.node import Node
from go2_control_interface_py.robot_interface import Go2RobotInterface
from go2_description.loader import loadG1


class MyApp(
    Node,
):
    def __init__(self):
        Node.__init__(self, "my_app2")

        self.robot = loadG1()

        self.robot_if = Go2RobotInterface(self)
        self.robot_if.register_callback(self._sensor_reading_callback)
        # self.viz = MeshcatVisualizer(self.robot.model, self.robot.collision_model, self.robot.visual_model)
        # self.viz.initViewer(open=True)
        # self.viz.loadViewerModel()
        # self.viz.display(np.array([0,0,0, 0,0,0,1] + [0.] * 27))

        # The robot will move by itself to the q_start configuration and wait for you first command
        start_q = [0.0] * 27
        self.robot_if.start_async(start_q)

    def _sensor_reading_callback(self, t, q, dq, ddq):
        # Reading timestamp, positions, velocities, accelerations
        # (Should be received at 500Hz approx.)
        # self.viz.display(np.array([0,0,0, 0,0,0,1] + q))

        # Sending commands
        q_des = [0.0] * 27
        v_des = [0.0] * 27
        tau_des = [0.0] * 27
        kp = ([150.0, 150.0, 50.0, 50.0] + [50.0] * 2) * 2 + [50.0] + ([50.0, 50.0, 50.0, 50.0] + [50.0] * 3) * 2
        kd = ([2.0, 2.0, 1.0, 1.0] + [1.0] * 2) * 2 + [1.0] + ([1.0, 1.0, 1.0, 1.0] + [1.0] * 3) * 2
        if self.robot_if.is_ready:
            self.robot_if.send_command(q_des, v_des, tau_des, kp, kd)


def main(args=None):
    rclpy.init(args=args)
    node = MyApp()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
