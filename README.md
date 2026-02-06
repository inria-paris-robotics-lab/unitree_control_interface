Go2 Control Interface
===

## Summary
Python library to easily send joint command and read joint values to the go2 robot, with safeties and init procedure.

## Installation
The following procedure allow for installing all the dependencies.

1. Clone unitree repo and use their workspace
```bash
git clone --recurse-submodules git@github.com:unitreerobotics/unitree_ros2.git
cd unitree_ros2/cyclonedds_ws/src
```

2. Clone this repo
```bash
git clone git@github.com:inria-paris-robotics-lab/go2_control_interface.git --recursive
```

3. Create conda environment.
(It is recommended to use `mamba` instead of `conda` for faster/better dependencies solving)
```bash
mamba env create -f go2_control_interface/environment.yaml
mamba activate go2_control_interface
```
Note: ros2 humble is taken from the conda channel **robostack-staging** (and not robostack-humble), because it is built with boost 1.82 (rather than boost 1.74)...


4. Clone some dependencies (Some dependencies are not available on conda, or not with adequate versions) (vcs allow for cloning and managing multiple repo at once)
```bash
vcs import --recursive < go2_control_interface/git-deps.yaml
```

5. Build all CMake packages
* A. Move to workspace root directory
```bash
cd ..
```

* B. Build those two packages alone first, and source them (required by unitree install procedure)

```bash
colcon build --packages-select cyclonedds
source install/setup.bash
```

* C. Build all remaining packages

```bash
colcon build --packages-skip unitree_sdk2py
```

6. Source the environment
```bash
mamba activate go2_control_interface # If not already done
source install/setup.bash
```

7. Build unitree_sdk
```bash
export CYCLONEDDS_HOME="$(pwd)/install/cyclonedds"
cd src/unitree_sdk2_python
pip install -e .
```

## How to control the robot

#### 0. Setup your env
##### On the real robot
Your network interface where the robot is plugged need to be set in "manual IPV4" (e.g. hve a static ip address) with `192.168.123.222` / `255.255.255.0` address/netmask
```bash
mamba activate go2_control_interface
source install/setup.bash
source <(ros2 run go2_control_interface autoset_environment_dds.py REAL)
```
##### In simulation (using go2_simulation)
```bash
mamba activate go2_control_interface
source install/setup.bash
source <(ros2 run go2_control_interface autoset_environment_dds.py SIMULATION)
```
* Replace `---absolute path to cyclonedds workspace---`

##### GO2 topics not appearing on local machine:
Your firewall might block communication between your machine and the go2 (topics such as `/lowstate`,`/api/...` will not appear), in that case deactivate your firewall with :
```bash
sudo ufw disable
```
and retry `ros2 topic list`. If they appear now you will need to update permissions in your firewall :
```bash
sudo ufw enable # reactivate the firewall
sudo ufw allow in proto udp from 192.168.123.222 # allow UDP messages from go2 IP
sudo ufw allow in proto udp to 192.168.123.222 # allow UDP messages to go2 IP
```
You can check permissions with `sudo ufw status verbose`. You should have:
```bash
Vers                       Action      De
----                       ------      --
Anywhere                   ALLOW IN    192.168.123.222/udp
192.168.123.222/udp        ALLOW IN    Anywhere
```

#### 1. Shutdown unitree default control
When powered on, the go2 have some default unitree controllers running, to make it stand up and walk. It needs to but shutdown as it is constantly spamming the motor with its commands.
```bash
ros2 run go2_control_interface shutdown_sportsmode.py
```

#### 2. Launch watchdog
The watchdog node enforces some safeties on the robot. For instance, if the commands sent are too spaced-out in time or if the joints are out of some certain safety bounds, it kills the robot.

The go2_control_interface won't start if this node is node running.

To launch it:
```bash
ros2 launch go2_control_interface watchdog.launch.py
```

#### 3. Run your app
Here is the boilerplate/example code to write your app
```python
import rclpy
from rclpy.node import Node
from go2_control_interface_py.robot_interface import Go2RobotInterface

class MyApp(Node, ):
    def __init__(self):
        Node.__init__(self, "my_app")
        self.robot_if = Go2RobotInterface(self)
        self.robot_if.register_callback(self._sensor_reading_callback)

        # The robot will move by itself to the q_start configuration and wait for your first command
        start_q = [0.] *12
        self.robot_if.start_async(start_q)

    def _sensor_reading_callback(self, t, q, dq, ddq):
        # Reading timestamp, positions, velocities, accelerations
        # (Should be received at 500Hz approx.)

        # Sending commands
        q_des   = [0.] * 12
        v_des   = [0.] * 12
        tau_des = [0.] * 12
        kp      = [0.] * 12
        kd      = [0.] * 12

        # Call this once you app is ready to send command. (In this case can be sent directly)
        if self.robot_if.can_be_unlocked():
            # The robot will stay in position control at q_start config until you call that routine
            # The 1.0 argument will make the interface transition smoothly from the position control to your commands over a 1.0s duration
            self.robot_if.unlock(1.0)

        # This flag is True once both the robot reached the start configuration and self.robot_if.unlock() has been called.
        if self.robot_if.can_be_controlled():
            self.robot_if.send_command(q_des, v_des, tau_des, kp, kd) # Will crash if called when robot is not ready.


def main(args=None):
    rclpy.init(args=args)
    node = MyApp()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
