#!/usr/bin/env python3

import sys
from unitree_control_interface_common.autodetect_network_if import UnitreeNetworkInfo


class UnitreeEnvSetup:
    def generateEnv(self, sim: bool) -> None:
        unitree_net_info = UnitreeNetworkInfo()
        ifname, ifip = unitree_net_info.getRobotInterfaceNameIp()

        self.output("\033[1;4mExecuting:\033[0m", display_only=True)
        self.output("export RCUTILS_COLORIZED_OUTPUT=1")
        if sim:
            self.output("unset RMW_IMPLEMENTATION")
            self.output("unset CYCLONEDDS_URI")
            self.output("export ROS_LOCALHOST_ONLY=1")

        else:
            self.output("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
            self.output(f'''export CYCLONEDDS_URI=\'<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="{ifname}" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>\'
                        ''')
            self.output("export ROS_LOCALHOST_ONLY=0")

    def output(self, str: str, *, display_only: bool = False) -> None:
        print(str, file=sys.stderr)
        if not display_only:
            print(str)


def main(args=None):
    is_sim = None
    # Try to parse argument
    try:
        if len(sys.argv) != 2:
            raise ValueError
        if sys.argv[1] == "SIMULATION":
            is_sim = True
        elif sys.argv[1] == "REAL":
            is_sim = False
        else:
            raise ValueError
    except ValueError:
        print("Error expecting exactly one argument being SIMULATION or REAL")
        exit()
    robot_env = UnitreeEnvSetup()
    robot_env.generateEnv(sim=is_sim)


if __name__ == "__main__":
    main()
