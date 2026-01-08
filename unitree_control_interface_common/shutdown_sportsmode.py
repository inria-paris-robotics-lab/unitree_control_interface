#!/usr/bin/env python3

import time
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient  # This seems to work for G1 aswell

from unitree_control_interface_common.autodetect_network_if import UnitreeNetworkInfo


class UnitreeShutdown:
    def __init__(self, netwif):
        self.completed = False
        ChannelFactoryInitialize(0, netwif)

    # Public methods
    def SwitchOff(self):
        self.sc = SportClient()
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()

        while result["name"]:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        self.completed = True
        return 0


def main():
    ifname, ifip = UnitreeNetworkInfo().getRobotInterfaceNameIp()

    client = UnitreeShutdown(ifname)
    print(f"Connected to robot on if {ifname}")

    print("Switch off sportsmode...")
    client.SwitchOff()

    while True:
        if client.completed == 1.0:
            time.sleep(1)
            print("Done!")
            sys.exit(0)
        time.sleep(1)


if __name__ == "__main__":
    main()
