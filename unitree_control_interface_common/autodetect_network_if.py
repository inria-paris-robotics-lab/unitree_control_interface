import socket
import fcntl
import struct
import subprocess


class UnitreeNetworkInfo:
    ROBOT_SUBNET = "192.168.123"
    ROBOT_HOST = ".18"
    JETSON_HOST = ".161"

    def getIPInfo(self, ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            ip_address = fcntl.ioctl(
                s.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack("256s", ifname[:15].encode("utf-8")),
            )[20:24]
            return socket.inet_ntoa(ip_address)
        except IOError:
            return None

    def getRobotInterfaceNameIp(self):
        # Return a list of network interface information
        nameindex_array = socket.if_nameindex()

        for index, ifname in nameindex_array:
            ipaddr = self.getIPInfo(ifname)
            if ipaddr is not None:
                if ipaddr.startswith(self.ROBOT_SUBNET):
                    return ifname, ipaddr
        return None, None

    # hardcoded IP address
    def ping(self, host):
        res = subprocess.run(
            ["ping", "-q", "-c", "2", "-W", "1", "-t", "100", self.ROBOT_SUBNET + host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if res.returncode != 0:
            # not up
            return False
        return True


def main(args=None):
    robot_net = UnitreeNetworkInfo()
    ifname, ifip = robot_net.getRobotInterfaceNameIp()
    print(f"DDS Interface name: {ifname}")
    print(f"IP address:         {ifip}")

    print("\nPinging devices:")
    robot_up = robot_net.ping(robot_net.ROBOT_HOST)
    jetson_up = robot_net.ping(robot_net.JETSON_HOST)
    print(f"Robot  is {'  ' if robot_up else 'NOT'} reachable ({robot_net.ROBOT_SUBNET + robot_net.ROBOT_HOST})")
    print(f"Jetson is {'  ' if jetson_up else 'NOT'} reachable ({robot_net.ROBOT_SUBNET + robot_net.ROBOT_HOST})")


if __name__ == "__main__":
    main()
