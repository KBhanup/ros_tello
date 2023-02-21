#!/usr/bin/python3

from tello.tello import Tello

import rospy

## This script change the name of the network that the tello creates.
#  A password is required.
#  ex. rosrun ros_tellopy rename_network.py _ap:="MyNetwork" _passwd:="password"
#  ~ip:=192.168.10.1 (Drone ip to connect to.)
#  ~port:=8889 (Local port to connect from.)
#  ~ap:="SSID" (Name of the network to create.)
#  ~passwd:="password" (Password of the network.)

rospy.init_node("setup_tello")
ip = rospy.get_param("~ip", "192.168.10.1")
port = rospy.get_param("~port", 8889)
wifi = rospy.get_param("~ap", "SSID")
passwd = rospy.get_param("~passwd", "password")
assert(passwd != "")

print(f"Connecting to drone at {ip}:{port}")
drone = Tello(ip, port)
print(f"Requesting to rename network to \"{wifi}\" with password \"{passwd}\"")
drone.set_wifi(wifi, passwd)
print(f"Process complete. The drone should be restarting. \n"
	  f"Please connect to {wifi}. If you do not see the drone,\n"
	  f"with the drone on, hold the button on the side until\n"
	  f"it restarts and check the inputs to this program.")
exit(0)
