#!/usr/bin/python3

from tello.tello import Tello

import rospy

## This script connects the tello to an AP
#  ex. rosrun ros_tellopy connect_router.py _ap:="MyNetwork" _passwd:="password"
#  ~ip:=192.168.10.1 (Drone ip to connect to.)
#  ~port:=8889 (Local port to connect from.)
#  ~ap:="SSID" (Name of the network to connect to.)
#  ~passwd:="password" (Password of the network.)

rospy.init_node("setup_tello")
ip = rospy.get_param("~ip","192.168.10.1")
port = rospy.get_param("~port",8889)
ap = rospy.get_param("~ap","USRLNet")
passwd = rospy.get_param("~passwd","dronesArena")

print(f"Connecting to drone at {ip}:{port}")
drone = Tello(ip,port)
print(f"Requesting to connect to network \"{ap}\" with password \"{passwd}\"")
drone.ap(ap,passwd)
print(f"Process complete. The drone should be restarting. \n"
	  f"Please connect to {ap}. If you do not see the drone,\n"
	  f"with the drone on, hold the button on the side until\n"
	  f"it restarts and check the inputs to this program.")
exit(0)
