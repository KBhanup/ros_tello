#!/usr/bin/python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import BatteryState, Image

from std_msgs.msg import Int32, Header, Float32, Empty
from sensor_msgs.msg import Imu

from ros_tellopy.msg import Vector3Array, Vector4Stamped, Vector4
from tello.tello import Tello


class Single:
	def handle_emergency(self, msg):
		self.drone.emergency()

	def handle_stop(self, msg):
		self.drone.stop()

	def handle_takeoff(self, msg):
		self.drone.takeoff()

	def handle_land(self, msg):
		self.drone.land()

	def handle_up(self, msg):
		self.drone.up(msg.data)

	def handle_down(self, msg):
		self.drone.down(msg.data)

	def handle_left(self, msg):
		self.drone.left(msg.data)

	def handle_right(self, msg):
		self.drone.right(msg.data)

	def handle_forward(self, msg):
		self.drone.forward(msg.data)

	def handle_back(self, msg):
		self.drone.back(msg.data)

	def handle_cw(self, msg):
		self.drone.cw(msg.data)

	def handle_ccw(self, msg):
		self.drone.ccw(msg.data)

	def handle_flip(self, msg):
		self.drone.flip(msg.data)

	def handle_go(self, msg):
		self.drone.go(msg.pos.x, msg.pos.y, msg.pos.z, msg.speed)

	def handle_curve(self, msg):
		self.drone.curve(msg.data[0].x, msg.data[0].y, msg.data[0].z, msg.data[1].x, msg.data[1].y, msg.data[1].z)

	def handle_speed(self, msg):
		self.drone.set_speed(msg.data)

	def handle_rc_control(self, msg):
		self.drone.rc_control(int(msg.data.x*100), int(msg.data.y*100), int(msg.data.z*100), int(msg.data.w*100))

	def handle_state(self, data, ip):
		header = Header()
		header.frame_id = self.name
		header.seq = self.state_seq
		header.stamp = rospy.Time.now()
		self.state_seq += 1

		batt_msg = BatteryState()
		batt_msg.header = header
		batt_msg.percentage = data["bat"]
		self.batt_pub.publish(batt_msg)

		imu_msg = Imu()
		imu_msg.header = header
		imu_msg.angular_velocity.x = data["vgx"]
		imu_msg.angular_velocity.y = data["vgy"]
		imu_msg.angular_velocity.z = data["vgz"]
		imu_msg.linear_acceleration.x = data["agx"]
		imu_msg.linear_acceleration.y = data["agy"]
		imu_msg.linear_acceleration.z = data["agz"]
		imu_msg.orientation.x = data["roll"]
		imu_msg.orientation.x = data["pitch"]
		imu_msg.orientation.x = data["yaw"]
		self.imu_pub.publish(imu_msg)

		self.baro_pub.publish(Float32(data["baro"]))
		self.height_pub.publish(Float32(data["h"]))
		self.temp_pub.publish(Float32(data["temph"]))
		self.dist_pub.publish(Float32(data["tof"]))
		self.time_pub.publish(Float32(data["time"]))

	def __init__(self):
		self.state_seq = 0
		self.loop_time = 1
		self.running = True

		rospy.init_node("single")

		tello_info = rospy.get_param("~robots")
		local_port = 8889
		self.name = tello_info[0]['name']
		self.ip = tello_info[0]['ip']

		self.batt_pub = rospy.Publisher(self.name + "/battery", BatteryState, queue_size=1)
		self.imu_pub = rospy.Publisher(self.name + "/imu", Imu, queue_size=1)
		self.baro_pub = rospy.Publisher(self.name + "/baro", Float32, queue_size=1)
		self.height_pub = rospy.Publisher(self.name + "/height", Float32, queue_size=1)
		self.temp_pub = rospy.Publisher(self.name + "/temp", Float32, queue_size=1)
		self.dist_pub = rospy.Publisher(self.name + "/flight_distance", Float32, queue_size=1)
		self.time_pub = rospy.Publisher(self.name + "/flight_time", Float32, queue_size=1)

		self.emergency = rospy.Subscriber(self.name + '/emergency', Empty, self.handle_emergency)
		self.stop = rospy.Subscriber(self.name + '/stop', Empty, self.handle_stop)
		self.takeoff = rospy.Subscriber(self.name + '/takeoff', Empty, self.handle_takeoff)
		self.land = rospy.Subscriber(self.name + '/land', Empty, self.handle_land)
		if False:
			self.up = rospy.Subscriber(self.name + '/up', Float32, self.handle_up)
			self.down = rospy.Subscriber(self.name + '/down', Float32, self.handle_down)
			self.left = rospy.Subscriber(self.name + '/left', Float32, self.handle_left)
			self.right = rospy.Subscriber(self.name + '/right', Float32, self.handle_right)
			self.forward = rospy.Subscriber(self.name + '/forward', Float32, self.handle_forward)
			self.back = rospy.Subscriber(self.name + '/back', Float32, self.handle_back)
			self.cw = rospy.Subscriber(self.name + '/cw', Float32, self.handle_cw)
			self.ccw = rospy.Subscriber(self.name + '/ccw', Float32, self.handle_ccw)
			self.flip = rospy.Subscriber(self.name + '/flip', String, self.handle_flip)
			self.curve = rospy.Subscriber(self.name + '/curve', Vector3Array, self.handle_curve)

		self.go = rospy.Subscriber(self.name + '/go', Vector4, self.handle_go)
		self.speed = rospy.Subscriber(self.name + '/speed', Float32, self.handle_speed)
		self.rc_control = rospy.Subscriber(self.name + '/rc_control', Vector4Stamped, self.handle_rc_control)

		self.drone = Tello(self.ip, local_port)
		self.drone.start_data_stream(self.handle_state)

		while not rospy.is_shutdown():
			self.drone.command(False)
			rospy.sleep(self.loop_time)
		self.running = False


if __name__ == "__main__":
	single = Single()
