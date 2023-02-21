#!/usr/bin/python3

from ros_tellopy.msg import State
from geometry_msgs.msg import PoseStamped

from KF import KF
from Utilities import quat_to_euler

import numpy as np
import rospy


## Converts Joystick inputs to control inputs to the nodes
class KFNode():
    def update_state(self, header):
        self.state.header = header
        self.state.pos.x = self.kf.x[0]
        self.state.pos.y = self.kf.x[1]
        self.state.pos.z = self.kf.x[2]
        self.state.pos.w = self.kf.x[3]
        self.state.vel.x = self.kf.x[4]
        self.state.vel.y = self.kf.x[5]
        self.state.vel.z = self.kf.x[6]
        self.state.vel.w = self.kf.x[7]

    def handle_vrpn(self, msg: PoseStamped):
        if (self.kf == None):
            her = quat_to_euler([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w])

            x_0 = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z,
                            her[0],
                            0,
                            0,
                            0,
                            0]).reshape((-1,1))

            self.kf = KF(x_0)
            self.update_state(msg.header)

        else:
            dt = msg.header.stamp.to_sec() - self.state.header.stamp.to_sec()
            self.kf.predict(dt)

            her = quat_to_euler([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w])

            obs = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z,
                            her[0]]).reshape((-1,1))
            self.kf.update(obs)
            self.update_state(msg.header)

        self.filt_pub.publish(self.state)

    def __init__(self):
        self.kf = None
        self.state = State()

        rospy.init_node("Joystick_Control")

        self.vrpn_sub = rospy.Subscriber('vrpn', PoseStamped, self.handle_vrpn)
        self.filt_pub = rospy.Publisher("filter", State, queue_size=1)

        rospy.spin()


if __name__ == "__main__":
    n = KFNode()
