#!/usr/bin/python3

import time
import rospy as rp #imports rospy library as rp
import threading

from geometry_msgs.msg import PoseStamped 
from tello_lmpc.msg import Setpoint #Imports Setpoint and SetpointArray which log position and velocity data in a 3d plane/space

class StickPosition():

    def setpointPublisher(self):    
        #Here is where would create the set point as well by using the position of the stick#
        ###Projected Set Points###
        self.set_point = Setpoint()
        r = rp.Rate(self.rate)

        while not rp.is_shutdown():
            self.set_point.position.x = self.stick_x + 1.5
            self.set_point.position.y = self.stick_y
            self.set_point.position.z = self.stick_z
            self.set_point.velocity.x = 0
            self.set_point.velocity.y = 0
            self.set_point.velocity.z = 0
            self.set_point_sender.publish(self.set_point)
            r.sleep()         # makes the node run at a certain frequency

    def handle_stick_position(self, msg):    
        self.stick_x = msg.pose.position.x
        self.stick_y = msg.pose.position.y
        self.stick_z = msg.pose.position.z
        self.stick_quat_x = msg.pose.orientation.x
        self.stick_quat_y = msg.pose.orientation.y
        self.stick_quat_z = msg.pose.orientation.z
        self.stick_quat_w = msg.pose.orientation.w
        

    def __init__(self, rate):
        # Start node
        rp.loginfo("Node instantiated")
        self.stick_x = 0
        self.stick_y = 0
        self.stick_z = 0
        self.stick_quat_x = 0
        self.stick_quat_y = 0
        self.stick_quat_z = 0
        self.stick_quat_w = 0
        rp.init_node('stick_setpoint_publisher') #Initializes the stick_setpoint_publisher(File name) node 
        self.stick_position_sub = rp.Subscriber('vrpn_client_node/stick/pose', PoseStamped, self.handle_stick_position)
        self.set_point_sender = rp.Publisher('/tello1/set_point', Setpoint, queue_size=1)

        self.rate = rate
        setpoint_thread = threading.Thread(target=self.setpointPublisher)
        setpoint_thread.start()

        rp.loginfo("Stick Node stopped")

if __name__ == "__main__":
    #Run Node
    StickPosition(20) #Publishes the setpoint using this function
    
