#!/usr/bin/python3

from geometry_msgs.msg import PoseStamped #Imports PoseStamped which record position data and time stamps it
from ros_tellopy.msg import Vector4Stamped # Import Vector4Stamped Which holds 4 Vector Points
import rosbag #Imports Rosbag which is used to record data
import sys # Imports sys which gives acess to more variables

###Assuming this is the file to actually copy the path trajectory of a drone from the previous run so that it can be followed in trajectory_publisher later

if(len(sys.argv)!=2): #If someone tries to run this on the command line without using a bag file then this won't "compile" and prints the correct way to run this file
    print("Usage: python3 extract_tracj_ctrl.py BAG_NAME")
    exit(0)

traj = open("/home/tello_project/git/ros_tello/scripts/extract_data/traj.csv","w") #Opens a csv file called 'traj.csv' to write in
traj.write("ts,x,y,z,qx,qy,qz,qw\n") #Writes a line of string and goes to the next line in traj.csv
ctrl = open("/home/tello_project/git/ros_tello/scripts/extract_data/ctrl.csv","w") #Opens a csv file called 'ctrl.csv' to write in
ctrl.write("ts,x,y,z,w\n") #Writes a line of string and goes to the next line in ctrl.csv
with rosbag.Bag(sys.argv[1], "r") as bag: #Opens a bag file in read mode as bag. | with is basically a try-finally file manager block all in one. It opens a file as a variable in one line.
    for topic, msg, t in bag.read_messages(): #For iterates by however many topic(s), msg(s), timestamps are read in the bag file 
        if topic == "/vrpn_client_node/tello1/pose": # Condition for if the topic is 'vrpn_client_node/tello1/pose'(the topic that gets MoCap position data)
            traj.write(f"{msg.header.stamp.to_sec()}," # Line 19-26 is writing what is published by the former topic into the traj csv file 
                       f"{msg.pose.position.x},"
                       f"{msg.pose.position.y},"
                       f"{msg.pose.position.z},"
                       f"{msg.pose.orientation.x},"
                       f"{msg.pose.orientation.y},"
                       f"{msg.pose.orientation.z},"
                       f"{msg.pose.orientation.w}\n"
                       )
        elif topic == "/tello1/rc_control": # Condition for if the topic is '/tello1/rc_control'(the topic that gets Drone position data)
            ctrl.write(f"{msg.header.stamp.to_sec()}," # Line 29-33 is writing what is published by the former topic into the crtl csv file
                       f"{msg.data.x},"
                       f"{msg.data.y},"
                       f"{msg.data.z},"
                       f"{msg.data.w}\n")
traj.close() #closes the traj csv file
ctrl.close() #closes the ctrl csv file
