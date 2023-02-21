#!/usr/bin/python3

from ros_tellopy.msg import Vector4Stamped, Vector4, State     # Imports Vector4Stamped and Vector4 data types
from sensor_msgs.msg import Joy                         # Imports joystick information
from std_msgs.msg import Empty, Header, Bool            # Imports data types
from geometry_msgs.msg import PoseStamped                   # Imports PoseStamped from ROS geometry_msgs
from quat_lib import quat2rotm, quat2eul
from tello_lmpc.msg import Setpoint ##Imports Setpoint only since it's dynamic and will need to update accordingly
import rospy        # Imports the rospy library
import time         # Imports the time library
import numpy as np  # Imports the numpy library

## Converts Joystick inputs to control inputs to the nodes
class JoystickControl():

    ## Allows for PD control to reach a setpoint
    def PDControl(self, setX, setY, setZ, kPz, kDz, kPx, kDx, kPy, kDy):
        #print(self.position_x)
    
        xError = setX - self.position_x        # Calculates the current x error
        #elf.xInt += xError              # Adds the current x error to the x error integral
        self.xInput = (kPx * xError) + (kDx * (xError - self.xPrevError))    # Calculates the P, I, and D components, adds them, and saves it as input
        ##print('Input X =', self.xInput)
        self.xPrevError = xError         # Sets the previous x error to the current x error
            
        yError = setY - self.position_y      # Calculates the current y error
        #self.yInt += yError              # Adds the current y error to the y error integral
        self.yInput = (kPy * yError) + (kDy * (yError - self.yPrevError))    # Calculates the P, I, and D components, adds them, and saves it as input
        ##print('Input Y =', self.yInput)
        self.yPrevError = yError         # Sets the previous y error to the current y error
            
        zError = setZ - self.position_z        # Calculates the current z error
        #self.zInt += zError              # Adds the current z error to the z error integral
        self.zInput = (kPz * zError) + (kDz * (zError - self.zPrevError))    # Calculates the P, I, and D components, adds them, and saves it as input
        ##print('Input Z =', self.zInput)
        self.zPrevError = zError         # Sets the previous z error to the current z error

        self.R = quat2rotm(self.quat_w, self.quat_x, self.quat_y, self.quat_z)


        self.input_body = np.matrix([self.xInput, self.yInput, self.zInput]).T
        self.input_world = self.R * self.input_body
        # Limit input to between -1 and 1
        if self.input_world[0] > 1:
            self.input_world[0] = 1
        elif self.input_world[0] < -1:
            self.input_world[0] = -1
        if self.input_world[1] > 1:
            self.input_world[1] = 1
        elif self.input_world[1] < -1:
            self.input_world[1] = -1
        if self.input_world[2] > 1:
            self.input_world[2] = 1
        elif self.input_world[2] < -1:
            self.input_world[2] = -1
        self.cmd.data.x = self.input_world[0,0]
        self.cmd.data.y = self.input_world[1,0]
        self.cmd.data.z = self.input_world[2,0]
        self.cmd.data.w = 0
        # We've initiated the Pd Controller here, but the goal is to use the x button to break out of this while loop. we'll check before the PD controller updates each time if x is pushed
        self.send_cmd = True        # Signals that commands to change orientation need to be published
        #while self.PDActive == True:
        #    self.handle_joy(Joy)


    ## Joy message callback
    ## Assumes defualt control bindings
    #  Y: Takeoff
    #  B: Emergency Shutoff
    #  A: Land
    #  X: Go to Start
    #  X + START: Start Trajectory
    #  Left joystick: Vertical Velocity/Yaw
    #  Right joystick: Horizontal Velocity
    #  Directional Pad: Slow Horizontal Velocity
    #  RB/RT: Slow Vertical Velocity
    #  LB/LT: Slow Yaw
    #  @param self This object pointer.
    #  @param msg Joy message
    def handle_joy(self, msg: Joy):		# The function that handles joystick data within the JoystickControl object, which is called when data is recieved from the subscribed 									
                                        # joystick      
        if msg.buttons[2]:					# If B is pressed
            self.emergency_pub.publish(Empty())		# Publish to the emergency topic, triggering an emergency shutoff
            self.send_cmd = False				# Signals that commands to change orientation do not need to be published
            print('PUBLISHED: emergency')
            self.button_just_pressed = True			# Signal that a button was just pressed
        elif msg.buttons[3]:					# If Y is pressed
            self.takeoff_pub.publish(Empty())			# Publish to the takeoff topic, causing the tello to take off
            print('PUBLISHED: takeoff')
            self.send_cmd = False				# Signals that commands to change orientation do not need to be published
            self.button_just_pressed = True			# Signal that a button was just pressed
        elif msg.buttons[1]:					# If A is pressed
            self.land_pub.publish(Empty())			# Publish to the land topic, causing the tello to land
            self.send_cmd = False				# Signals that commands to change orientation do not need to be published
            self.button_just_pressed = True			# Signal that a button was just pressed
        elif max(msg.buttons[4:8]) > 0 or msg.axes[4] != 0 or msg.axes[5] != 0:	# If the bumpers, triggers, or the directional pad have input
            self.cmd.data.x = .2*(msg.axes[5])		# Sets the x value of cmd's vector4 to the up/down axis data from the cross pad multiplied by 0.2, causing the tello to change its pitch 
                                                    # slowly, leading to movement along the x axis once cmd is published
            self.cmd.data.y = .2*(msg.axes[4])		# Sets the y value of cmd's vector4 to the up/down axis data from the cross pad multiplied by 0.2, causing the tello to change its roll 									
                                                    # slowly, leading to movement along the x axis once cmd is published
            self.cmd.data.z = .2*(msg.buttons[4] - msg.buttons[6])      # Sets the z value of cmd's vector4 to 0.2 if the left bumper is pressed, -0.2 if the left trigger is pressed, and 
                                                                        # 0 if both or neither are pressed, causing vertical movement of the tello once the cmd data is published
            self.cmd.data.w = .2*(msg.buttons[5] - msg.buttons[7])      # Sets the w value of cmd's vector4 to 0.2 if the right bumper is pressed, -0.2 if the right trigger is pressed, and 
                                                        # 0 if both or neither are pressed, changing the yaw of the tello once the cmd data is published
        elif msg.buttons[9]:            # If START is pressed
            self.PDActive = True       # Signal that the PD controller should be active
            self.button_just_pressed = True     # Signal that a button was just pressed
            self.send_cmd = False       # Signals that commands to change orientation do not need to be published
        elif msg.buttons[0]:            # If X is pressed
            self.PDActive = False      # Signal that the PD controller should not be active
            self.button_just_pressed = True     # Signal that a button was just pressed
            self.send_cmd = False       # Signals that commands to change orientation do not need to be published

        #elif msg.buttons[0]:                        # If X is pressed
        #    if msg.buttons[9]:                          # If the START button is pressed
        #        self.start_traj_pub.publish(True)       # Publishes true to the start trajectory topic
        #    else:                                       # If the START button is not pressed
        #        self.go_to_start_pub.publish(True)      # Publishes true to the go to start topic
        #    self.button_just_pressed = True             # Signal that a button was just pressed
        #    self.send_cmd = False                       # Signals that commands to change the orientation do not need to be published

        elif not self.button_just_pressed:			# If a button was not just pressed
            self.cmd.data.x = msg.axes[3]			# Sets the x value of cmd's vector4 to the up/down axis data from the right stick, causing the tello to change its pitch, leading to 									
                                                    # movement along the x axis once cmd is published
            self.cmd.data.y = msg.axes[2]			# Sets the y value of cmd's vector4 to the left/right axis data from the right stick, causing the tello to change its roll, leading to
                                                    # movement along the y axis once cmd is published
            self.cmd.data.z = msg.axes[1]			# Sets the z value of cmd's vector4 to the up/down axis data from the left stick, causing the tello to move vertically along the z axis
                                                    # once cmd is published
            self.cmd.data.w = msg.axes[0]			# Sets the w value of cmd's vector4 to the left/right axis data from the left stick, changing the yaw of the tello once cmd is published
            self.send_cmd = True                    # Signals that commands to change orientation need to be published
        else:							# Otherwise, if there is no input but button_just_pressed is not set to false
            self.button_just_pressed = False			# Set button_just_pressed to false, as a button has no longer just been pressed
            self.start_traj_pub.publish(False)          # Publishes false to the start trajectory topic
            self.go_to_start_pub.publish(False)         # Publishes false to the go to start topic
        

    def handle_pose(self, msg: State):      # Saves the position data from the tello in an array
        self.pose = np.array([
            msg.pos.x,
            msg.pos.y,
            msg.pos.z,
            msg.pos.w,
        ]).reshape((-1, 1))

    ##Records Drone Mocap Positions##
    def handle_tello_position(self, msg):
        self.position_x = msg.pose.position.x
        self.position_y = msg.pose.position.y
        self.position_z = msg.pose.position.z
        #print(self.position_x)
        #print(self.position_y)
        #print(self.position_z)
        self.quat_x = msg.pose.orientation.x
        self.quat_y = msg.pose.orientation.y
        self.quat_z = msg.pose.orientation.z
        self.quat_w = msg.pose.orientation.w

    ##Grabs the Setpoint from the publisher##
    def handle_stick_setpoint(self, msg):
        self.recieved_setX = msg.position.x
        self.recieved_setY = msg.position.y
        self.recieved_setZ = msg.position.z

    ## Create Joystick controller node
    #  @param self This object pointer
    def __init__(self, rate):					# The initalization function for the JoystickControl object
        self.send_cmd = True					# Initialize send_cmd to true, so that axis data will be immediately published
        self.button_just_pressed = False			# Initialize button_just_pressed to false, since a button has not yet been pressed
        self.cmd = Vector4Stamped(Header(), Vector4(0, 0, 0, 0))	# Initialize cmd, a Vector4Stamped object to hold a vector4 with orientation data 
        print("Node instantiated")				# Print that a node has been instantiated
        rospy.init_node("Joystick_Control")			# Initializes a Joystick_Control node   
        
        ### PD INITIALIZATION ###
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.quat_w = 0.0
        self.PDActive = False      # Initializes the PD controller as inactive
        self.xInput = 0         # Initializes the PD x input
        self.yInput = 0         # Initializes the PD y input
        self.zInput = 0         # Initializes the PD z input
        self.xPrevError = 0    # Initializes the previous x error
        self.yPrevError = 0    # Initializes the previous y error 
        self.zPrevError = 0    # Initializes the previous z error

        ### SUBSCRIBER INITIALIZATION ###
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.handle_joy)		# Subscribes to the data coming from the joystick, and calls handle_joy whenever it recieves data from the joystick
        self.pose_sub = rospy.Subscriber('/tello1/pose', PoseStamped, self.handle_pose)
        self.tello_position_sub = rospy.Subscriber('vrpn_client_node/tello1/pose', PoseStamped, self.handle_tello_position)
        self.setpoint_sub = rospy.Subscriber('/tello1/set_point', Setpoint, self.handle_stick_setpoint)

        ### PUBLISHER INITIALIZATION ###
        self.takeoff_pub = rospy.Publisher("/tello1/takeoff", Empty, queue_size=1)		# Initializes a publisher object to publish to the takeoff topic
        self.land_pub = rospy.Publisher("/tello1/land", Empty, queue_size=1)			# Initializes a publisher object to publish to the landing topic
        self.emergency_pub = rospy.Publisher("/tello1/emergency", Empty, queue_size=1)	# Initializes a publisher object to publish to the emergency topic
        self.stop_pub = rospy.Publisher("/tello1/stop", Empty, queue_size=1)			# Initializes a publisher object to publish to the stop topic
        self.vel_pub = rospy.Publisher("/tello1/rc_control", Vector4Stamped, queue_size=1)	# Initializes a publisher object to publish to the velocity topic

        self.go_to_start_pub = rospy.Publisher("/tello1/go_to_start",Bool,queue_size=1)    # Initializes a publisher object to publish to the go to start topic
        self.start_traj_pub  = rospy.Publisher("/tello1/start_traj",Bool,queue_size=1)     # Initializes a publisher object to publish to the start trajectory topic

        r = rospy.Rate(rate)
        while not rospy.is_shutdown():		# Loop while the tello is not shutdown
            if self.PDActive == True:      # If the PD controller is active, call the PDControl function
                self.PDControl(self.recieved_setX, self.recieved_setY, self.recieved_setZ, 1.8, 5, 1.65, 5.8, 2.8, 7.75)
            
            if self.send_cmd:				# If axis command data stored within the cmd vector4 is to be published
                self.cmd.header.stamp = rospy.Time.now()	# Records a time stamp and stores it in the header of the cmd vector4 when the cmd data is about to be published
                self.vel_pub.publish(self.cmd)		# Publish the orientation command data stored within cmd's vector4

            r.sleep()
        print("Node deleted")				# Print that a node was deleted, since there was a shutdown if the program is past the while loop


if __name__ == "__main__":	# If this is being ran as the main program
    j = JoystickControl(20)	# Create a JoystickControl object named j