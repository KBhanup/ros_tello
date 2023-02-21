#!/usr/bin/python3

from ros_tellopy.msg import Vector4Stamped, Vector4     # Imports Vector4Stamped and Vector4 data types
from sensor_msgs.msg import Joy                         # Imports joystick information
from std_msgs.msg import Empty, Header, Bool            # Imports data types
import rospy        # Imports the rospy library
import time         # Imports the time library


## Converts Joystick inputs to control inputs to the nodes
class JoystickControl():

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
            self.button_just_pressed = True			# Signal that a button was just pressed
        elif msg.buttons[3]:					# If Y is pressed
            self.takeoff_pub.publish(Empty())			# Publish to the takeoff topic, causing the tello to take off
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
        elif msg.buttons[0]:                        # If X is pressed
            if msg.buttons[9]:                          # If the START button is pressed
                self.start_traj_pub.publish(True)       # Publishes true to the start trajectory topic
            else:                                       # If the START button is not pressed
                self.go_to_start_pub.publish(True)      # Publishes true to the go to start topic
            self.button_just_pressed = True             # Signal that a button was just pressed
            self.send_cmd = False                       # Signals that commands to change the orientation do not need to be published
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

    ## Create Joystick controller node
    #  @param self This object pointer
    def __init__(self):					# The initalization function for the JoystickControl object
        self.send_cmd = True					# Initialize send_cmd to true, so that axis data will be immediately published
        self.button_just_pressed = False			# Initialize button_just_pressed to false, since a button has not yet been pressed
        self.cmd = Vector4Stamped(Header(), Vector4(0, 0, 0, 0))	# Initialize cmd, a Vector4Stamped object to hold a vector4 with orientation data 
        print("Node instantiated")				# Print that a node has been instantiated
        rospy.init_node("Joystick_Control")			# Initializes a Joystick_Control node

        self.joy_sub = rospy.Subscriber('joy', Joy, self.handle_joy)		# Subscribes to the data coming from the joystick, and calls handle_joy whenever it recieves data from the joystick

        self.takeoff_pub = rospy.Publisher("takeoff", Empty, queue_size=1)		# Initializes a publisher object to publish to the takeoff topic
        self.land_pub = rospy.Publisher("land", Empty, queue_size=1)			# Initializes a publisher object to publish to the landing topic
        self.emergency_pub = rospy.Publisher("emergency", Empty, queue_size=1)	# Initializes a publisher object to publish to the emergency topic
        self.stop_pub = rospy.Publisher("stop", Empty, queue_size=1)			# Initializes a publisher object to publish to the stop topic
        self.vel_pub = rospy.Publisher("rc_control", Vector4Stamped, queue_size=1)	# Initializes a publisher object to publish to the velocity topic

        self.go_to_start_pub = rospy.Publisher("/go_to_start",Bool,queue_size=1)    # Initializes a publisher object to publish to the go to start topic
        self.start_traj_pub  = rospy.Publisher("/start_traj",Bool,queue_size=1)     # Initializes a publisher object to publish to the start trajectory topic

        dT = 0.05                       # Initializes the loop interval to 0.05
        last_end = time.time()			# Initialize the last end to the current time
        time_error = 0					# Initialize potential error for time calculation
        while not rospy.is_shutdown():		# Loop while the tello is not shutdown
            if self.send_cmd:				# If axis command data stored within the cmd vector4 is to be published
                self.cmd.header.stamp = rospy.Time.now()	# Records a time stamp and stores it in the header of the cmd vector4 when the cmd data is about to be published
                self.vel_pub.publish(self.cmd)		# Publish the orientation command data stored within cmd's vector4
            elapsed = time.time() - last_end		# Sets elapsed to the time time between now and the last time the while loop ended
            if elapsed < dT + time_error:		    # If the time elapsed is less than dT plus the time error
                time.sleep(dT - elapsed + time_error)   # Waits for the next interval to come if the loop executed faster than it
            last_end = time.time()			# Sets last_end to the current time as of the end of the while loop
        print("Node deleted")				# Print that a node was deleted, since there was a shutdown if the program is past the while loop


if __name__ == "__main__":	# If this is being ran as the main program
    j = JoystickControl()	# Create a JoystickControl object named j
