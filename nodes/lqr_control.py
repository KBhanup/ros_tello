#!/usr/bin/python3

from ros_tellopy.msg import Vector4Stamped, Vector4, State
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, Header
import rospy
import time


## Converts Joystick inputs to control inputs to the nodes
class LQRControl():
    ## Joy message callback
    #  Y: Takeoff
    #  X or B: emergency shutoff
    #  A: Land
    #  Left joystick: vertical velocity/yaw
    #  Right joystick: horizontal velocity
    #  @param self This object pointer.
    #  @param msg Joy message
    def handle_joy(self, msg: Joy):

        self.allow_lqr = False
        self.send_cmd = False

        # Emergency
        if msg.buttons[0] or msg.buttons[2]:
            self.emergency_pub.publish(Empty())
            self.button_just_pressed = True
        # Takeoff
        elif msg.buttons[3]:
            self.takeoff_pub.publish(Empty())
            self.button_just_pressed = True
        # Land
        elif msg.buttons[1]:
            self.land_pub.publish(Empty())
            self.button_just_pressed = True
        # Use LQR
        elif msg.buttons[4] or msg.buttons[5]:
            self.allow_lqr = True
            self.button_just_pressed = True
        # Use joy
        elif not self.button_just_pressed:
            self.cmd.data.x = msg.axes[3]
            self.cmd.data.y = msg.axes[2]
            self.cmd.data.z = msg.axes[1]
            self.cmd.data.w = msg.axes[0]
            self.send_cmd = True
        # anything else
        else:
            self.button_just_pressed = False

    def handle_pose(self, msg: State):
        self.pose = np.array([
            msg.pos.x,
            msg.pos.y,
            msg.pos.z,
            msg.pos.w,
        ]).reshape((-1, 1))

    def handle_goal(self, msg: State):
        self.goal = np.array([
            msg.pos.x,
            msg.pos.y,
            msg.pos.z,
            msg.pos.w,
        ]).reshape((-1, 1))

    ## Create Joystick controller node
    #  @param self This object pointer
    def __init__(self):
        self.pose = None
        self.goal = None
        self.send_cmd = True
        self.allow_lqr = False
        self.button_just_pressed = False
        self.cmd = Vector4Stamped(Header(), Vector4(0, 0, 0, 0))
        self.lqr = LQR("../scripts/controller.csv")

        rospy.init_node("Joystick_Control")
        self.joy_sub = rospy.Subscriber('joy', Joy, self.handle_joy)
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.handle_pose)
        self.goal_sub = rospy.Subscriber('goal', PoseStamped, self.handle_goal)
        self.vel_pub = rospy.Publisher("rc_control", Vector4Stamped, queue_size=1)

        dT = 0.05
        last_end = time.time()
        while not rospy.is_shutdown():
            if self.send_cmd:
                self.cmd.header.stamp = rospy.Time.now()
                self.vel_pub.publish(self.cmd)
            elif self.allow_lqr:
                if (self.goal is not None) and (self.pose is not None):
                    ctrl = self.lqr.get_ctrl(self.goal, self.pose)
                    cmd = Vector4Stamped()
                    cmd.header.stamp = rospy.Time.now()
                    cmd.data.x = ctrl[0]
                    cmd.data.y = ctrl[1]
                    cmd.data.z = ctrl[2]
                    cmd.data.w = ctrl[3]
                    self.vel_pub.publish(cmd)

            elapsed = time.time() - last_end
            if elapsed < dT + time_error:
                time.sleep(dT - elapsed + time_error)
            last_end = time.time()
        print("Node deleted")


if __name__ == "__main__":
    j = JoystickControl()
