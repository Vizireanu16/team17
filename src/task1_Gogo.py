#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Task1Gogo():
#robot should search environment every 0.1s
    def callback(self, data):
        print("Subscriber obtained the following message: \"{}\"".format(data.data))
        turn_left = 0
        turn_right = 0

    def _init_():
         def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.init_node('publisher_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.1 # m/s
        print ("init")

        self.pub.publish(self.vel_cmd)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("publisher node is active...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping publisher node at: {}".format(rospy.get_time()))
        self.vel_cmd.linear.x = 0.0 # m/s
        self.pub.publish(self.vel_cmd)

    def main_loop(self):
        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
                print("init")
            elif self.turn:
                if abs(self.init_yaw - self.yaw) >= pi/2:
                    # If the robot has turned 90 degrees (in radians) then stop turning
                    self.turn = False
                    # publish a default twist message:
                    self.vel = Twist()
                    # reset the init_yaw value for the next turn operation:
                    self.init_yaw = self.yaw
                    print("turn-fwd transition")
                else:
                    # if not, then keep on turning at 0.2 rad/s:
                    self.vel = Twist()
                    self.vel.angular.z = 0.2
                    print("turning")
            else:
                if sqrt(pow(self.init_x - self.x, 2) + pow(self.init_y - self.y, 2)) >= 0.5:
                    # if distance travelled is greater than 0.5m then stop, and start turning:
                    self.vel = Twist()
                    self.turn = True
                    # reset the init_x & y values for the next forwards operation:
                    self.init_x = self.x
                    self.init_y = self.y
                    print("fwd-turn transition")
                else:
                    # if not, then keep on moving forward at 0.1 m/s:
                    self.vel = Twist()
                    self.vel.linear.x = 0.1
                    print("moving forwards")
            # publish whatever velocity command has been set in the above:
            self.pub.publish(self.vel)
            # print a status message:
            self.print_current_velocity_command()
            # maintain the loop rate @ 10 hz
            self.rate.sleep()
    

#robot should turn 90 degrees when obstacle is spotted