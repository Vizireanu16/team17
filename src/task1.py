#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Tasks:

    def callback(self, msg):
        self.move = Twist()

        left_arc = msg.ranges[0:10]
        right_arc = msg.ranges[-10:]
        front_arc = np.array(left_arc + right_arc)
        front_range = front_arc.min()


        while front_range > 0.01:
            left_arc = msg.ranges[0:10]
            right_arc = msg.ranges[-10:]
            front_arc = np.array(left_arc + right_arc)
            front_range = front_arc.min()
            print front_range
            self.move.linear.x = 0.1
            print self.move.linear.x

        self.move.linear.x = 0
        self.pub.publish(self.move)


        #rospy.init_node("check_obstacle")
        #sub = rospy.Subscriber("/scan", LaserScan, callback)
        #pub = rospy.Publisher("/cmd_vel", Twist)
        #move = Twist()
        rospy.spin()


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
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Tasks()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
