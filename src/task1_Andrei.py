#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random
import time

class Task1:

    global counter
    counter = 0

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('publisher_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.1 # m/s

        self.pub.publish(self.vel_cmd)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("publisher node is active...")

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)


    def callback(self, msg):

        global counter

        self.vel_cmd = Twist()

        left_arc = msg.ranges[0:30]
        right_arc = msg.ranges[-30:]
        left = np.array(left_arc)
        right = np.array(right_arc)
        front_arc = np.array(left_arc + right_arc)

        i = 0
        l = 0
        r = 0

        for range in front_arc:
            if range < 0.5:
                i = i + 1

        for range in left:
            if range < 0.5:
                l = l + 1

        for range in right:
            if range < 0.5:
                r = r + 1

        if i == len(front_arc) and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1.82
            self.pub.publish(self.vel_cmd)
            counter = 0
            #print "a"

        elif r == len(right) and l < len(left) and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1.82
            self.pub.publish(self.vel_cmd)
            counter = 0
            #print "b"

        elif r < len(right) and l == len(left) and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -1.82
            self.pub.publish(self.vel_cmd)
            counter = counter + 1
            #print "c"

        elif r == 0 and l < len(left) and l != 0 and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -1.82
            self.pub.publish(self.vel_cmd)
            counter = 0
            #print "d"

        elif r < len(right) and l == 0 and r != 0 and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1.82
            self.pub.publish(self.vel_cmd)
            counter  = 0
            #print "e"

        elif r < 5 and l < 5 and counter < 6:
            self.vel_cmd.linear.x = 0.26
            self.vel_cmd.angular.z = random.uniform(-0.5, 0.5)
            self.pub.publish(self.vel_cmd)
            counter = 0
            #print "g"

        elif r < len(right) and l < len(left) and r != 0 and l != 0 and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1.82
            self.pub.publish(self.vel_cmd)
            counter = counter + 1
            #print "f"

        if counter > 5:
            counter = 0
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1.82
            self.pub.publish(self.vel_cmd)
            time.sleep(2)

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping publisher node at: {}".format(rospy.get_time()))
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0
        self.pub.publish(self.vel_cmd)


    def main_loop(self):
        while not self.ctrl_c:
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Task1()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
