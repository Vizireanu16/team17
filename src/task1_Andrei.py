#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Task1:

    """
    def callback(self, msg):
        self.move = Twist()

        left_arc = msg.ranges[0:10]
        right_arc = msg.ranges[-10:]
        front_arc = np.array(left_arc + right_arc)
        front_range = front_arc.min()


        while front_range > 0.01:
            left_arc = msg.ranges[0:10]
            right_arc = msg.ranges[-10:]
            print (left_arc)
            print (right_arc)
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
        """
    def callback(self, msg):
        print msg.ranges[0]
        #self.vel_cmd.linear.x = 0.1
        left_arc = msg.ranges[0:10]
        right_arc = msg.ranges[-10:]
        front_arc = np.array(left_arc + right_arc)

        rotate = False
        i = 0

        for range in front_arc:
            if range < 0.4:
                i = i + 1

        #print front_arc
        #print ("ranges: ",front_arc)
        #print ("i: ", i)
        print i
        print len(front_arc)
        if i == len(front_arc):
            rotate = True

        if rotate == True:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0.5
            self.pub.publish(self.vel_cmd)
            rospy.sleep(1) # Sleeps for 1 sec
            self.vel_cmd.angular.z = 0
            self.pub.publish(self.vel_cmd)
            print "Good"

        if msg.ranges[0] < 0.3:
            self.vel_cmd.linear.x = 0
            self.pub.publish(self.vel_cmd)

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
    publisher_instance = Task1()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
