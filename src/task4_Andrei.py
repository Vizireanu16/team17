#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random
import time
from move_tb3 import MoveTB3

class Task4:

    global counter
    counter = 0

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('publisher_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.1 # m/s

        self.pub.publish(self.vel_cmd)
        self.robot_controller = MoveTB3()
        #self.move_rate = 'fast'
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
        nr = 0

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
            if nr == 0:
                self.vel_cmd.linear.x = 0
                self.vel_cmd.angular.z = -1
                self.pub.publish(self.vel_cmd)
                nr = nr + 1
            #self.move2()
            counter = 0
            print "a"

        elif r == len(right) and l < len(left) and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1
            self.pub.publish(self.vel_cmd)
            counter = 0
            print "b"

        elif r < len(right) and l == len(left) and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -1
            self.pub.publish(self.vel_cmd)
            counter = counter + 1
            print "c"

        elif r == 0 and l < len(left) and l != 0 and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -1
            self.pub.publish(self.vel_cmd)
            counter = 0
            print "d"

        elif r < len(right) and l == 0 and r != 0 and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1
            self.pub.publish(self.vel_cmd)
            counter  = 0
            print "e"

        elif r < 5 and l < 5 and counter < 6:
            self.vel_cmd.linear.x = 0.26
            self.vel_cmd.angular.z = 0
            #self.vel_cmd.angular.z = random.uniform(-0.5, 0.5)
            self.pub.publish(self.vel_cmd)
            counter = 0
            print "g"

        elif r < len(right) and l < len(left) and r != 0 and l != 0 and counter < 6:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1
            self.pub.publish(self.vel_cmd)
            counter = counter + 1
            print "f"

        if counter > 5:
            counter = 0
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 1
            self.pub.publish(self.vel_cmd)
            time.sleep(2)


    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping publisher node at: {}".format(rospy.get_time()))
        self.robot_controller.stop()

    def move1(self):
        time.sleep(1)
        self.robot_controller.set_move_cmd(0.26, 0)
        self.robot_controller.publish()
        time.sleep(4.7)
        self.robot_controller.stop()


    def move2(self):
        print "move2()"
        self.robot_controller.set_move_cmd(0, -1.82)
        self.robot_controller.publish()
        time.sleep(0.8)
        #self.robot_controller.stop()

    def move3(self):
        self.robot_controller.set_move_cmd(0.26, 0)
        self.robot_controller.publish()
        time.sleep(2.5)

    def move4(self):
        self.robot_controller.set_move_cmd(0, 1.82)
        self.robot_controller.publish()
        time.sleep(0.8)

    def move5(self):
        self.robot_controller.set_move_cmd(0.26, 0)
        self.robot_controller.publish()
        time.sleep(2)


    def main_loop(self):
        while not self.ctrl_c:
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()
        #self.move1()
        #self.move2()
        #self.move3()
        #self.move4()
        #self.move5()



        #self.vel_cmd.linear.x = 0.26 # m/s
        #self.pub.publish(self.vel_cmd)
        #time.sleep(3)

        #i = 0
        #while i < 2:

        #    self.vel_cmd.linear.x = 0.26 # m/s
        #    self.pub.publish(self.vel_cmd)
        #    time.sleep(4.7)

        #    time.sleep(2)
        #    self.vel_cmd.angular.z = -1.82 # m/s
        #    self.pub.publish(self.vel_cmd)

        #    i = i + 1


        #self.vel_cmd.linear.x = 0 # m/s
        #self.vel_cmd.angular.z = 1.82 # m/s
        #self.pub.publish(self.vel_cmd)
        print "Okay"
        #self.robot_controller.set_move_cmd(0.26, 0.0)
        #self.robot_controller.publish()
        #time.sleep(3)

        #while not self.ctrl_c:



            #self.vel_cmd.linear.x = 0 # m/s
            #self.vel_cmd.angular.z = 0 # m/s
            #self.pub.publish(self.vel_cmd)

            #self.pub.publish(self.vel_cmd)
            #self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Task4()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
