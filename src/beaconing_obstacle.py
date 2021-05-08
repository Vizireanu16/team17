#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import random
import time
import math
import cv2
from cv_bridge import CvBridge
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

class Task1(object):

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

        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.lowerbound = []
        self.upperbound = []

        self.color_boundaries = {
            "red":    ([-1.8, 217, 100], [3.3, 255, 255]),
            "blue":   ([115, 224, 100],   [130, 255, 255]),
            "yellow": ([28, 180, 100], [32, 255, 255]),
            "green":   ([58, 50, 100], [61, 256, 255]),
            "Turquoise":   ([75, 50, 100], [91, 254, 255]),
            "purple":   ([145, 185, 100], [150, 250, 255])
        }

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
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping publisher node at: {}".format(rospy.get_time()))
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0
        self.pub.publish(self.vel_cmd)

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img
        
        if len(self.lowerbound) != 0 and len(self.upperbound) != 0:
            self.mask = cv2.inRange(hsv_img, self.lowerbound, self.upperbound)

        m = cv2.moments(self.mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            cv2.imshow('cropped image', crop_img)
            cv2.waitKey(5)
        
    def find_colour(self):
        for color_name, (lower, upper) in self.color_boundaries.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                print("SEARCH INITIATED: The target colour is {}".format (color_name))
                self.lowerbound = lower_bound
                self.upperbound = upper_bound

    def turn_180(self):
        time.sleep(1)
        self.robot_controller.set_move_cmd(0.0, 0.33)    
        self.robot_controller.publish()
        print "turn left"
        time.sleep(6)

    def turn_back(self):
        self.robot_controller.set_move_cmd(0.0, -0.32)    
        self.robot_controller.publish()
        print "turn right"
        time.sleep(6)
        self.robot_controller.stop()   
        print "stop"


    def find_pillar(self):
        self.robot_controller.set_move_cmd(0.35, 0.0)    
        self.robot_controller.publish()
        print "move forward"
        time.sleep(3)
        self.robot_controller.set_move_cmd(0.0, 0.4)    
        self.robot_controller.publish()
        print "turn left"
        time.sleep(7)
        self.robot_controller.stop()    
        print "stop"


    def main(self):
        self.turn_180()      #turn back to check target colour
        self.find_colour()   #check target colour
        self.turn_back()     #turn back to the front
        self.find_pillar()   #move forward
        while not self.ctrl_c:
            if self.m00 > self.m00_min:
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'                
            if self.move_rate == 'fast':
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop':
                self.robot_controller.stop()
                print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                break
            else:
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

            self.pub.publish(self.vel_cmd)
            self.robot_controller.publish()
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Task1()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
