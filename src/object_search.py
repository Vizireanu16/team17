#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import time
import math

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import numpy as np



class colour_search(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

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

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-45, 45)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:45]
        right_arc = scan_data.ranges[-45:]    
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.left = left_arc
        self.right = right_arc
        self.front_distance = front_arc
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]

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


    def leave_spawn(self):
        self.robot_controller.set_move_cmd(0.2, 0.0)    
        self.robot_controller.publish()
        print "leaving spawn"
        time.sleep(2)       


    def main(self):
        self.turn_180()      #turn back to check target colour
        self.find_colour()   #check target colour
        self.turn_back()     #turn back to the front
        self.leave_spawn()   #move forward
        while not self.ctrl_c:
            while self.min_distance > 0.3:
                while self.min_distance < 0.5:
                    i = 0
                    l = 0
                    r = 0

                    for range in self.front_distance:
                        if range < 0.4:
                            i = i + 1

                    for range in self.left:
                        if range < 0.4:
                            l = l + 1

                    for range in self.right:
                        if range < 0.4:
                            r = r + 1    

                    if i == len(self.front_distance):
                        self.robot_controller.set_move_cmd(0.0, 1.0)    
                        self.robot_controller.publish()
                        print "turn left"
                    elif r == len(self.right) and l < len(self.left):
                        self.robot_controller.set_move_cmd(0.0, 1.0)    
                        self.robot_controller.publish()
                        print "turn left"
                    elif r < len(self.right) and l == len(self.left):
                        self.robot_controller.set_move_cmd(0.0, -1.0)    
                        self.robot_controller.publish()
                        print "turn right"
                    elif r == 0 and l < len(self.left) and l != 0:
                        self.robot_controller.set_move_cmd(0.0, -1.0)    
                        self.robot_controller.publish()
                        print "turn right"
                    elif r < len(self.right) and l == 0 and r != 0:
                        self.robot_controller.set_move_cmd(0.0, 1.0)    
                        self.robot_controller.publish()
                        print "turn left"
                    elif r < len(self.right) and l < len(self.left) and r != 0 and l != 0:
                        self.robot_controller.set_move_cmd(0.0, 1.0)    
                        self.robot_controller.publish()
                        print "turn left"            
                             
                self.robot_controller.set_move_cmd(0.2, 0.0)    
                self.robot_controller.publish()
                print "moving forward" 

            self.rate.sleep()

            
            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
