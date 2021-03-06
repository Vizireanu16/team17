#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import time
import math
import random

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

    global counter
    counter = 0

    def __init__(self):
        rospy.init_node('object_search')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5000)

        self.m00 = 0
        self.m00_min = 100000

        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.lowerbound = []
        self.upperbound = []

        self.color_boundaries = {
            "red":    ([-1.8, 217, 100], [3.3, 255, 255]),
            "blue":   ([115, 224, 100],   [130, 255, 255]),
            "yellow": ([28, 180, 100], [32, 255, 255]),
            "green":   ([58, 50, 100], [61, 256, 255]),
            "Turquoise":   ([75, 150, 100], [100, 255, 255]),
            "purple":   ([145, 150, 100], [150, 250, 255])
        }

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.arc_angles = np.arange(-40, 40)
        self.init_x = 0
        self.init_y = 0
        self.raw_data = []

        self.right_status = False
        self.left_status = False
        self.front_status = False
        self.T_turn = 0
        self.init_search = False

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def scan_callback(self, scan_data):
        self.dead_left90 = scan_data.ranges[90]
        self.dead_right90 = scan_data.ranges[-90]
        self.dead_front = scan_data.ranges[0]
        left_arc45_55 = scan_data.ranges[30:40]
        left_arc55_60 = scan_data.ranges[40:50]
        left_arc45_60 = np.array(left_arc55_60[::-1] + left_arc45_55[::-1])
        self.left_arc = left_arc45_60.min()
        right_arc45_55 = scan_data.ranges[-40:-30]
        right_arc55_60 = scan_data.ranges[-50:-40]
        right_arc45_60 = np.array(right_arc55_60[::-1] + right_arc45_55[::-1])
        self.right_arc = right_arc45_60.min()
        close_left_arc = scan_data.ranges[0:7]
        close_right_arc = scan_data.ranges[-7:]
        close_front_arc = np.array(close_left_arc[::-1] + close_right_arc[::-1])
        self.close_front_distance = close_front_arc.min()
        self.left_arc = scan_data.ranges[0:45]
        self.right_arc = scan_data.ranges[-45:]
        front_arc = np.array(self.left_arc[::-1] + self.right_arc[::-1])
        self.min_distance = front_arc.min()

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

        cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(5)

    def find_colour(self):
        for color_name, (lower, upper) in self.color_boundaries.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                print("SEARCH INITIATED: The target beacon colour is {}".format (color_name))
                self.lowerbound = lower_bound
                self.upperbound = upper_bound

    def spawn_zone_marker(self):
        print(self.robot_odom.posx )




    def turn_180(self):
        time.sleep(1)
        self.robot_controller.set_move_cmd(0.0, 0.33)
        self.robot_controller.publish()
        #print "checking colour"
        time.sleep(6)

    def turn_back(self):
        self.robot_controller.set_move_cmd(0.0, -0.35)
        self.robot_controller.publish()
        #print "turning back"
        time.sleep(6)
        self.robot_controller.stop()
        #print "stop"

    def leave_spawn(self):
        self.robot_controller.set_move_cmd(0.15, 0.0)
        self.robot_controller.publish()
        #print "leaving spawn"
        time.sleep(3)


    def right90(self):
        if self.dead_right90 < 0.5:
            self.right_status = True
        else:
            self.right_status = False

    def left90(self):
        if self.dead_left90 < 0.5:
            self.left_status = True
        else:
            self.left_status = False

    def front(self):
        if self.dead_front < 0.445:
            self.front_status = True
        else:
            self.front_status = False

    def turn_right90(self):
        print "turn right 90"
        self.robot_controller.set_move_cmd(0.0, -0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        print "adjust right 90"
        while self.dead_left90 > 0.358:
            self.robot_controller.set_move_cmd(0.0, -0.2)
            self.robot_controller.publish()
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()
        print"stop"
        rospy.sleep(2)

    def turn_left90(self):
        print "turn left 90"
        self.robot_controller.set_move_cmd(0.0, 0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        print "adjust left 90"
        while self.dead_right90 > 0.358:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()
        print"stop"
        rospy.sleep(2)

    def avoid_wall(self):
        if self.left_arc < 0.3:
            self.robot_controller.set_move_cmd(0.0, -0.3)
            self.robot_controller.publish()
            print "adjust turn right"
        elif self.right_arc < 0.3:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
            print" adjust turn left"
        else:
            self.robot_controller.set_move_cmd(0.2, 0.0)
            self.robot_controller.publish()

    def check_object(self):
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
            #print "Turn fast"
        elif self.move_rate == 'slow':
            if 0 < self.cy and self.cy <= 560-100:
                self.robot_controller.set_move_cmd(0.1, 0.25)
                self.robot_controller.publish()
                #print "Adjust left"
            elif self.cy > 560+100:
                self.robot_controller.set_move_cmd(0.1, -0.25)
                self.robot_controller.publish()
                #print "Adjust right"
        elif self.move_rate == 'stop':
            if self.close_front_distance < 0.6 :
                self.robot_controller.set_move_cmd(0.1, 0.0)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.stop()
                print "BEACONING COMPLETE: The robot has now stopped."
                self.distance_status = True
            else:
                self.avoid_object()
                #print "moving towards beacon"
        else:
            self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

        self.robot_controller.publish()

    def avoid_object(self):

        while self.min_distance < 0.5:

            global counter

            left = np.array(self.left_arc)
            right = np.array(self.right_arc)
            front_arc = np.array(self.left_arc + self.right_arc)

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
                self.robot_controller.set_move_cmd(0.0, 1.82)
                self.robot_controller.publish()
                counter = 0
                #print "a"

            elif r == len(right) and l < len(left) and counter < 6:
                self.robot_controller.set_move_cmd(0.0, 1.82)
                self.robot_controller.publish()
                counter = 0
                #print "b"

            elif r < len(right) and l == len(left) and counter < 6:
                self.robot_controller.set_move_cmd(0.0, -1.82)
                self.robot_controller.publish()
                counter = counter + 1
                #print "c"

            elif r == 0 and l < len(left) and l != 0 and counter < 6:
                self.robot_controller.set_move_cmd(0.0, -1.82)
                self.robot_controller.publish()
                counter = 0
                #print "d"

            elif r < len(right) and l == 0 and r != 0 and counter < 6:
                self.robot_controller.set_move_cmd(0.0, 1.82)
                self.robot_controller.publish()
                counter  = 0
                #print "e"

            elif r < 5 and l < 5 and counter < 6:
                self.robot_controller.set_move_cmd(0.26, random.uniform(-0.5, 0.5))
                self.robot_controller.publish()
                counter = 0
                #print "g"

            elif r < len(right) and l < len(left) and r != 0 and l != 0 and counter < 6:
                self.robot_controller.set_move_cmd(0.0, 1.82)
                self.robot_controller.publish()
                counter = counter + 1
                #print "f"

            if counter > 5:
                counter = 0
                self.robot_controller.set_move_cmd(0.0, 1.82)
                self.robot_controller.publish()
                time.sleep(2)

        self.robot_controller.set_move_cmd(0.25, 0.0)
        self.robot_controller.publish()




        #print "moving forward"

    def main(self):
        self.turn_180()      #turn back to check target colour
        self.find_colour()   #check target colour
        self.turn_back()     #turn back to the front
        self.leave_spawn()   #move forward
        self.init_search = False
        self.distance_status = False
        counter = 0
        while not self.ctrl_c:
            self.spawn_zone_marker()
            self.check_object()
            self.avoid_object()

        self.rate.sleep()


if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
