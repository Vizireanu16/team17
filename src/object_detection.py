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

        self.status = 0

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.lower_bound = []
        self.upper_bound = []

        self.color_boundaries = {
            "red":    ([0, 200, 100], [8, 255, 255]),
            "blue":   ([115, 224, 100],   [130, 255, 255]),
            "yellow": ([28, 180, 100], [32, 255, 255]),
            "green":   ([40, 50, 100], [65, 255, 255]),
            "lightblue":   ([75, 50, 100], [90, 255, 255]),
            "purple":   ([145, 185, 100], [150, 250, 255])
        }

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
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
        
        self.mask = cv2.inRange(hsv_img, np.array(self.lower_bound[0]), np.array(self.upper_bound[0]))

        m = cv2.moments(self.mask)
        self.m00 = m_db['m00']
        self.cy = m_db['m10'] / (m_db['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            cv2.imshow('cropped image', crop_img)
            cv2.waitKey(5)


        
        
    def find_colour(self):
        for color_name, (lower, upper) in self.color_boundaries.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound)
            if mask.any():
                print("SEARCH INITIATED: The target colour is {}".format (color_name))
                self.lower_bound = lower_bound
                self.upper_bound = upper_bound

            
    def turn_180(self):
        time.sleep(1)
        self.robot_controller.set_move_cmd(0.0, 0.55)    
        self.robot_controller.publish()
        print "turn left"
        time.sleep(3)

    def turn_back(self):
        self.robot_controller.set_move_cmd(0.0, -0.56)    
        self.robot_controller.publish()
        print "turn right"
        time.sleep(3)
        self.robot_controller.set_move_cmd(0.0, 0.0)    
        self.robot_controller.publish()
        print "stop"


    def find_pillar(self):
        self.robot_controller.set_move_cmd(0.33, 0.0)    
        self.robot_controller.publish()
        print "move forward"
        time.sleep(3)
        self.robot_controller.set_move_cmd(0.0, 0.4)    
        self.robot_controller.publish()
        print "turn left"
        time.sleep(7)
        self.robot_controller.set_move_cmd(0.0, 0.0)    
        self.robot_controller.publish()
        print "stop"


    def main(self):
        while not self.ctrl_c:

            if self.status == 0:
                self.turn_180()
                self.status+=1
            elif self.status == 1:
                self.find_colour()
                self.status+=1
            elif self.status == 2:
                self.turn_back()
                self.status+=1
            elif self.status == 3:
                self.find_pillar()
                self.status+=1
            elif self.status == 4:
                if self.m00 > self.m00_min:
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                            self.stop_counter = 30
                        else:
                            self.move_rate = 'slow'
                    else:
                        self.move_rate = 'fast'                
                if self.move_rate == 'fast':
                    print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop' and self.stop_counter > 0:
                    print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels... Counting down: {}".format(self.cy, self.stop_counter))
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                else:
                    print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
                self.robot_controller.publish()
                self.rate.sleep()

            
            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
