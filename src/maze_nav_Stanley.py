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



class maze_nav(object):

    def __init__(self):
        rospy.init_node('object_search')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(3000)
        
        self.m00 = 0
        self.m00_min = 100000

        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-96, 96)

        self.right_status = False
        self.left_status = False
        self.front_status = False
        self.T_turn = 0

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

        m = cv2.moments(self.mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(5)      
    

    def leave_spawn(self):
        rospy.sleep(1)
        self.robot_controller.set_move_cmd(0.2, 0.0)    
        self.robot_controller.publish()
        print "leaving spawn"
        rospy.sleep(3)

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
        if self.left_arc < 0.2:
            self.robot_controller.set_move_cmd(0.0, -0.3)
            self.robot_controller.publish()
            print "adjust turn right"
        elif self.right_arc < 0.2:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
            print" adjust turn left"
        else:            
            self.robot_controller.set_move_cmd(0.2, 0.0)    
            self.robot_controller.publish()  

    def main(self):
        self.leave_spawn()   #move forward
        while not self.ctrl_c:
            self.left90()
            self.right90()
            self.front()
            if self.left_status == True and self.front_status == True:
                print "1"
                self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish()
                print"stop"
                rospy.sleep(1.5) 
                self.turn_right90()
            elif self.right_status == True and self.front_status == True:
                print "2"
                self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish()
                print"stop"
                rospy.sleep(1.5) 
                self.turn_left90()
            elif self.front_status == True and self.right_status == False and self.left_status == False:
                if self.T_turn == 0:
                    print "3"
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    print"stop"
                    rospy.sleep(1.5)   
                    self.turn_left90()
                    self.T_turn += 1
                elif self.T_turn == 1:
                    print "4"
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    print"stop"
                    rospy.sleep(1.5) 
                    self.turn_right90()
                    self.T_turn += 1
                elif self.T_turn == 2:
                    print "5"
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    print"stop"
                    rospy.sleep(1.5) 
                    self.turn_left90()
            else:
                self.avoid_wall()  
                    
          
                 
        self.rate.sleep()

            
                   
if __name__ == '__main__':
    search_ob = maze_nav()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
