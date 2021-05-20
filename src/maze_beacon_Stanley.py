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
        rospy.init_node('object_search')
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

        self.rate = rospy.Rate(5000)
        
        self.m00 = 0
        self.m00_min = 10000000

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

        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-96, 96)
        self.all_distance = 0.0
        self.all_angle = 0.0
        self.front_distance = 0.0
        self.front_angle = 0.0
        self.right_distance = 0.0
        self.right_angle = 0.0
        self.small_front_distance = 0
        self.small_front_angle = 0
        self.raw_data = []

        self.init_x = 0.0
        self.init_y = 0.0

        self.init_search = False
        self.start_nav = False

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def scan_callback(self, scan_data):
        close_left_arc = scan_data.ranges[0:3]
        close_right_arc = scan_data.ranges[-3:] 
        close_front_arc = np.array(close_left_arc[::-1] + close_right_arc[::-1])
        self.close_front_distance = close_front_arc.min()

        # front detection
        front_left_arc = scan_data.ranges[0:16]
        front_right_arc = scan_data.ranges[-15:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-15, 16)

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]
        
        #right detection
        right_arc = scan_data.ranges[-70:-15]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(-70,-15)

        # find the miniumum object distance within the right laserscan arc:
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]

        #left detection
        left_arc = scan_data.ranges[16:60]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(16,60)

        # find the miniumum object distance within the left laserscan arc:
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]

        left_arc40 = scan_data.ranges[0:40]
        right_arc40 = scan_data.ranges[-40:]
        self.left40 = left_arc40
        self.right40 = right_arc40
        front_arc40 = np.array(left_arc40[::-1] + right_arc40[::-1])
        self.front_distance40 = front_arc40
        self.min_distance40 = front_arc40.min()

            

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

        #if self.m00 > self.m00_min:
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
            
    def turn_180(self):
        time.sleep(1)
        self.robot_controller.set_move_cmd(0.0, 0.66)    
        self.robot_controller.publish()
        #print "checking colour"
        time.sleep(3)

    def turn_back(self):
        self.robot_controller.set_move_cmd(0.0, -0.68)    
        self.robot_controller.publish()
        #print "turning back"
        time.sleep(3)
        self.robot_controller.stop()   
        #print "stop"

    def leave_spawn(self):
        self.robot_controller.set_move_cmd(0.18, 0.0)    
        self.robot_controller.publish()
        #print "leaving spawn"
        time.sleep(1)


    def maze_nav(self, distance):
        if self.front_distance > distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0.2, -0.8)
            self.robot_controller.publish()
            #print "right1"
        elif self.front_distance < distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0, -0.5)
            self.robot_controller.publish()
            #print "right2"
        elif self.front_distance > distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0.08, 0.5)
            self.robot_controller.publish()
            #print "left1"
        elif self.front_distance < distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0, 0.5)
            self.robot_controller.publish()
            #print "left2"
        else:
            self.robot_controller.set_move_cmd(0, -0.5)
            self.robot_controller.publish()
            #print "right3"
    
    def avoid_object(self):
        while self.min_distance40 < 0.4: 
            i = 0
            l = 0
            r = 0
            for range in self.front_distance40:
                if range < 0.4:
                    i = i + 1

            for range in self.left40:
                if range < 0.4:
                    l = l + 1

            for range in self.right40:
                if range < 0.4:
                    r = r + 1 
            if i == len(self.front_distance40):
                self.robot_controller.set_move_cmd(0.0, 0.4)    
                self.robot_controller.publish()
                #print "turn left-1"
            elif r == len(self.right40) and l < len(self.left40):                   
                self.robot_controller.set_move_cmd(0.0, 0.4)    
                self.robot_controller.publish()
                #print "turn left-2"                             
            elif r < len(self.right40) and l == len(self.left40):
                self.robot_controller.set_move_cmd(0.0, -0.4)    
                self.robot_controller.publish()
                #print "turn right-1"
            elif r == 0 and l < len(self.left40) and l != 0:
                self.robot_controller.set_move_cmd(0.0, -0.4)    
                self.robot_controller.publish()
                #print "turn right-2"
            elif r < len(self.right40) and l == 0 and r != 0:
                self.robot_controller.set_move_cmd(0.0, 0.4)    
                self.robot_controller.publish()
                #print "turn left-3"
            elif r < len(self.right40) and l < len(self.left40) and r != 0 and l != 0:
                self.robot_controller.set_move_cmd(0.0, -0.4)    
                self.robot_controller.publish()
                #print "turn right-3"            
        self.robot_controller.set_move_cmd(0.25, 0.0)    
        self.robot_controller.publish()
        #print "moving forward"
            

    def check_spawn(self):
        distance_to_spawn = math.sqrt((self.robot_odom.posx - self.init_x)**2 + (self.robot_odom.posy- self.init_y)**2)
        #print(distance_to_spawn)
        spawn = False
        if distance_to_spawn > 1.5:
            spawn = False
        else: 
            spawn = True

        return spawn

    def check_object(self):
        if self.m00 > self.m00_min and self.check_spawn() == False:           
            if self.cy >= 560-100 and self.cy <= 560+100:
                if self.move_rate == 'slow':
                    self.move_rate = 'stop'                             
            else:
                self.move_rate = 'slow'
        else:
            self.move_rate = 'fast'                
        if self.move_rate == 'fast':
            self.maze_nav(0.35)
            #print "Turn fast"
        elif self.move_rate == 'slow' and self.check_spawn() == False:
            if 0 < self.cy and self.cy <= 560-100 and self.check_spawn() == False:
                self.robot_controller.set_move_cmd(0.1, 0.25)
                self.robot_controller.publish()
                #print "Adjust left"
            elif self.cy > 560+100:
                self.robot_controller.set_move_cmd(0.1, -0.25)
                self.robot_controller.publish()
                #print "Adjust right"
        elif self.move_rate == 'stop' and self.check_spawn() == False:
            if self.close_front_distance < 0.5 and self.check_spawn() == False:
                #print "enter zone"
                self.robot_controller.set_move_cmd(0.1, 0.0)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.stop()
                print "BEACONING COMPLETE: The robot has now stopped."
                self.distance_status = True
            else:
                self.avoid_object()  
                #print "moving towards beacon"                           
        elif self.check_spawn() == False:
            self.maze_nav(0.35) 
                    
        self.robot_controller.publish()


    def main(self):
        self.turn_180()      #turn back to check target colour
        self.find_colour()   #check target colour
        self.turn_back()     #turn back to the front
        self.leave_spawn()   #move forward
        self.init_search = False
        self.distance_status = False
        if self.start_nav == False:
                self.init_x = self.robot_odom.posx
                self.init_y = self.robot_odom.posy
                self.start_nav = True
        #counter = 0
        while not self.ctrl_c:
            if self.m00 > self.m00_min and self.check_spawn() == False: 
                self.init_search = True
                if self.init_search == False:                    
                    print "BEACON DETECTED: Beaconing initiated."     
                self.check_object()
                if self.distance_status == True:
                    break
            else:
                self.maze_nav(0.35)
                 
        self.rate.sleep()

            
            
            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
