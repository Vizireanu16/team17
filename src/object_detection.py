#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

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
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

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

        darkblue_lower = (115, 224, 100)
        darkblue_upper = (130, 255, 255)
        
        lightblue_lower = (87, 130, 100)
        lightblue_upper = (91, 254, 255)

        green_lower = (58, 185, 100)
        green_upper = (61, 256, 255)

        red_lower = (-1.8, 217, 100)
        red_upper = (3.3, 255, 255)
        
        """Dark blue"""
        mask_db = cv2.inRange(hsv_img, darkblue_lower, darkblue_upper)
        res_db = cv2.bitwise_and(crop_img, crop_img, mask = mask_db)
        m_db = cv2.moments(mask_db)
        self.m00 = m_db['m00']
        self.cy = m_db['m10'] / (m_db['m00'] + 1e-5)
        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            cv2.imshow('cropped image', crop_img)
            cv2.waitKey(5)
            print ("Dark blue")
        """light blue"""
        mask_lb = cv2.inRange(hsv_img, lightblue_lower, lightblue_upper)
        res_lb = cv2.bitwise_and(crop_img, crop_img, mask = mask_lb)
        m_lb = cv2.moments(mask_lb)
        self.m00 = m_lb['m00']
        self.cy = m_lb['m10'] / (m_lb['m00'] + 1e-5)
        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            cv2.imshow('cropped image', crop_img)
            cv2.waitKey(5)
            print ("light blue")
        """green"""
        mask_g = cv2.inRange(hsv_img, green_lower, green_upper)
        res_g = cv2.bitwise_and(crop_img, crop_img, mask = mask_g)
        m_g = cv2.moments(mask_g)
        self.m00 = m_g['m00']
        self.cy = m_g['m10'] / (m_g['m00'] + 1e-5)
        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            cv2.imshow('cropped image', crop_img)
            cv2.waitKey(5)
            print ("green")
        """red"""
        mask_r = cv2.inRange(hsv_img, red_lower, red_upper)
        res_r = cv2.bitwise_and(crop_img, crop_img, mask = mask_r)
        m_r = cv2.moments(mask_r)
        self.m00 = m_r['m00']
        self.cy = m_r['m10'] / (m_r['m00'] + 1e-5)
        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            cv2.imshow('cropped image', crop_img)
            cv2.waitKey(5)
            print ("red")
            

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
                #print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                #print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 0:
                #print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels... Counting down: {}".format(self.cy, self.stop_counter))
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                #print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
