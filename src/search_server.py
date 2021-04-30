#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import time

# Import all the necessary ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-45, 45)
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:45]
        right_arc = scan_data.ranges[-45:]    
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.left = left_arc
        self.right = right_arc
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
    
    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid velocity.  Select a value between 0 and 0.26 m/s.")
            success = False
        if goal.approach_distance <= 0.2:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print("Request to move at {:.3f}m/s and stop {:.2f}m infront of any obstacles".format(goal.fwd_velocity, goal.approach_distance))

        # Get the current robot odometry:
        self.posx0 = self.robot_odom.posx
        self.posy0 = self.robot_odom.posy

        print("The robot will start to move now...")
                
        while self.min_distance > goal.approach_distance:      
            # check if there has been a request to cancel the action mid-way through:
            while self.min_distance < goal.approach_distance + 0.1:
                if i == len(front_arc):
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = 1.82
                    self.pub.publish(self.vel_cmd)
                    print "a"
                elif r == len(right) and l < len(left):
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = 1.82
                    self.pub.publish(self.vel_cmd)
                    print "b"
                elif r < len(right) and l == len(left):
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = -1.82
                    self.pub.publish(self.vel_cmd)
                    print "c"
                elif r == 0 and l < len(left) and l != 0:
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = -1.82
                    self.pub.publish(self.vel_cmd)
                    print "d"
                elif r < len(right) and l == 0 and r != 0:
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = 1.82
                    self.pub.publish(self.vel_cmd)
                    print "e"
                elif r < len(right) and l < len(left) and r != 0 and l != 0:
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = 1.82
                    self.pub.publish(self.vel_cmd)
                    print "f"
                elif r < 5 and l < 5:
                    self.vel_cmd.linear.x = 0.26
                    self.vel_cmd.angular.z = random.uniform(-0.5, 0.5)
                    self.pub.publish(self.vel_cmd)
                    print "g"
                                  
            self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)    
            self.robot_controller.publish()
            print("moving forward")
           
            self.distance = sqrt(pow(self.posx0 - self.robot_odom.posx, 2) + pow(self.posy0 - self.robot_odom.posy, 2))
            # populate the feedback message and publish it:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)
        
        if success:
            rospy.loginfo("approach completed sucessfully.")
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.min_distance
            self.result.closest_object_angle = self.object_angle

            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()