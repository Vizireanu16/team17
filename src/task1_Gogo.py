#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Task1:


    """#robot should search environment every 0.1s
    def callback(self, data):
        print ("ranges[0] " + str(msg.ranges[0]))
        left_arc = data.ranges[0:21]
        right_arc = data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.left_dis = left_arc
        self.right_dis = right_arc
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
       
    def _init_():
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
            if self.startup:
                self.vel = Twist()
                print("init")
            elif self.turn:
                if abs(self.turn_left) >= pi/2:
                    # If the robot has turned 90 degrees (in radians) then stop turning
                    self.turn = False
                    # publish a default twist message:
                    self.vel = Twist()
                    # reset the init_yaw value for the next turn operation:
                    self.init_yaw = self.yaw
                    print("turn-fwd transition")
                else:
                    # if not, then keep on turning at 0.2 rad/s:
                    self.vel = Twist()
                    self.vel.angular.z = 0.2
                    print("turning")
            else:
                if sqrt(pow(self.init_x - self.x, 2) + pow(self.init_y - self.y, 2)) >= 0.2:
                    # if distance travelled is greater than 0.5m then stop, and start turning:
                    self.vel = Twist()
                    self.turn = True
                    # reset the init_x & y values for the next forwards operation:
                    self.init_x = self.x
                    self.init_y = self.y
                    print("fwd-turn transition")
                else:
                    # if not, then keep on moving forward at 0.1 m/s:
                    self.vel = Twist()
                    self.vel.linear.x = 0.1
                    print("moving forwards")
            # publish whatever velocity command has been set in the above:
            self.pub.publish(self.vel)
            # print a status message:
            self.print_current_velocity_command()
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Task1()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass"""

    def callback(self, msg):
        print ("ranges[0] " + str(msg.ranges[0]))
        #self.vel_cmd.linear.x = 0.1
        left_arc = msg.ranges[0:45]
        right_arc = msg.ranges[-45:]
        left = np.array(left_arc)
        right = np.array(right_arc)
        front_arc = np.array(left_arc + right_arc)

        #rotate = False
        i = 0
        l = 0
        r = 0

        for range in front_arc:
            if range < 0.4:
                i = i + 1

        for range in left:
            if range < 0.4:
                l = l + 1

        for range in right:
            if range < 0.4:
                r = r + 1


        #print front_arc
        #print ("ranges: ",front_arc)
        #print ("i: ", i)
        print ("i " + str(i))
        print ("front_arc " + str(len(front_arc)))

        print ("l " + str(l))
        print ("left " + str(len(left)))

        print ("r " + str(r))
        print ("right " + str(len(right)))

        #if i == len(front_arc):
        #    rotate = True

        if i == len(front_arc):
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0.5
            self.pub.publish(self.vel_cmd)
            print "a"

        elif r == len(right) and l < len(left):
             self.vel_cmd.linear.x = 0
             self.vel_cmd.angular.z = 0.5
             self.pub.publish(self.vel_cmd)
             print "b"

        elif r < len(right) and l == len(left):
             self.vel_cmd.linear.x = 0
             self.vel_cmd.angular.z = -0.5
             self.pub.publish(self.vel_cmd)
             print "c"

        elif r == 0 and l < len(left) and l != 0:
             self.vel_cmd.linear.x = 0
             self.vel_cmd.angular.z = -0.5
             self.pub.publish(self.vel_cmd)
             print "d"

        elif r < len(right) and l == 0 and r != 0:
             self.vel_cmd.linear.x = 0
             self.vel_cmd.angular.z = 0.5
             self.pub.publish(self.vel_cmd)
             print "e"

        elif r < len(right) and l < len(left) and r != 0 and l != 0:
             self.vel_cmd.linear.x = 0
             self.vel_cmd.angular.z = 0.5
             self.pub.publish(self.vel_cmd)
             print "f"

        elif r < 5 and l < 5:
             self.vel_cmd.linear.x = 0.1
             self.vel_cmd.angular.z = 0
             self.pub.publish(self.vel_cmd)
             print "g"
        """
        elif msg.ranges[0] < 0.3:
            self.vel_cmd.linear.x = 0
            self.pub.publish(self.vel_cmd)
        """

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
    
        

