#!/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class FollowWall(object):
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """
    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('follow_wall')
        init_speed = 0
        
        self.twist = Twist()
        # self.target_distance = rospy.get_param('~target_distance')
        self.target_distance = .5
        self.actual_distance = 0

        self.angle45 = 0
        self.angle135 = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)
        
    def callbackScan(self, data):
        self.angle45 = data.ranges[45]
        self.angle135 = data.ranges[135]
        print "45 ", self.angle45
        print "135 ", self.angle135
        self.calculateAngularVel()

        
    def calculateAngularVel(self):
        self.diff = self.angle45 - self.angle135
        if self.angle45 == 0 and self.angle135 == 0:
            self.twist.angular.z = .1
            self.twist.linear.x = 0
        elif self.angle135 == 0:
            self.twist.angular.z = -.2*self.diff 
            self.twist.linear.x = 0  
        elif self.angle45 == 0:
            self.twist.angular.z = -.2*self.diff 
            self.twist.linear.x = 0 
        else:
            self.twist.angular.z = .2* self.diff
            self.twist.linear.x = 0

            print "diff", math.fabs(self.diff)
            if math.fabs(self.diff) < .1 and math.fabs(self.diff) > 0.0:
                self.twist.linear.x = .1
                self.twist.angular.z = 0
            


    def run(self):
        """ Our main 5Hz run loop """

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print "twist z ", self.twist.angular.z ,"twist x ", self.twist.linear.x
            self.pub.publish(self.twist)
            r.sleep()

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

if __name__ == '__main__':
    node = FollowWall()

    node.run()