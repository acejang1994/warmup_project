#!/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import rospy
from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

class FollowContour(object):
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """
    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('contour_obstacle')

        self.twist = Twist()
        self.twist.linear.x = 0.5

        self.target_distance = 0.5
        self.target_angle = math.pi/3
        self.foundObstacle= False
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pubToViz = rospy.Publisher('/centroid', PointStamped, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)
        
    def callbackScan(self, data):
        self.ranges = data.ranges
        self.findContour()

    def findContour(self):

        objectdict = {}
        sumx= 0
        sumy= 0
        angle_centroid= 0
        dist_centroid = 0
        centroid = 0
        for i in range(-90, 90):
            if self.ranges[i] <= .8 and self.ranges[i]>0:
                locationX = self.ranges[i]*math.cos(i*math.pi/180.0)
                locationY = self.ranges[i]*math.sin(i*math.pi/180.0)
                objectdict[i] = [locationX, locationY]
                sumx += locationX
                sumy += locationY

        if len(objectdict) > 5:
            centroid = [-sumx/len(objectdict), -sumy/len(objectdict)]
            dist_centroid = math.sqrt((centroid[0]**2+centroid[1]**2))
            angle_centroid = math.atan(centroid[1]/centroid[0])
            self.point = PointStamped(point=Point(x=centroid[0], y=centroid[1]), header=Header(stamp=rospy.Time.now(), frame_id='base_laser_link'))

        #checking if it found an obstacle and make it go forward as a default.
        self.found_obstacle(dist_centroid, angle_centroid)
        self.twist.angular.z = 0   
        self.twist.linear.x = math.fabs(dist_centroid - self.target_distance)
        
        if self.foundObstacle:
            self.twist.angular.z = (angle_centroid - self.target_angle)
        else:
        #if it lost a signal it will go left.
            self.twist.linear.x = 0
            self.twist.angular.z = .2
       
    def found_obstacle(self, dist_centroid, angle_centroid):

        if dist_centroid < self.target_distance and dist_centroid != 0:
            self.foundObstacle=True
        else:
            self.foundObstacle =False
       
    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():

            if self.point:
                self.pubToViz.publish(self.point)
            self.pub.publish(self.twist)

            r.sleep()

if __name__ == '__main__':
    node = FollowContour()
    node.run()