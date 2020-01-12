#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

class Kalman(object):
    """docstring for Kalman"""
    def __init__(self, n_states, n_sensors):
        super(Kalman, self).__init__()
        self.n_states = n_states
        self.n_sensors = n_sensors

        self.x = np.matrix(np.zeros(shape=(n_states,1)))
        self.P = np.matrix(np.identity(n_states)) 
        self.F = np.matrix(np.identity(n_states))
        self.u = np.matrix(np.zeros(shape=(n_states,1)))
        self.H = np.matrix(np.zeros(shape=(n_sensors, n_states)))
        self.R = np.matrix(np.identity(n_sensors))
        self.I = np.matrix(np.identity(n_states))

        self.first = True

    def update(self, Z):
        '''Z: new sensor values as numpy matrix'''

        w = Z - self.H * self.x
        #S = self.H * self.P * self.H.getT() + self.R
        #K = self.P * self.H.getT() * S.getI()
        #self.x = self.x + K * w
        #self.P = (self.I - K * self.H) * self.P

    def predict(self):
        self.x = self.F * self.x + self.u
        self.P = self.F * self.P * self.F.getT()

class Subscriber(object):
    
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('filter_node', anonymous=True, log_level=rospy.DEBUG)

        self.kalman = Kalman(n_states = 6, n_sensors = 2)
        self.kalman.H = np.matrix(np.identity(self.kalman.n_states))
        self.kalman.P *= 10
        self.kalman.R *= 0.02

        self.pub_hibrid = rospy.Publisher('kalman/hibrid', Vector3)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            Zneural = np.matrix([33,5,10,22,3,8]).getT()
            Zaruco = np.matrix([22,3,8]).getT()

            self.hybridFilter(Zneural,Zaruco)
            r.sleep()
        
        try: 
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shutting down")

    def hybridFilter(self, d1, d2):

        Z1 = d1
        Z2 = d2
        
        # Z = np.matrix(np.zeros(shape=(6, 1)))
        # Z[0] = Zneural[0]
        # Z[1] = Zneural[1]
        # Z[2] = Zneural[2]
        # Z[3] = Zaruco[0]
        # Z[4] = Zaruco[1]
        # Z[5] = Zaruco[2]

        if self.kalman.first:
            self.kalman.x = Z1
            self.kalman.first = False

        self.kalman.update(Z1)
        #self.kalman.predict()

        vec = Vector3()
        vec.x = self.kalman.x[0]
        vec.y = self.kalman.x[1]
        vec.z = self.kalman.x[2]

        rospy.loginfo("kalman.sensor[1].x : %f", vec.x)
        rospy.loginfo("kalman.sensor[1].y : %f", vec.y)
        rospy.loginfo("kalman.sensor[1].z : %f", vec.z)
        rospy.loginfo("------------------------")


        self.kalman.update(Z2)
        #self.kalman.predict()

        vec.x = self.kalman.x[0]
        vec.y = self.kalman.x[1]
        vec.z = self.kalman.x[2]

        rospy.loginfo("kalman.sensor[2].z : %f", vec.x)
        rospy.loginfo("kalman.sensor[2].y : %f", vec.y)
        rospy.loginfo("kalman.sensor[2].z : %f", vec.z)
        rospy.loginfo("------------------------")


        self.pub_hibrid.publish(vec)

if __name__ == '__main__':
    subscriber = Subscriber()
