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
        S = self.H * self.P * self.H.getT() + self.R
        K = self.P * self.H.getT() * S.getI()
        self.x = self.x + K * w
        self.P = (self.I - K * self.H) * self.P

    def predict(self):
        self.x = self.F * self.x + self.u
        self.P = self.F * self.P * self.F.getT()

class Subscriber(object):
    """docstring for Subscriber"""
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('filter_node', anonymous=True, log_level=rospy.DEBUG)

        self.kalman = Kalman(n_states = 3, n_sensors = 3)
        self.kalman.H = np.matrix(np.identity(self.kalman.n_states))
        self.kalman.P *= 10
        self.kalman.R *= 0.01

        self.pub_hibrid = rospy.Publisher('kalman/hibrid', Vector3)
        #self.sub_rcnn = rospy.Subscriber('objects', Detection2DArray)
        #self.sub_aruco = rospy.Subscriber('bebop/pose_aruco', Odometry)

        # create the important subscribers
        self.sub_rcnn = rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN)
        self.sub_aruco = rospy.Subscriber("bebop/pose_aruco", Odometry, self.callbackPoseAruco)
        #rospy.Subscriber("bebop/pose_aruco",Odometry, self.callbackPoseAruco)
        #rospy.Subscriber('accelerometer', Vector3, self.callback_accel)
        rospy.spin()

    def callbackPoseRCNN(self, data):
        global rcnn_pose
        # recive data
        objArray = Detection2DArray()
        rcnn_pose = data

    def callbackPoseAruco(self, data):
        # recive data
        aruco_pose = data
        # print "received data: ", data
        #Z = np.matrix([data.x,data.y,data.z]).getT()
        Z = np.matrix([1,1,10]).getT()

        if self.kalman.first:
            self.kalman.x = Z
            self.kalman.first = False

        self.kalman.update(Z)
        self.kalman.predict()

        vec = Vector3()
        vec.x = self.kalman.x[0]
        vec.y = self.kalman.x[1]
        vec.z = self.kalman.x[2]

        self.pub_hibrid.publish(vec)



if __name__ == '__main__':
    subscriber = Subscriber()
