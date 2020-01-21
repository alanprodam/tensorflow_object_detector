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

# ROS related imports of Odometry ans Navigation
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3

# ROS related imports of Vision
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

        self.I = np.matrix(np.identity(n_states))                   #identity(3,3)
        self.x = np.matrix(np.zeros(shape=(n_states,1)))            #zeros(3,1)
        self.P = np.matrix(np.identity(n_states))                   #identity(3,3)                  
        self.F = np.matrix(np.identity(n_states))                   #identity(3,3)  
        self.u = np.matrix(np.zeros(shape=(n_states,1)))            #zeros(3,1)
        self.H = np.concatenate((self.I,self.I), axis=0)            #identity(3,3)
        self.R = np.matrix(np.identity(n_sensors))                  #identity(6,6)
        self.Q = np.matrix(np.identity(n_states))                   #identity(3,3) 

        self.first = True

    def update(self, Z):

        w = Z - self.H * self.x
        S = self.H * self.P * self.H.getT() + self.R
        K = self.P * self.H.getT() * S.getI()
        self.x = self.x + K * w
        self.P = (self.I - K * self.H) * self.P

    def predict(self):
        self.x = self.F * self.x # + self.u
        self.P = self.F * self.P * self.F.getT() + self.Q

class Subscriber(object):
    
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('filter_node', anonymous=True, log_level=rospy.DEBUG)

        self.kalman = Kalman(n_states = 3, n_sensors = 6)
        self.kalman.P *= 10
        self.kalman.R *= 0.02

        self.VecNeural = Vector3()
        self.VecAruco = Vector3()

        self.pub_hibrid = rospy.Publisher('kalman/hibrid', Vector3)
        rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN)
        rospy.Subscriber("aruco_double/pose",Pose, self.callbackPoseAruco)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            Zneural = [self.VecNeural.x, self.VecNeural.y, self.VecNeural.z]
            Zaruco = [self.VecAruco.x, self.VecAruco.y, self.VecAruco.z]

            Z = np.matrix(np.concatenate((Zneural, Zaruco), axis=None)).getT()

            if self.kalman.first:
                # insert the first 3 values of the vector 
                self.kalman.x = Z[0:3, :]
                self.kalman.first = False

            if Z[2] != 0 and Z[5] != 0: 
                # greater neural error and lower aruco error at low height
                covNeural = (3.5/(abs(self.kalman.x[2])+0.1))+1.5 #np.exp(abs(self.kalman.x[2])*0.5-1)
                covAruco = 0.005*abs(self.kalman.x[2])+0.3
            elif Z[2] == 0:
                covNeural = 15
                covAruco = 0.01*abs(self.kalman.x[2])+1
            elif Z[5] == 0:
                covNeural = (1.5/(abs(self.kalman.x[2])+1))+1
                covAruco = 15

            # set values of cov in R matrix
            arrayNeral = np.full((1, 3), covNeural, dtype=float)
            arrayAruco = np.full((1, 3), covAruco, dtype=float)
            Zarray = np.concatenate((arrayNeral, arrayAruco), axis=None)
            self.kalman.R = np.diag(Zarray)

            rospy.logdebug("------------------------")
            rospy.logdebug("arrayNeral : %f", covNeural)
            rospy.logdebug("arrayAruco : %f", covAruco)

            self.kalman.predict()
            self.kalman.update(Z)
            
            vec = Vector3()
            vec.x = self.kalman.x[0]
            vec.y = self.kalman.x[1]
            vec.z = self.kalman.x[2]

            rospy.logdebug("------------------------")
            rospy.logdebug("kalman.sensor[1].x : %f", vec.x)
            rospy.logdebug("kalman.sensor[1].y : %f", vec.y)
            rospy.logdebug("kalman.sensor[1].z : %f", vec.z)

            self.pub_hibrid.publish(vec)
            r.sleep()

        try: 
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shutting down")

    def callbackPoseRCNN(self, data):
        # recive data
        objArray = Detection2DArray()

        # rcnn_pose
        objArray = data
        # rospy.logdebug(" lenth objArray.detections: %f", len(objArray.detections))
        
        if len(objArray.detections) != 0:
            self.VecNeural.x = self.kalman.x[0] = objArray.detections[0].results[0].pose.pose.position.x
            self.VecNeural.y = self.kalman.x[1] = objArray.detections[0].results[0].pose.pose.position.y
            self.VecNeural.z = self.kalman.x[2] = objArray.detections[0].results[0].pose.pose.position.z
            # rospy.logdebug("--------------------------------")
            # rospy.logdebug("rcnn_pose.x (m): %f", self.VecNeural.x)
            # rospy.logdebug("rcnn_pose.y (m): %f", self.VecNeural.y)
            # rospy.logdebug("rcnn_pose.z (m): %f", self.VecNeural.z)


    def callbackPoseAruco(self, data):
        # recive data
        #aruco_pose = data

        # print "received data: ", data
        self.VecAruco = Vector3()
        self.VecAruco = Vector3(data.position.x, data.position.y, data.position.z)
        # rospy.logdebug("--------------------------------")
        # rospy.logdebug("aruco_pose.x (m): %f", self.VecAruco.x)
        # rospy.logdebug("aruco_pose.y (m): %f", self.VecAruco.y)
        # rospy.logdebug("aruco_pose.z (m): %f", self.VecAruco.z)


if __name__ == '__main__':
    subscriber = Subscriber()
