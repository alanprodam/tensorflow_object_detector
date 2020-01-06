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
        #self.sub_rcnn = rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN)
        #self.sub_aruco = rospy.Subscriber("bebop/pose_aruco", Odometry, self.callbackPoseAruco)
        
        rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN)
        # rospy.Subscriber("bebop/pose_aruco",Odometry, self.callbackPoseAruco)
        

        try: 
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shutting down")

    def callbackPoseRCNN(self, data):
        global obj, obj_hypothesis
        # recive data
        objArray = Detection2DArray()

        obj=Detection2D()
        obj_hypothesis= ObjectHypothesisWithPose()

        # rcnn_pose 
        objArray = data

        obj = objArray.detections

        rospy.loginfo(" lenth obj: %f", len(obj))

        for i in range(len(obj)):
            rospy.loginfo("************* frame %d", i)
            object_id = obj[i].header.frame_id
            rospy.loginfo(" object_id: %s", object_id)
            result_id = obj[i].results[0].id
            rospy.loginfo(" result_id: %s", result_id)
            object_score = obj[i].results[0].score
            rospy.loginfo(" result_score: %f", object_score)
            # pose
            object_pose = obj[i].results[0].pose.pose.position.x
            rospy.loginfo(" result_pose.x: %f", object_pose)
        
        if len(obj) != 0:
            obj_hypothesis.id = obj[0].results[0].id
            obj_hypothesis.score = obj[0].results[0].score
            obj_hypothesis.pose.pose.position.x = obj[0].results[0].pose.pose.position.x
            obj_hypothesis.pose.pose.position.x = obj[0].results[0].pose.pose.position.y
            obj_hypothesis.pose.pose.position.x = obj[0].results[0].pose.pose.position.z



    def callbackPoseAruco(self, data):
        # recive data
        aruco_pose = data
        rospy.loginfo(rcnn_pose.pose.pose.position.z)

        # print "received data: ", data
        #Z = np.matrix([data.x,data.y,data.z]).getT()
        Z = np.matrix([1,1,10]).getT()

        if self.kalman.first:
            self.kalman.x = Z
            self.kalman.first = False

        self.kalman.update(Z)
        self.kalman.predict()

        vec = Vector3()
        # vec.x = self.kalman.x[0]
        # vec.y = self.kalman.x[1]
        # vec.z = self.kalman.x[2]
        vec.x = 1
        vec.y = 2
        vec.z = 3

        rospy.loginfo("--------------------------------")
        rospy.loginfo("X (m): %f", vec.x)
        rospy.loginfo("Y (m): %f", vec.y)
        rospy.loginfo("Z (m): %f", vec.z)

        self.pub_hibrid.publish(vec)



if __name__ == '__main__':
    subscriber = Subscriber()
