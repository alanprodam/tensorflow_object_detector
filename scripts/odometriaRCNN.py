#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import numpy as np
try:
    import tensorflow as tsf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from geometry_msgs.msg import Twist, Vector3
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Estimator
class Estimator(object):

    def __init__(self):
        super(Estimator, self).__init__()
        rospy.init_node('estimator_node', anonymous=True, log_level=rospy.DEBUG)

        self.list_z = []

        # Publishers
        self.rcnn_pub = rospy.Publisher('rcnn/nav_position', Vector3, queue_size=100)

        # Subscribers
        self.object_sub = rospy.Subscriber("rcnn/objects", Detection2DArray, self.objCallback, queue_size=100)

        #r = rospy.Rate(1.0)
        #r.sleep()

        try: 
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shutting down")

    def objCallback(self, data):

        # recive data
        objArray = Detection2DArray()
        objArray = data
        obj = objArray.detections

        # Object search
        if len(obj) >= 1:
            for i in range(len(obj)):
                self.list_z.append(objArray.detections[i].results[0].pose.pose.position.z)
                rospy.logdebug("position.z [%d]: %f", i, objArray.detections[i].results[0].pose.pose.position.z)


        # rospy.logdebug("Tamanho da lista(out): %f", len(self.list_z))
        # rospy.logdebug("Somatoria lista(out): %f", self.sum_z)
        # rospy.logdebug("Altura Filtrada (out): %f", self.med_z)
        rospy.logdebug("--------------------------------")

        msg_navigation = Vector3()
        msg_navigation.x = 0
        msg_navigation.y = 0
        msg_navigation.z = 0
 
        self.rcnn_pub.publish(msg_navigation)

if __name__=='__main__':
    estimator = Estimator()