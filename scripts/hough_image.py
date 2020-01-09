#!/usr/bin/env python
import os
import sys, time, math
import rospy
import cv2
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print(" source ~/tensorflow/bin/activate")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

from geometry_msgs.msg import Twist, PointStamped

out1 = 0
out2 = 0
msg_navigation = PointStamped()

class hough_lines:
 
  def __init__(self):
    self.img_lines_pub = rospy.Publisher("image_lines",Image, queue_size=100) 
    self.img_edges_pub = rospy.Publisher("image_edges",Image, queue_size=100)
    self.nav_hough_lines_pub = rospy.Publisher("bebop/nav_hough_lines",Twist, queue_size = 100)

    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop/image_raw",Image,self.callback, queue_size=1, buff_size=2**24)
    self.navigation_sub = rospy.Subscriber('navigation', PointStamped, self.callback_navigation, queue_size=100)
    #self.curves_sub = rospy.Subscriber('navigation_curves', ByteMultiArray, self.callback_curves, queue_size=10)

  def callback_navigation(self,data):
    global out1, out2, msg_navigation

    msg_navigation = PointStamped()
    msg_navigation = data
    out1 = msg_navigation.point.x
    out2 = msg_navigation.point.y

    # rospy.loginfo("msg_navigation x: %f",out1)
    # rospy.loginfo("msg_navigation y: %f",out2)

###############################################################################
   
  def callback(self,data):
    global out1, out2,msg_navigation

    numLines=3
    yaw = 0
    x = 0
    med_theta = 0
    lines_vector = [0, 0, 0] 

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #-- Convert in gray scale\n",
    resize = cv2.resize(src_image, (224, 224), interpolation=cv2.INTER_CUBIC)

    gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    edges = cv2.Canny(gray, 350, 400, apertureSize=3, L2gradient=True) #Deteccao de bordas ... min: 350 / max: 400 

    lines = cv2.HoughLines(edges, numLines, np.pi/90, 100)

    if lines is not None: 
            if lines.shape[0] >= numLines:
                x = 0
                med_theta = 0
                for i in range(0,numLines):
                    for rho, theta in lines[i]:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
   
                        cv2.line(resize, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.line(edges, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        
                        med_theta = med_theta + (theta/numLines)
                        lines_vector[i] = theta
                        x = x+x1+x2

    mediana = int(x/(numLines*2))

    med_theta = math.degrees(med_theta)

    # zerar erro de leitura Yaw
    if abs( math.degrees(lines_vector[0]) - math.degrees(lines_vector[1]) ) < 60 and abs( math.degrees(lines_vector[0]) - math.degrees(lines_vector[2]) ) < 60 and abs( math.degrees(lines_vector[1]) - math.degrees(lines_vector[2]) ) < 60:
      if med_theta > (90):
        yaw = (180-med_theta)
      else:
        yaw = -med_theta

    # rospy.loginfo("linha 1: %f",math.degrees(lines_vector[0]))
    # rospy.loginfo("linha 2: %f",math.degrees(lines_vector[1]))
    # rospy.loginfo("linha 3: %f",math.degrees(lines_vector[2]))

    # rospy.loginfo("Media Theta: %f",med_theta)
    # rospy.loginfo("Valor x: %f",x)
    # rospy.loginfo("-------------------------")

    ganho_pid = 1000
    # y in the drone of ROS = X in the image
    y_correction = float(mediana - gray.shape[1]/2)/ganho_pid

    rospy.loginfo("half_img: %f",gray.shape[1]/2)
    rospy.loginfo("mediana: %f",mediana)
    rospy.loginfo("y_correction: %f",y_correction)

    rospy.loginfo("yaw(depois): %f",yaw)
    rospy.loginfo("-------------------------")

    # rospy.loginfo("linha 2: %f",math.degrees(lines_vector[1]))
    # rospy.loginfo("linha 3: %f",math.degrees(lines_vector[2]))

    # rospy.loginfo("-------------------------")
    # if out1 == 0:
    #     rospy.loginfo("Reta")
    # else:
    #     rospy.loginfo("Curva")
    #     if out2 == 1:
    #         rospy.loginfo("...Esquerda")
    #     else:
    #         rospy.loginfo("...Direita")
    # rospy.loginfo("-------------------------")

    nav_drone = Twist()

    if lines is not None:
      rospy.loginfo("Navigation!")
      if out1 == 0:
        rospy.loginfo("Reta")
        nav_drone.linear.x = 0.03
        nav_drone.linear.y = y_correction
        nav_drone.linear.z = 0

        nav_drone.angular.x = 0
        nav_drone.angular.y = 0
        nav_drone.angular.z = yaw*(np.pi/180)

      else:
        rospy.loginfo("Curva")
        if out2 == 1:
            nav_drone.linear.x = 0
            nav_drone.linear.y = 0
            nav_drone.linear.z = 0

            nav_drone.angular.x = 0
            nav_drone.angular.y = 0
            nav_drone.angular.z = 2*(np.pi/180)
            rospy.loginfo("...Esquerda: %f deg/s",nav_drone.angular.z*(180/np.pi))

        else:
            nav_drone.linear.x = 0
            nav_drone.linear.y = 0
            nav_drone.linear.z = 0

            nav_drone.angular.x = 0
            nav_drone.angular.y = 0
            nav_drone.angular.z = -2*(np.pi/180)
            rospy.loginfo("...Direita: %f deg/s",nav_drone.angular.z*(180/np.pi))

    else:
      rospy.loginfo("Parado!")
      nav_drone.linear.x = 0
      nav_drone.linear.y = 0
      nav_drone.linear.z = 0

      nav_drone.angular.x = 0
      nav_drone.angular.y = 0
      nav_drone.angular.z = 0
      
    rospy.loginfo("-------------------------")

    try:
      self.nav_hough_lines_pub.publish(nav_drone)
    except:
      rospy.loginfo('No publish!')

    cv2.imshow("Image-lines",resize)
    cv2.imshow("Image-edges",edges)
    cv2.waitKey(1)


    try:
      self.img_lines_pub.publish(self.bridge.cv2_to_imgmsg(resize, "bgr8"))
      self.img_edges_pub.publish(self.bridge.cv2_to_imgmsg(edges, "mono8"))
    except CvBridgeError as e:
      print(e)


###############################################################################

def main(args):

  ic = hough_lines()
  #-- Name of node
  rospy.init_node('hough', log_level=rospy.DEBUG)

  r = rospy.Rate(1.0)
  
  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)