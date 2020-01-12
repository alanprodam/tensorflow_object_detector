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

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PointStamped, Quaternion, Twist, Vector3

PARAMETERS_PATH = os.path.join(os.path.dirname(sys.path[0]),'data','Parameters')

z_position = 0

moviment = 0
rotation = 0

list_moviment = []
list_rotation = []

class lines:
 
  def __init__(self):
    self.navigation_pub = rospy.Publisher('navigation', Twist, queue_size=100)

    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop/image_raw",Image,self.callback, queue_size=100)
    # /bebop/odom
    self.odm_sub = rospy.Subscriber('/bebop/odom', Odometry, self.callback_pose, queue_size=100)

  def callback_pose(self,data):
    global z_position

    msg_odom = Odometry()
    msg_odom = data
    z_position = msg_odom.pose.pose.position.z


###############################################################################
   
  def callback(self,data):
    global moviment, rotation, list_rotation, list_moviment

    yaw = 0
    x = 0
    med_theta = 0
    lines_vector = [0, 0, 0] 

    parametersLeftRigth = np.load(PARAMETERS_PATH+'/parameters_dir_esq_completo.npy', allow_pickle=True).item()
    parametersCurvenonCurve = np.load(PARAMETERS_PATH+'/parameters_reta_curva_completo.npy', allow_pickle=True).item()

    num_px = 224
    dim = (224, 224)
    numLines=3

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    img_array = np.array(src_image)
    img_resize = cv2.resize(img_array, (224, 224), interpolation=cv2.INTER_CUBIC)
    img_gray = cv2.cvtColor(img_resize, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
    img_edges = cv2.Canny(img_gray, 300, 350, apertureSize=3, L2gradient=True) #Deteccao de bordas ... min: 350 / max: 400 
    img_reshape = cv2.resize(img_edges, dim, interpolation = cv2.INTER_CUBIC).reshape((num_px*num_px*1,1))
    #img_reshape = skimage.transform.resize(image, output_shape=(num_px,num_px)).reshape((num_px*num_px*1,1))

    out1 = int(predict_curve(img_reshape,parametersCurvenonCurve))
    out2 = int(predict_curve(img_reshape,parametersLeftRigth))

    #-- Convert in gray scale\n",
    resize = cv2.resize(src_image, (224, 224), interpolation=cv2.INTER_CUBIC)

    gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    edges = cv2.Canny(gray, 300, 350, apertureSize=3, L2gradient=True) #Deteccao de bordas ... min: 350 / max: 400 

    lines = cv2.HoughLines(img_edges, numLines, np.pi/90, 100)

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

    # rospy.loginfo("half_img: %f",gray.shape[1]/2)
    # rospy.loginfo("mediana: %f",mediana)
    # rospy.loginfo("y_correction: %f",y_correction)

    # rospy.loginfo("yaw(depois): %f",yaw)
    # rospy.loginfo("-------------------------")

    # rospy.loginfo("linha 2: %f",math.degrees(lines_vector[1]))
    # rospy.loginfo("linha 3: %f",math.degrees(lines_vector[2]))

    # Filter moviment
    if len(list_moviment) < 5:
        list_moviment.append(out1)
        # rospy.loginfo('Size of list: %f',len(list_moviment))
        # rospy.loginfo('****Incomplete List Moviment')
        # rospy.loginfo('------------------------------')
        
    else:
        list_moviment.append(out1)
        del list_moviment[0]
        sum_foward = sum(list_moviment)
        # rospy.loginfo('Size of list Moviment: %f',len(list_moviment))
        # rospy.loginfo('Sum List Moviment: %f',sum_foward)

        if sum_foward == 0:
            moviment = 0
            list_rotation = []
            ###### LIMPAR vetor list rotation
            
        if sum_foward == 5:
            moviment = 1

        # if moviment == 0:
        #     rospy.loginfo("Foward! (Filter)")
        # else: 
        #     rospy.loginfo("Curve! (Filter)")
        #     rospy.loginfo('------------------------------')

    # Filter rotation
    if len(list_rotation) < 5:
        list_rotation.append(out2)
        # rospy.loginfo('Size of list Rotation: %f',len(list_rotation))
        # rospy.loginfo('****Incomplete List Rotation')
        # rospy.loginfo('------------------------------')
        
    else:
        list_rotation.append(out2)
        del list_rotation[0]
        sum_rotation = sum(list_rotation)
        # rospy.loginfo('Size of list Rotation: %f',len(list_rotation))
        #rospy.loginfo('Complete list: [%f %f %f %f %f]',list_out[0],list_out[1],list_out[2],list_out[3],list_out[4])
        # rospy.loginfo('Sum List Rotation: %f',sum_rotation)

        if sum_rotation == 0:
            rotation = 0
            
        if sum_rotation == 5:
            rotation = 1

        # if rotation == 0:
        #     rospy.loginfo("Curve Right! (Filter)")
        # else: 
        #     rospy.loginfo("Curve Left! (Filter)")
        #     rospy.loginfo('------------------------------')




    #rospy.loginfo("Position Z %f", z_position)
    ganho_pid_altura = 5
    altura_desejada = 2.5
    # y in the drone of ROS = X in the image
    erro = float(2.5 - z_position)
    if erro > abs(0.2):
        z_correction = float(2.5 - z_position)/ganho_pid_altura
        #rospy.loginfo("Correction Z %f", z_correction)
    else:
        z_correction = 0
        #rospy.loginfo("Correction Z %f", z_correction)

    nav_drone = Twist()

    nav_drone.linear.z = z_correction

    if lines is not None:
      rospy.loginfo("Navigation!")
      if moviment == 0:
        rospy.loginfo("Reta (Filter)")
        nav_drone.linear.x = 0.05
        nav_drone.linear.y = y_correction
        nav_drone.linear.z = 0

        nav_drone.angular.x = 0
        nav_drone.angular.y = 0
        nav_drone.angular.z = yaw*(np.pi/180)
        rospy.loginfo("yaw: %f deg/s",nav_drone.angular.z*(180/np.pi))
        rospy.loginfo("-------------------------")

      else:
        rospy.loginfo("Curva (Filter)")
        if rotation == 1:
            nav_drone.linear.x = 0
            nav_drone.linear.y = 0
            nav_drone.linear.z = 0

            nav_drone.angular.x = 0
            nav_drone.angular.y = 0
            nav_drone.angular.z = 0.5*(np.pi/180)
            rospy.loginfo("...Left-yaw: %f deg/s",nav_drone.angular.z*(180/np.pi))
            rospy.loginfo("-------------------------")

        else:
            nav_drone.linear.x = 0
            nav_drone.linear.y = 0
            nav_drone.linear.z = 0

            nav_drone.angular.x = 0
            nav_drone.angular.y = 0
            nav_drone.angular.z = -0.5*(np.pi/180)
            rospy.loginfo("...Right-yaw: %f deg/s",nav_drone.angular.z*(180/np.pi))
            rospy.loginfo("-------------------------")

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
      self.navigation_pub.publish(nav_drone)
    except:
      rospy.loginfo('No publish!')

    cv2.imshow("Image-lines",resize)
    #cv2.imshow("Image-edges",edges)
    cv2.waitKey(1)



    # try:
    #   self.img_filter_pub.publish(resize)
    #   
    # except CvBridgeError as e:
    #   print(e)
  
###############################################################################


def sigmoid(Z):
    """
    Implements the sigmoid activation in numpy
    
    Arguments:
    Z -- numpy array of any shape
    
    Returns:
    A -- output of sigmoid(z), same shape as Z
    cache -- returns Z as well, useful during backpropagation
    """
    
    A = 1/(1+np.exp(-Z))
    cache = Z
    
    return A, cache


def relu(Z):
    """
    Implement the RELU function.

    Arguments:
    Z -- Output of the linear layer, of any shape

    Returns:
    A -- Post-activation parameter, of the same shape as Z
    cache -- a python dictionary containing "A" ; stored for computing the backward pass efficiently
    """
    
    A = np.maximum(0,Z)
    
    assert(A.shape == Z.shape)
    
    cache = Z 
    return A, cache


def relu_backward(dA, cache):
    """
    Implement the backward propagation for a single RELU unit.

    Arguments:
    dA -- post-activation gradient, of any shape
    cache -- 'Z' where we store for computing backward propagation efficiently

    Returns:
    dZ -- Gradient of the cost with respect to Z
    """
    
    Z = cache
    dZ = np.array(dA, copy=True) # just converting dz to a correct object.
    
    # When z <= 0, you should set dz to 0 as well. 
    dZ[Z <= 0] = 0
    
    assert (dZ.shape == Z.shape)
    
    return dZ


def sigmoid_backward(dA, cache):
    """
    Implement the backward propagation for a single SIGMOID unit.

    Arguments:
    dA -- post-activation gradient, of any shape
    cache -- 'Z' where we store for computing backward propagation efficiently

    Returns:
    dZ -- Gradient of the cost with respect to Z
    """
    
    Z = cache
    
    s = 1/(1+np.exp(-Z))
    dZ = dA * s * (1-s)
    
    assert (dZ.shape == Z.shape)
    
    return dZ


def initialize_parameters(n_x, n_h, n_y):
    """
    Argument:
    n_x -- size of the input layer
    n_h -- size of the hidden layer
    n_y -- size of the output layer
    
    Returns:
    parameters -- python dictionary containing your parameters:
                    W1 -- weight matrix of shape (n_h, n_x)
                    b1 -- bias vector of shape (n_h, 1)
                    W2 -- weight matrix of shape (n_y, n_h)
                    b2 -- bias vector of shape (n_y, 1)
    """
    
    np.random.seed(1)
    
    W1 = np.random.randn(n_h, n_x)*0.01
    b1 = np.zeros((n_h, 1))
    W2 = np.random.randn(n_y, n_h)*0.01
    b2 = np.zeros((n_y, 1))
    
    assert(W1.shape == (n_h, n_x))
    assert(b1.shape == (n_h, 1))
    assert(W2.shape == (n_y, n_h))
    assert(b2.shape == (n_y, 1))
    
    parameters = {"W1": W1,
                  "b1": b1,
                  "W2": W2,
                  "b2": b2}
    
    return parameters     


def initialize_parameters_deep(layer_dims):
    """
    Arguments:
    layer_dims -- python array (list) containing the dimensions of each layer in our network
    
    Returns:
    parameters -- python dictionary containing your parameters "W1", "b1", ..., "WL", "bL":
                    Wl -- weight matrix of shape (layer_dims[l], layer_dims[l-1])
                    bl -- bias vector of shape (layer_dims[l], 1)
    """
    
    np.random.seed(1)
    parameters = {}
    L = len(layer_dims)            # number of layers in the network

    for l in range(1, L):
        parameters['W' + str(l)] = np.random.randn(layer_dims[l], layer_dims[l-1]) / np.sqrt(layer_dims[l-1]) #*0.01
        parameters['b' + str(l)] = np.zeros((layer_dims[l], 1))
        
        assert(parameters['W' + str(l)].shape == (layer_dims[l], layer_dims[l-1]))
        assert(parameters['b' + str(l)].shape == (layer_dims[l], 1))

        
    return parameters

def linear_forward(A, W, b):
    """
    Implement the linear part of a layer's forward propagation.

    Arguments:
    A -- activations from previous layer (or input data): (size of previous layer, number of examples)
    W -- weights matrix: numpy array of shape (size of current layer, size of previous layer)
    b -- bias vector, numpy array of shape (size of the current layer, 1)

    Returns:
    Z -- the input of the activation function, also called pre-activation parameter 
    cache -- a python dictionary containing "A", "W" and "b" ; stored for computing the backward pass efficiently
    """
    
    # rospy.loginfo("******DEBUG******")
    # print("A-Shape: ",A.shape)
    # print("W-Shape: ",W.shape)
    # print("b-Shape: ",b.shape)

    Z = W.dot(A) + b
    #Z = np.dot(A,W) + b
    
    assert(Z.shape == (W.shape[0], A.shape[1]))
    cache = (A, W, b)
    
    return Z, cache

def linear_activation_forward(A_prev, W, b, activation):
    """
    Implement the forward propagation for the LINEAR->ACTIVATION layer

    Arguments:
    A_prev -- activations from previous layer (or input data): (size of previous layer, number of examples)
    W -- weights matrix: numpy array of shape (size of current layer, size of previous layer)
    b -- bias vector, numpy array of shape (size of the current layer, 1)
    activation -- the activation to be used in this layer, stored as a text string: "sigmoid" or "relu"

    Returns:
    A -- the output of the activation function, also called the post-activation value 
    cache -- a python dictionary containing "linear_cache" and "activation_cache";
             stored for computing the backward pass efficiently
    """
    
    if activation == "sigmoid":
        # Inputs: "A_prev, W, b". Outputs: "A, activation_cache".
        Z, linear_cache = linear_forward(A_prev, W, b)
        A, activation_cache = sigmoid(Z)
    
    elif activation == "relu":
        # Inputs: "A_prev, W, b". Outputs: "A, activation_cache".
        Z, linear_cache = linear_forward(A_prev, W, b)
        A, activation_cache = relu(Z)
    
    assert (A.shape == (W.shape[0], A_prev.shape[1]))
    cache = (linear_cache, activation_cache)

    return A, cache

def L_model_forward(X, parameters):
    """
    Implement forward propagation for the [LINEAR->RELU]*(L-1)->LINEAR->SIGMOID computation
    
    Arguments:
    X -- data, numpy array of shape (input size, number of examples)
    parameters -- output of initialize_parameters_deep()
    
    Returns:
    AL -- last post-activation value
    caches -- list of caches containing:
                every cache of linear_relu_forward() (there are L-1 of them, indexed from 0 to L-2)
                the cache of linear_sigmoid_forward() (there is one, indexed L-1)
    """

    caches = []
    A = X
    L = len(parameters) // 2                  # number of layers in the neural network
    
    # Implement [LINEAR -> RELU]*(L-1). Add "cache" to the "caches" list.
    for l in range(1, L):
        A_prev = A 
        A, cache = linear_activation_forward(A_prev, parameters['W' + str(l)], parameters['b' + str(l)], activation = "relu")
        caches.append(cache)
    
    # Implement LINEAR -> SIGMOID. Add "cache" to the "caches" list.
    AL, cache = linear_activation_forward(A, parameters['W' + str(L)], parameters['b' + str(L)], activation = "sigmoid")
    caches.append(cache)
    
    assert(AL.shape == (1,X.shape[1]))
            
    return AL, caches

def compute_cost(AL, Y):
    """
    Implement the cost function defined by equation (7).

    Arguments:
    AL -- probability vector corresponding to your label predictions, shape (1, number of examples)
    Y -- true "label" vector (for example: containing 0 if non-cat, 1 if cat), shape (1, number of examples)

    Returns:
    cost -- cross-entropy cost
    """
    
    m = Y.shape[1]

    # Compute loss from aL and y.
    cost = (1./m) * (-np.dot(Y,np.log(AL).T) - np.dot(1-Y, np.log(1-AL).T))
    
    cost = np.squeeze(cost)      # To make sure your cost's shape is what we expect (e.g. this turns [[17]] into 17).
    assert(cost.shape == ())
    
    return cost

def linear_backward(dZ, cache):
    """
    Implement the linear portion of backward propagation for a single layer (layer l)

    Arguments:
    dZ -- Gradient of the cost with respect to the linear output (of current layer l)
    cache -- tuple of values (A_prev, W, b) coming from the forward propagation in the current layer

    Returns:
    dA_prev -- Gradient of the cost with respect to the activation (of the previous layer l-1), same shape as A_prev
    dW -- Gradient of the cost with respect to W (current layer l), same shape as W
    db -- Gradient of the cost with respect to b (current layer l), same shape as b
    """
    A_prev, W, b = cache
    m = A_prev.shape[1]

    dW = 1./m * np.dot(dZ,A_prev.T)
    db = 1./m * np.sum(dZ, axis = 1, keepdims = True)
    dA_prev = np.dot(W.T,dZ)
    
    assert (dA_prev.shape == A_prev.shape)
    assert (dW.shape == W.shape)
    assert (db.shape == b.shape)
    
    return dA_prev, dW, db

def linear_activation_backward(dA, cache, activation):
    """
    Implement the backward propagation for the LINEAR->ACTIVATION layer.
    
    Arguments:
    dA -- post-activation gradient for current layer l 
    cache -- tuple of values (linear_cache, activation_cache) we store for computing backward propagation efficiently
    activation -- the activation to be used in this layer, stored as a text string: "sigmoid" or "relu"
    
    Returns:
    dA_prev -- Gradient of the cost with respect to the activation (of the previous layer l-1), same shape as A_prev
    dW -- Gradient of the cost with respect to W (current layer l), same shape as W
    db -- Gradient of the cost with respect to b (current layer l), same shape as b
    """
    linear_cache, activation_cache = cache
    
    if activation == "relu":
        dZ = relu_backward(dA, activation_cache)
        dA_prev, dW, db = linear_backward(dZ, linear_cache)
        
    elif activation == "sigmoid":
        dZ = sigmoid_backward(dA, activation_cache)
        dA_prev, dW, db = linear_backward(dZ, linear_cache)
    
    return dA_prev, dW, db

def L_model_backward(AL, Y, caches):
    """
    Implement the backward propagation for the [LINEAR->RELU] * (L-1) -> LINEAR -> SIGMOID group
    
    Arguments:
    AL -- probability vector, output of the forward propagation (L_model_forward())
    Y -- true "label" vector (containing 0 if non-cat, 1 if cat)
    caches -- list of caches containing:
                every cache of linear_activation_forward() with "relu" (there are (L-1) or them, indexes from 0 to L-2)
                the cache of linear_activation_forward() with "sigmoid" (there is one, index L-1)
    
    Returns:
    grads -- A dictionary with the gradients
             grads["dA" + str(l)] = ... 
             grads["dW" + str(l)] = ...
             grads["db" + str(l)] = ... 
    """
    grads = {}
    L = len(caches) # the number of layers
    m = AL.shape[1]
    Y = Y.reshape(AL.shape) # after this line, Y is the same shape as AL
    
    # Initializing the backpropagation
    dAL = - (np.divide(Y, AL) - np.divide(1 - Y, 1 - AL))
    
    # Lth layer (SIGMOID -> LINEAR) gradients. Inputs: "AL, Y, caches". Outputs: "grads["dAL"], grads["dWL"], grads["dbL"]
    current_cache = caches[L-1]
    grads["dA" + str(L)], grads["dW" + str(L)], grads["db" + str(L)] = linear_activation_backward(dAL, current_cache, activation = "sigmoid")
    
    for l in reversed(range(L-1)):
        # lth layer: (RELU -> LINEAR) gradients.
        current_cache = caches[l]
        dA_prev_temp, dW_temp, db_temp = linear_activation_backward(grads["dA" + str(l + 2)], current_cache, activation = "relu")
        grads["dA" + str(l + 1)] = dA_prev_temp
        grads["dW" + str(l + 1)] = dW_temp
        grads["db" + str(l + 1)] = db_temp

    return grads

def update_parameters(parameters, grads, learning_rate):
    """
    Update parameters using gradient descent
    
    Arguments:
    parameters -- python dictionary containing your parameters 
    grads -- python dictionary containing your gradients, output of L_model_backward
    
    Returns:
    parameters -- python dictionary containing your updated parameters 
                  parameters["W" + str(l)] = ... 
                  parameters["b" + str(l)] = ...
    """
    
    L = len(parameters) // 2 # number of layers in the neural network

    # Update rule for each parameter. Use a for loop.
    for l in range(L):
        parameters["W" + str(l+1)] = parameters["W" + str(l+1)] - learning_rate * grads["dW" + str(l+1)]
        parameters["b" + str(l+1)] = parameters["b" + str(l+1)] - learning_rate * grads["db" + str(l+1)]
        
    return parameters

def predict(X, y, parameters):
    """
    This function is used to predict the results of a  L-layer neural network.
    
    Arguments:
    X -- data set of examples you would like to label
    parameters -- parameters of the trained model
    
    Returns:
    p -- predictions for the given dataset X
    """
    
    m = X.shape[1]
    n = len(parameters) // 2 # number of layers in the neural network
    p = np.zeros((1,m))
    
    # Forward propagation
    probas, caches = L_model_forward(X, parameters)

    
    # convert probas to 0/1 predictions
    for i in range(0, probas.shape[1]):
        if probas[0,i] > 0.5:
            p[0,i] = 1
        else:
            p[0,i] = 0
    
    #print results
    #print ("predictions: " + str(p))
    #print ("true labels: " + str(y))
    print("Accuracy: "  + str(np.sum((p == y)/m)))
        
    return p

def predict_curve(X, parameters):
    """
    This function is used to predict the results of a  L-layer neural network.
    
    Arguments:
    X -- data set of examples you would like to label
    parameters -- parameters of the trained model
    
    Returns:
    p -- predictions for the given dataset X
    """
    
    m = X.shape[1]
    n = len(parameters) // 2 # number of layers in the neural network
    p = np.zeros((1,m))

    
    # Forward propagation
    probas, caches = L_model_forward(X, parameters)

    
    # convert probas to 0/1 predictions
    for i in range(0, probas.shape[1]):
        if probas[0,i] > 0.5:
            p[0,i] = 1
        else:
            p[0,i] = 0
        
    return p

def print_mislabeled_images(classes, X, y, p):
    """
    Plots images where predictions and truth were different.
    X -- dataset
    y -- true labels
    p -- predictions
    """
    a = p + y
    mislabeled_indices = np.asarray(np.where(a == 1))
    plt.rcParams['figure.figsize'] = (40.0, 40.0) # set default size of plots
    num_images = len(mislabeled_indices[0])
    for i in range(num_images):
        index = mislabeled_indices[1][i]
        
        plt.subplot(2, num_images, i + 1)
        plt.imshow(X[:,index].reshape(64,64,3), interpolation='nearest')
        plt.axis('off')
        plt.title("Prediction: " + classes[int(p[0,index])].decode("utf-8") + " \n Class: " + classes[y[0,index]].decode("utf-8"))


def main(args):
  #-- Name of node
  rospy.init_node('lines', log_level=rospy.DEBUG)
  
  ic = lines()
  
  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)