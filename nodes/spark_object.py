# 
# this function can obtain the specific color object (i.e. blue)
# publish: the topic of position of the coordinate of object in the pixel frame
# created by Jason, Dec. 20th, 2019


#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import tf
# import torch

from numpy import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError

class detector_from_color:

  def __init__(self):
    global xc, yc, xc_prev, yc_prev, found_count
    global trans, rot
    xc = 0
    yc = 0
    xc_prev = xc
    yc_prev = yc
    found_count = 0
     
    self.sub_color = rospy.Subscriber("/camera/rgb/image_raw", Image, self.color_cb, queue_size=1)
    
 
    self.sub_depth = rospy.Subscriber("/camera/depth/image", Image, self.depth_cb, queue_size=1)

    self.pub_point = rospy.Publisher('position_write_topic', Point, queue_size=10)
   # self.pub1 = rospy.Publisher('object_position', position, queue_size=5)


  # since no library can be used, we decide to write this function by ourself
  def rotation_from_quaternion(self, x):
    '''
        Converts a quaternion to a rotation matrix.
    '''
    # Documented in <http://en.wikipedia.org/w/index.php?title=
    # Quaternions_and_spatial_rotation&oldid=402924915>
    a, b, c, d = x

    r1 = [a ** 2 + b ** 2 - c ** 2 - d ** 2,
          2 * b * c - 2 * a * d,
          2 * b * d + 2 * a * c]
    r2 = [2 * b * c + 2 * a * d,
          a ** 2 - b ** 2 + c ** 2 - d ** 2,
          2 * c * d - 2 * a * b]
    r3 = [2 * b * d - 2 * a * c,
          2 * c * d + 2 * a * b,
          a ** 2 - b ** 2 - c ** 2 + d ** 2]

    return np.array([r1, r2, r3])


  def quaternion_from_rotation(slef, R):
    '''
        Converts a rotation matrix to a quaternion.
        This is the robust method mentioned on wikipedia:
        <http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>
        TODO: add the more robust method with 4x4 matrix and eigenvector
    '''
    largest = np.argmax(R.diagonal())
    permutations = {0: [0, 1, 2],
                    1: [1, 2, 0],
                    2: [2, 0, 1]}
    u, v, w = permutations[largest]
    rr = 1 + R[u, u] - R[v, v] - R[w, w]
    assert rr >= 0
    r = np.sqrt(rr)
    if r == 0:  # TODO: add tolerance
        return quaternion_from_axis_angle(default_axis(), 0.0)
    else:
        q0 = (R[w, v] - R[v, w]) / (2 * r)
        qu = (r / 2)
        qv = (R[u, v] + R[v, u]) / (2 * r)
        qw = (R[w, u] + R[u, w]) / (2 * r)

        Q = np.zeros(4)
        Q[0] = q0
        Q[u + 1] = qu
        Q[v + 1] = qv
        Q[w + 1] = qw
        # if Q[0] < 0:
            # Q = -Q
        return Q

  def cal_world_coordinate(self, data):
    global trans, rot
    # rot_array = np.array([[rot[0], rot[1], rot[2], rot[3]]]).T
    print(trans[0])
    rot_array = np.array(rot)
    trans_array = np.array(trans)

    rotation_matrix = self.rotation_from_quaternion(rot_array)
    # print(self.quaternion_from_rotation(rotation_matrix))
    transformation_matrix = mat(np.c_[rotation_matrix, trans_array])

    b = np.array([0, 0, 0, 1])
    
    transformation_matrix = vstack((transformation_matrix,b))
    

    temp = Point()
    temp_1 = data
 
    # print(temp_1[0])
    # P^w = T_c^{w}*P^c
    a = mat(np.array([[temp_1.x, temp_1.y, temp_1.z, 1]]).T)
    # print(a, type(a), transformation_matrix, type(transformation_matrix))
    # note that multiply for matrix in py
    temp_matrix = transformation_matrix * a
    print(temp_matrix)
    temp.x = temp_matrix[0, 0]
    temp.y = temp_matrix[1, 0]
    temp.z = temp_matrix[2, 0]
    # print("world", temp)
    return temp

  # realize n camera frame
  # and publish transformation of grasper
  def cal_translation(self, data):
    
    # transfer depth value
    d = data

    camera_factor = 1
    camera_cx = 314.005672
    camera_cy = 242.391705
    camera_fx = 520.377485
    camera_fy = 521.494097

    p = Point()
    # calculate this point's space coordinate
    p.z = d / camera_factor;
    p.x = (xc_prev - camera_cx) * p.z / camera_fx;
    p.y = (yc_prev - camera_cy) * p.z / camera_fy;
    # p.x = (yc_prev - camera_cy) * p.z / camera_fy;
    # p.y = (xc_prev - camera_cx) * p.z / camera_fx;
    print("point\n", p)

    # translate the coordinate of point into world frame    
    # world_p = Point()
    # world_p = self.cal_world_coordinate(p)
    # print("world_p\n", world_p)

    
    translation_factor = 1000
    translation = Point()
    # translation from grasper to object
    if (np.isnan(p.x)):
      print('nan')
    else:
      # x,y inverse with the real x,y
      # translation.x = (world_origin_arm.y - world_p.y) * translation_factor
      # translation.y = (world_p.x - world_origin_arm.x) * translation_factor
      # translation.z = (world_origin_arm.z - world_p.z) * translation_factor
      
      '''
      we will define a linear regression between 
      the pose of the camera and calibration, the equation is
      y = 6(x - 0.626) + 0.25
      # -0.626
      translation.y = (-0.01 - p.x) * translation_factor
      translation.x = (0.25 - p.y) * translation_factor
      # -0.646
      translation.y = (0 - p.x) * translation_factor
      translation.x = (0.37 - p.y) * translation_factor
      # -0.665
      translation.y = (0 - p.x) * translation_factor
      translation.x = (0.5 - p.y) * translation_factor
      '''
      print(trans[2])
      origin_arm = Point()
      origin_arm.x = 0
      origin_arm.y = 6 * (-trans[2] - 0.626) + 0.25
      # note taht the coordinate of grasper is reverse to the camera
      translation.y = (origin_arm.x - p.x) * translation_factor
      translation.x = (origin_arm.y - p.y) * translation_factor
      translation.z = 35
      
    print("translation in mm in the world frame\n", translation)
    r1 = rospy.Rate(1)  # 1s
    r1.sleep()
    self.pub_point.publish(translation)
    
  def depth_cb(self, data):

    try:
        depth = CvBridge().imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
        print('error')

    d = depth[yc_prev, xc_prev]
    print("depth", d)

    # calculate and publish translation
    self.cal_translation(d)    

  
  def color_cb(self, data):
    #print("received image\n")
    global xc, yc, xc_prev, yc_prev, found_count
    # change to opencv
    try:
      cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print('error')

    # change rgb to hsv
    cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)

    # extract blue
    LowerBlue = np.array([100, 90, 80])
    UpperBlue = np.array([130, 255, 255])
    mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
    cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)

    # gray process
    cv_image4 = cv_image3[:, :, 0]

    # smooth and clean noise
    blurred = cv2.blur(cv_image4, (9, 9))
    (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    cv_image5 = cv2.erode(cv_image5, None, iterations=4)
    cv_image5 = cv2.dilate(cv_image5, None, iterations=4)

    # detect contour
    #cv2.imshow("win1", cv_image1)
    #cv2.imshow("win2", cv_image5)
    #cv2.waitKey(1)
    _, contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # if find contours, pick the max box
    # return the central point of the max box
    if len(contours) > 0:
        size = []
        size_max = 0
        for i, c in enumerate(contours):
            rect = cv2.minAreaRect(c)
            # draw
            cv2.ellipse(cv_image5, rect, (0, 255, 0), 2, 8)
            cv2.circle(cv_image5, (np.int32(rect[0][0]), np.int32(rect[0][1])), 2, (255, 0, 0), 2, 8, 0)

            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # choose the central of shape
            x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
            y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
            w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
            h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2) 
            size.append(w * h)
            if size[i] > size_max:
                size_max = size[i]
                index = i
                xc = x_mid
                yc = y_mid
        # if box is not moving for 20 times
        #print found_count
        if found_count >= 20:
            self.is_found_object = True
        else:
            # if box is not moving
            if abs(xc - xc_prev) <= 2 and abs(yc - yc_prev) <= 2:
                found_count = found_count + 1
            else:
                found_count = 0
    else:
        found_count = 0
    xc_prev = xc
    yc_prev = yc   
    print("object coordiante in pixel: ",xc_prev, yc_prev)

    cv2.imshow("output", cv_image5)
    cv2.waitKey(1)

    # try:
    #   self.pub1.publish(position)
    # except CvBridgeError as e:
    #   print(e)


def main(args):
  dfc = detector_from_color()
  rospy.init_node('detector_from_color', anonymous=True)
  listener = tf.TransformListener()
  rate = rospy.Rate(10) # 10hz
  global trans, rot
  while not rospy.is_shutdown():
    try:
      (trans, rot) = listener.lookupTransform("/camera_link", "/world_link", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    # print("hello", trans, rot)

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)
    rate.sleep()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
