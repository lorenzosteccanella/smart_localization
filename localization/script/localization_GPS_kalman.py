#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
import random
import csv
import pandas as pd
import time
#kalman
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise





class localization_GPS_kalman():

  def __init__(self):
    rospy.init_node('localization_GPS_kalman')
    self.gps_msg = Odometry()

    pub = rospy.Publisher("/localization_GPS_Path", Path)
    pub2 = rospy.Publisher("/localization_GPS_Kalman_Path", Path)

    rospy.Subscriber('/GPS', Odometry, self.gps_cb)

    rate = rospy.Rate(1.0)
    rospy.loginfo('localization_GPS_kalman successfully started')
  
    path_gps_position=[]
    path_kalman_position=[]
  
    initial_guess=(self.gps_msg.pose.pose.position.x, self.gps_msg.pose.pose.position.x)
    tracker = self.tracker1(initial_guess)

    while not rospy.is_shutdown():

      pos = self.gps_msg.pose.pose.position
      path_gps_position.append((pos.x,pos.y))
      trajectory_gps=self.initPath_msg(path_gps_position)
      tracker.predict()
      z=np.array([[pos.x,pos.y]]).T
      tracker.update(z)
      print((tracker.x[0],tracker.x[2]),(pos.x,pos.y))
      path_kalman_position.append((tracker.x[0],tracker.x[2]))
      trajectory_kalman=self.initPath_msg(path_kalman_position)

      pub.publish(trajectory_gps)
      pub2.publish(trajectory_kalman)

      rate.sleep()
 
  def gps_cb(self, msg):
    self.gps_msg = msg  

  def initPath_msg(self, path_position):
    #path to send the Rviz
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "/map"
    for i in range(len(path_position)):

      somePose = PoseStamped()
      somePose.header.frame_id = "odom"
      somePose.header.stamp=rospy.Time.now()
      somePose.pose.position.x = path_position[i][0]
      somePose.pose.position.y = path_position[i][1]
      somePose.pose.position.z = 0.0
      Zorientation=0
      quaternion = tf.transformations.quaternion_from_euler(0, 0, Zorientation)
      somePose.pose.orientation.x = quaternion[0]
      somePose.pose.orientation.y = quaternion[1]
      somePose.pose.orientation.z = quaternion[2]
      somePose.pose.orientation.w = quaternion[3]

      path.poses.append(somePose)
    return path

  R_std = 0.35
  Q_std = 0.04

  def tracker1(self, initial_guess):
    tracker = KalmanFilter(dim_x=4, dim_z=2)
    dt = 1   # time step

    tracker.F = np.array([[1, dt, 0,  0],
                          [0,  1, 0,  0],
                          [0,  0, 1, dt],
                          [0,  0, 0,  1]])
    tracker.u = 0.
    tracker.H = np.array([[1, 0, 0, 0],
                          [0, 0, 1, 0]])

    tracker.R = np.eye(2) * self.R_std**2
    q = Q_discrete_white_noise(dim=2, dt=dt, var=self.Q_std**2)
    tracker.Q = block_diag(q, q)
    tracker.x = np.array([[initial_guess[0], 0, initial_guess[1], 0]]).T
    tracker.P = np.eye(4) * 5.
    return tracker








if __name__ == '__main__':
  try:
    orient = localization_GPS_kalman()
  except rospy.ROSInterruptException:
    pass
