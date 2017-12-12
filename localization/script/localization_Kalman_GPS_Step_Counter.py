#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import random
import csv
import pandas as pd
import time
from std_msgs.msg import UInt8, Float32
from math import sqrt, atan2, asin, degrees, radians, pi


class Sensor_Fusion():

  def __init__(self):
    rospy.init_node('Sensor_Fusion')
    self.orientation_gps_msg = Float32()
    self.gps_msg = Odometry()

    pub = rospy.Publisher("/Sensor_Fusion", Odometry)

    rospy.Subscriber('/Orientation_GPS', Float32, self.orientation_gps_cb)
    rospy.Subscriber('/GPS', Odometry, self.gps_cb)

    rate = rospy.Rate(25.0)
    rospy.loginfo('Sensor_Fusion successfully started')

    P = np.mat(np.diag([0.5]*3))
    moving_average_orientation=[]
    moving_average_size=1

    # Init Kalman
    x = np.matrix([[0.0, 0.0, 0.0]]).T
    # Initial Uncertainty
    P = np.diag([10.0, 10.0, 100.0])
    # Dinamic Matrix
    A = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    # Measurement Matrix
    H = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    # Measurement Noise Covariance
    rp=10.0**2
    ro=10.0**2
    R = np.matrix([[rp,0.0,0.0],[0.0,rp,0.0],[0.0,0.0,ro]])
    # Process Noise Covariance Matrix
    G = np.matrix([[1.0],[1.0],[1.0]])
    sa=0.001
    Q = G*G.T*sa**2
    # Identity Matrix
    I = np.eye(3)

    while not rospy.is_shutdown():

      pos = self.gps_msg.pose.pose.position
      rot = self.orientation_gps_msg
      if(len(moving_average_orientation)<moving_average_size):
        moving_average_orientation.append(rot.data)
      else:
        del moving_average_orientation[0]
        moving_average_orientation.append(rot.data)
      # Prediction
      print(x)
      x = A*x

      P = A*P*A.T + Q
      
      # Correction
      S = H*P*H.T + R
      K = (P*H.T) * np.linalg.pinv(S)
   
      # Update the estimate via Z
      Z=np.array([pos.x, pos.y, np.mean(moving_average_orientation)])
      y= Z - (H*x)
      x= x + (K*y)
      
      # Update the error covariance
      P = (I -(K*H))*P



      quaternion = tf.transformations.quaternion_from_euler(0, 0, np.mean(moving_average_orientation))
      p=Odometry()
      p.header.stamp = rospy.Time.now()
      p.header.frame_id = "/map"
      p.child_frame_id = ""
      p.pose.pose.position.x= pos.x
      p.pose.pose.position.y= pos.y
      p.pose.pose.position.z=0.0
      p.pose.pose.orientation.x=quaternion[0]
      p.pose.pose.orientation.y=quaternion[1]
      p.pose.pose.orientation.z=quaternion[2]
      p.pose.pose.orientation.w=quaternion[3]
      p_cov = np.array([0.0]*36).reshape(6,6)
      p_cov[0:2,0:2] = P[0:2,0:2]
      p.pose.covariance = tuple(p_cov.ravel().tolist())

      pub.publish(p)

      rate.sleep()

  def orientation_gps_cb(self, msg):
    self.orientation_gps_msg = msg  
  def gps_cb(self, msg):
    self.gps_msg = msg  








if __name__ == '__main__':
  try:
    orient = Sensor_Fusion()
  except rospy.ROSInterruptException:
    pass
