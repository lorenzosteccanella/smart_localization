#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import random
import csv
import pandas as pd
import utm
from sensor_msgs.msg import Imu
import time




class localization():

  def __init__(self):
    rospy.init_node('localization')
    self.imu_msg = Imu()
    self.gps_msg = Odometry()

    pub = rospy.Publisher("/localization", Odometry)

    rospy.Subscriber('/imu/data', Imu, self.imu_cb)
    rospy.Subscriber('/GPS', Odometry, self.gps_cb)

    rate = rospy.Rate(25.0)
    rospy.loginfo('localization successfully started')

    P = np.mat(np.diag([0.5]*3))

    while not rospy.is_shutdown():

      pos = self.gps_msg.pose.pose.position
      rot = self.imu_msg.orientation
      quaternion = (
    	  rot.x,
    	  rot.y,
    	  rot.z,
    	  rot.w)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      quaternion = tf.transformations.quaternion_from_euler(0, 0, euler[2])
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

  def imu_cb(self, msg):
    self.imu_msg = msg  
  def gps_cb(self, msg):
    self.gps_msg = msg  








if __name__ == '__main__':
  try:
    orient = localization()
  except rospy.ROSInterruptException:
    pass
