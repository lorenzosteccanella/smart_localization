#!/usr/bin/env python
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import random
import csv
import pandas as pd
import utm




class GPS_publisher():

  def __init__(self):
    rospy.init_node('GPS_publisher')
    self.odom_msg = Odometry()
    pubGPS = rospy.Publisher("GPS", Odometry)
    pubGPSPoint = rospy.Publisher("GPSPoint", PointStamped)
    # Broadcast the transform at 25 HZ
    rate = rospy.Rate(25.0)
    rospy.loginfo('GPS fake sensor successfully started')
    P = np.mat(np.diag([0.5]*3))

    GPS=self.load_data_from_CSV()

    index=0
    while not rospy.is_shutdown():
      pos = self.odom_msg.pose.pose.position
      rot = self.odom_msg.pose.pose.orientation
      p=Odometry()
      p.header.stamp = rospy.Time.now()
      p.header.frame_id = "/map"
      p.child_frame_id = ""
      x,y,n,b=utm.from_latlon(GPS[index][1],GPS[index][2])
      p.pose.pose.position.x=x
      p.pose.pose.position.y=y
      p.pose.pose.position.z=0.0
      p.pose.pose.orientation.x=0.0
      p.pose.pose.orientation.y=0.0
      p.pose.pose.orientation.z=0.0
      p.pose.pose.orientation.w=1.0
      p_cov = np.array([0.0]*36).reshape(6,6)
      p_cov[0:2,0:2] = P[0:2,0:2]
      p.pose.covariance = tuple(p_cov.ravel().tolist())

      index+=1
      
      
      p2=PointStamped()
      p2.header.stamp = rospy.Time.now()
      p2.header.frame_id = "/map"
      p2.point.x=p.pose.pose.position.x
      p2.point.y=p.pose.pose.position.y
      p2.point.z=0.0
      pubGPS.publish(p)
      pubGPSPoint.publish(p2)
      rate.sleep()

  def load_data_from_CSV(self):
    df = pd.read_csv('/home/lorenzo/smart_localization/src/sensor_publisher/data/Data.csv', delimiter=',')
    GPS=df.values[:,4:7]
    #GPS.sort(axis=0)
    nanIndex= pd.isnull(GPS)
    GPS[nanIndex]=0
    print(GPS[0])
    return GPS

  def get_sync_UTM(self,time):
    return 0
	
    
    
    
  


if __name__ == '__main__':
  try:
    loc = GPS_publisher()
  except rospy.ROSInterruptException:
    pass
