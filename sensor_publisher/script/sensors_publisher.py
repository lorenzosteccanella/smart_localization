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
from fusion import Fusion
from sensor_msgs.msg import Imu, MagneticField
import time
from math import sqrt, atan2, asin, degrees, radians




class sensor_publisher():

  def __init__(self):
    rospy.init_node('sensor_publisher')
    pubGPS = rospy.Publisher("GPS", Odometry)
    pubGPSPoint = rospy.Publisher("GPSPoint", PointStamped)
    rate = rospy.Rate(25.0)
    rospy.loginfo('sensors publisher successfully started')
    P = np.mat(np.diag([0.5]*3))

    self.imu = Imu()
    self.imu_pub = rospy.Publisher('imu/data_raw', Imu)

    self.imu_mag = MagneticField()
    self.mag_pub = rospy.Publisher('imu/mag', MagneticField)

    GPS=self.GPS_load_data_from_CSV()
    IMU=self.IMU_load_data_from_CSV()

    fuse = Fusion()

    index=0
    while not rospy.is_shutdown():
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
      
      p2=PointStamped()
      p2.header.stamp = rospy.Time.now()
      p2.header.frame_id = "/map"
      p2.point.x=p.pose.pose.position.x
      p2.point.y=p.pose.pose.position.y
      p2.point.z=0.0

      accel = (IMU[index][0], IMU[index][1], IMU[index][2])
      gyro = (IMU[index][3], IMU[index][4], IMU[index][5])
      mag = (IMU[index][6], IMU[index][7], IMU[index][8])

      #print(accel,gyro,mag)
      fuse.update(accel, gyro, mag)
      quaternion = tf.transformations.quaternion_from_euler(fuse.pitch, fuse.roll, fuse.heading)
      #print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))

      now = rospy.Time.now()
      imu = Imu(header=rospy.Header(frame_id="/map"))
      imu.header.stamp = now
      imu.orientation.x = quaternion[0]
      imu.orientation.y = quaternion[1]
      imu.orientation.z = quaternion[2]
      imu.orientation.w = quaternion[3]
      imu.linear_acceleration.x = accel[0]
      imu.linear_acceleration.y = accel[1]
      imu.linear_acceleration.z = accel[2]
      imu.angular_velocity.x = gyro[0]
      imu.angular_velocity.y = gyro[1]
      imu.angular_velocity.z = gyro[2]

      imu_mag = MagneticField(header=rospy.Header(frame_id="/map"))
      imu_mag.header.stamp = now
      imu_mag.magnetic_field.x = mag[0]
      imu_mag.magnetic_field.y = mag[1]
      imu_mag.magnetic_field.z = mag[2]

      self.imu = imu
      self.imu_pub.publish(self.imu)
      self.imu_mag = imu_mag
      self.mag_pub.publish(self.imu_mag)
      pubGPS.publish(p)
      pubGPSPoint.publish(p2)
      
      index+=1
      rate.sleep()

  def GPS_load_data_from_CSV(self):
    df = pd.read_csv('/home/lorenzo/smart_localization/src/sensor_publisher/data/Data.csv', delimiter=',')
    GPS=df.values[:,4:7]
    #GPS.sort(axis=0)
    nanIndex= pd.isnull(GPS)
    GPS[nanIndex]=0
    print(GPS[0])
    return GPS

  def IMU_load_data_from_CSV(self):
    df = pd.read_csv('/home/lorenzo/smart_localization/src/sensor_publisher/data/Data.csv', delimiter=',')
    IMU=df.values[:,7:16]
    #IMU.sort(axis=0)
    nanIndex= pd.isnull(IMU)
    IMU[nanIndex]=0
    print(IMU[0])
    return IMU
	
    
    
    
  


if __name__ == '__main__':
  try:
    loc = sensor_publisher()
  except rospy.ROSInterruptException:
    pass
