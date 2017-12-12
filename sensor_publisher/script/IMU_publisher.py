#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PointStamped
import random
import csv
import pandas as pd
import utm
from fusion import Fusion
from sensor_msgs.msg import Imu
import time
from math import sqrt, atan2, asin, degrees, radians




class IMU_publisher():

  def __init__(self):
    rospy.init_node('IMU_publisher')
    self.imu = Imu()
    self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
    self.imu.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
    self.imu.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
    self.imu_pub = rospy.Publisher('imu', Imu)

    rate = rospy.Rate(25.0)
    rospy.loginfo('IMU fake sensor successfully started')
    IMU=self.load_data_from_CSV()

    fuse = Fusion()

    index=0
    while not rospy.is_shutdown():

      accel = (IMU[index][0], IMU[index][1], IMU[index][2])
      gyro = (degrees(IMU[index][3]), degrees(IMU[index][4]), degrees(IMU[index][5]))
      mag = (IMU[index][6], IMU[index][7], IMU[index][8])

      #print(accel,gyro,mag)
      fuse.update(accel, gyro, mag)
      quaternion = tf.transformations.quaternion_from_euler(0,0,fuse.heading)#(fuse.pitch, fuse.roll, fuse.heading)
      #print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))

      now = rospy.Time.now()
      imu = Imu(header=rospy.Header(frame_id="imu_link"))
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

      self.imu = imu
      self.imu_pub.publish(self.imu)

      index+=1
      rate.sleep()

  def load_data_from_CSV(self):
    df = pd.read_csv('/home/dario/catkin_ws/src/smart_localization/sensor_publisher/data/Data.csv', delimiter=',')
    IMU=df.values[:,7:16]
    #IMU.sort(axis=0)
    nanIndex= pd.isnull(IMU)
    IMU[nanIndex]=0
    print(IMU[0])
    return IMU

  def get_sync_UTM(self,time):
    return 0







if __name__ == '__main__':
  try:
    orient = IMU_publisher()
  except rospy.ROSInterruptException:
    pass
