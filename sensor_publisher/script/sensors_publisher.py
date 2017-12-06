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
from math import sqrt, atan2, asin, degrees, radians, pi
import os
from std_msgs.msg import Int32, Float32




class sensor_publisher():

  dir = os.path.dirname(__file__)
  filename = os.path.join(dir, '../data/Data.csv')

  def __init__(self):
    rospy.init_node('sensor_publisher')
    pubGPS = rospy.Publisher("GPS", Odometry)
    pub_GPS_Orientation = rospy.Publisher('Orientation_GPS', Float32)
    pubGPSPoint = rospy.Publisher("GPSPoint", PointStamped)
    rate = rospy.Rate(25.0)
    rospy.loginfo('sensors publisher successfully started')
    P = np.mat(np.diag([0.5]*3))

    self.imu = Imu()
    self.imu_pub = rospy.Publisher('imu/data_raw', Imu)

    self.imu_mag = MagneticField()
    self.mag_pub = rospy.Publisher('imu/mag', MagneticField)

    StepN_pub= rospy.Publisher('Step_Counter', Int32)

    GPS=self.GPS_load_data_from_CSV()
    IMU=self.IMU_load_data_from_CSV()
    StepN=self.StepCounter_load_data_from_CSV()
    fuse = Fusion()


    mag_x_offset = -0.4200000762939453
    mag_y_offset = -1.260000228881836
    mag_z_offset = 0.20999908447265625

    mag_x_scale = 0.9850912029395295
    mag_y_scale = 0.9877287231497508
    mag_z_scale = 1.0283391398550155
    index=0
    moving_average_n_element=5
    accel_x=[]
    accel_y=[]
    accel_z=[]
    gyro_x=[]
    gyro_y=[]
    gyro_z=[]
    mag_x=[]
    mag_y=[]
    mag_z=[]
    GPS_orientation=[]
    gps_orientation=0
    x,y,n,b=utm.from_latlon(GPS[index][1],GPS[index][2])
    x_prev=x
    y_prev=y
    while not rospy.is_shutdown():
      p=Odometry()
      p.header.stamp = rospy.Time.now()
      p.header.frame_id = "/map"
      p.child_frame_id = ""
      x,y,n,b=utm.from_latlon(GPS[index][1],GPS[index][2])
      if(x_prev != x and y_prev != y):
        if(len(GPS_orientation)<2):
	  GPS_orientation.append((x,y))
        else:
	  del GPS_orientation[0]
	  GPS_orientation.append((x,y))
          gps_orientation= self.theta_two_points(GPS_orientation[0][0],GPS_orientation[0][1],GPS_orientation[1][0],GPS_orientation[1][1])
	  pub_GPS_Orientation.publish(gps_orientation)
	
        
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

      if(len(accel_x)<moving_average_n_element):
        accel_x.append(IMU[index][0])
    	accel_y.append(IMU[index][1])
    	accel_z.append(IMU[index][2])
    	gyro_x.append(IMU[index][3])
    	gyro_y.append(IMU[index][4])
    	gyro_z.append(IMU[index][5])
	mag_x.append(IMU[index][6] + mag_x_offset + mag_x_scale)
    	mag_y.append(IMU[index][7] + mag_y_offset + mag_y_scale)
    	mag_z.append(IMU[index][8] + mag_z_offset + mag_z_scale)
      else:
        del accel_x[0]
    	del accel_y[0]
    	del accel_z[0]
    	del gyro_x[0]
    	del gyro_y[0]
    	del gyro_z[0]
    	del mag_x[0]
    	del mag_y[0]
    	del mag_z[0]
	accel_x.append(IMU[index][0])
    	accel_y.append(IMU[index][1])
    	accel_z.append(IMU[index][2])
    	gyro_x.append(IMU[index][3])
    	gyro_y.append(IMU[index][4])
    	gyro_z.append(IMU[index][5])
    	mag_x.append(IMU[index][6] + mag_x_offset + mag_x_scale)
    	mag_y.append(IMU[index][7] + mag_y_offset + mag_y_scale)
    	mag_z.append(IMU[index][8] + mag_z_offset + mag_z_scale)


      #print(accel,gyro,mag)
      #fuse.update(accel, gyro, mag)
      #quaternion = tf.transformations.quaternion_from_euler(fuse.pitch, fuse.roll, fuse.heading)
      #print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))

      now = rospy.Time.now()
      imu = Imu(header=rospy.Header(frame_id="/map"))
      imu.header.stamp = now
      imu.orientation.x = 0
      imu.orientation.y = 0
      imu.orientation.z = 0
      imu.orientation.w = 0
      imu.linear_acceleration.x = np.mean(accel_x)
      imu.linear_acceleration.y = np.mean(accel_y)
      imu.linear_acceleration.z = np.mean(accel_z)
      imu.angular_velocity.x = np.mean(gyro_x)
      imu.angular_velocity.y = np.mean(gyro_y)
      imu.angular_velocity.z = np.mean(gyro_z)

      imu_mag = MagneticField(header=rospy.Header(frame_id="/map"))
      imu_mag.header.stamp = now
      imu_mag.magnetic_field.x = np.mean(mag_x)
      imu_mag.magnetic_field.y = np.mean(mag_y)
      imu_mag.magnetic_field.z = np.mean(mag_z)

      numberOfStep= StepN[index]

      self.imu = imu
      self.imu_pub.publish(self.imu)
      self.imu_mag = imu_mag
      self.mag_pub.publish(self.imu_mag)
      pubGPS.publish(p)
      pubGPSPoint.publish(p2)
      print(numberOfStep)
      StepN_pub.publish(int(numberOfStep))
      
      index+=1
      x_prev=x
      y_prev=y
      rate.sleep()

  def theta_two_points(self,x0,y0,x1,y1):
    theta = atan2(y1 - y0, x1 - x0);
    if (theta < 0.0):
      theta += 2*pi
    return theta


  def GPS_load_data_from_CSV(self):
    try:
      df = pd.read_csv(self.filename, delimiter=',')
    except Exception, e:
      print ("Error in reading", self.filename)
      print (e)
    GPS=df.values[:,4:7]
    #GPS.sort(axis=0)
    nanIndex= pd.isnull(GPS)
    GPS[nanIndex]=0
    print(GPS[0])
    return GPS

  def IMU_load_data_from_CSV(self):
    try:
      df = pd.read_csv(self.filename, delimiter=',')
    except Exception, e:
      print ("Error in reading", self.filename)
      print (e)
    IMU=df.values[:,7:16]
    #IMU.sort(axis=0)
    nanIndex= pd.isnull(IMU)
    IMU[nanIndex]=0
    print(IMU[0])
    return IMU

  def StepCounter_load_data_from_CSV(self):
    try:
      df = pd.read_csv(self.filename, delimiter=',')
    except Exception, e:
      print ("Error in reading", self.filename)
      print (e)
    StepN=df.values[:,16]
    nanIndex= pd.isnull(StepN)
    StepN[nanIndex]=0
    print(StepN[0])
    return StepN
	
    
    
    
  


if __name__ == '__main__':
  try:
    loc = sensor_publisher()
  except rospy.ROSInterruptException:
    pass
