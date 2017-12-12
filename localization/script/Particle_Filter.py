#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Float32
import time
from math import sqrt, atan2, asin, degrees, radians, pi, cos, sin
import utm
import random
import math



class Particle_Localization():
  prevPoseX=0
  prevPoseY=0
  prevPoseYaw=0.0
  sens_noise=0.2
  def __init__(self):
    rospy.init_node('Particle_Localization')
    self.imu_msg = Imu()
    self.Step_Counter_msg = Int32()
    self.orientation_gps_msg = Float32()
    self.gps_msg = Odometry()

    pub = rospy.Publisher("/Particle_Localization", Odometry)
    pub_PC = rospy.Publisher("/particleCloud", PoseArray)
    
    rospy.Subscriber('/imu/data', Imu, self.imu_cb)
    rospy.Subscriber('/Step_Counter', Int32, self.Step_Counter_cb)
    rospy.Subscriber('/Orientation_GPS', Float32, self.orientation_gps_cb)
    rospy.Subscriber('/GPS', Odometry, self.gps_cb)

    rate = rospy.Rate(25.0)
    
    rospy.loginfo('Particle_Localization successfully started')
    
    ########  TODO
    # starting point in utm coordinate 
    x,y,n,b=utm.from_latlon(45.19428, 11.30919)
    # lunghezza passo in metri
    lunghezza_passo = 0.74
    prev_step=0

    particles=self.initParticle(x,y)

    P = np.mat(np.diag([0.5]*3))

    while not rospy.is_shutdown():

      step = self.Step_Counter_msg.data
      rot = self.imu_msg.orientation
      pos = self.gps_msg.pose.pose.position

      quaternion = (
    	  rot.x,
    	  rot.y,
    	  rot.z,
    	  rot.w)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      orientation = euler[2] + pi
      orientation %= 2 * pi
      quaternion = tf.transformations.quaternion_from_euler(0, 0, orientation)
      
      # check wether the step counter value increments i.e: we did another step
      if(step>prev_step):
        distance= lunghezza_passo *(step-prev_step)
        
        particles=self.motion(particles, distance, orientation)
        
        x += distance * cos(orientation)
        y += distance * sin(orientation)
      p=Odometry()
      p.header.stamp = rospy.Time.now()
      p.header.frame_id = "/map"
      p.child_frame_id = ""
      p.pose.pose.position.x= x
      p.pose.pose.position.y= y
      p.pose.pose.position.z=0.0
      p.pose.pose.orientation.x=quaternion[0]
      p.pose.pose.orientation.y=quaternion[1]
      p.pose.pose.orientation.z=quaternion[2]
      p.pose.pose.orientation.w=quaternion[3]
      p_cov = np.array([0.0]*36).reshape(6,6)
      p_cov[0:2,0:2] = P[0:2,0:2]
      p.pose.covariance = tuple(p_cov.ravel().tolist())

      pub.publish(p)
      pub_PC.publish(particles)
      prev_step=step

      rate.sleep()

  def imu_cb(self, msg):
    self.imu_msg = msg  
  def Step_Counter_cb(self, msg):
    self.Step_Counter_msg = msg  
  def orientation_gps_cb(self, msg):
    self.orientation_gps_msg = msg  
  def gps_cb(self, msg):
    self.gps_msg = msg 



  def initParticle(self,x,y):

    poseArray = PoseArray()
    poseArray.header.stamp = rospy.Time.now()
    poseArray.header.frame_id = "/map"
    Zorientation=0
    quaternion = (
    	  self.imu_msg.orientation.x,
    	  self.imu_msg.orientation.y,
    	  self.imu_msg.orientation.z,
    	  self.imu_msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta=euler[2]
    for i in range(0, 500):
      Zorientation=(Zorientation + 0.0872665)%(2*math.pi)
      somePose = Pose()
      somePose.position.x = random.uniform(x-10,x+10)#(-33, +33) 
      somePose.position.y = random.uniform(y-10,y+10)#(-59, +59)
      somePose.position.z = 0.0
      
      quaternion = tf.transformations.quaternion_from_euler(0, 0, (random.gauss(theta,0.2)%(2*math.pi)))
      somePose.orientation.x = quaternion[0]
      somePose.orientation.y = quaternion[1]
      somePose.orientation.z = quaternion[2]
      somePose.orientation.w = quaternion[3]

      poseArray.poses.append(somePose)
    return poseArray


  def motion(self,particles, DeltaSpace, Theta):
      # Calculate distance between orientation
      for i in range(len(particles.poses)):
	# Update position
    return particles







if __name__ == '__main__':
  try:
    orient = Particle_Localization()
  except rospy.ROSInterruptException:
    pass
