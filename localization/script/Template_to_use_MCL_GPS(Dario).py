#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, PointStamped
import random
import math
import time


class MCL():
  prevPoseX=0
  prevPoseY=0
  prevPoseYaw=0.0
  landmarks  = [[-33.0, -59.0], [-33.0, +59.0], [+33.0, +59.0], [+33.0, -59.0]]
  sense_noise=0.8
  def __init__(self):
    rospy.init_node('MCL')
    self.odom_msg = Odometry()
    self.gps_msg = Odometry()
    self.imu_msg = Odometry()
    rospy.Subscriber('odom', Odometry, self.odom_cb)
    rospy.Subscriber('/GPS', Odometry, self.gps_cb)
    rospy.Subscriber('/odometry/filtered', Odometry, self.imu_cb)
    pub = rospy.Publisher("/particleCloudGPS", PoseArray)
    pub2 = rospy.Publisher("/PointMCLGPS", PointStamped)
    rate = rospy.Rate(80.0)
    rospy.loginfo('MCL node successfully started')
    particles=self.initParticle()
    numeroCicli=0
    while not rospy.is_shutdown():
      #time.sleep(3)
      if(numeroCicli==1):
           particles=self.motion(particles)
           numeroCicli=0
      #self.printParticles(particles,"motion")
      Z=[self.gps_msg.pose.pose.position.x, self.gps_msg.pose.pose.position.y]
      w=self.particles_measurement_prob(Z, particles)
      particles=self.resampling_wheel(particles, w)
      #self.printParticles(particles,"resampling")
      point=self.get_position(particles)
      numeroCicli+=1
      pub.publish(particles)  
      pub2.publish(point)

      rate.sleep()

  def printParticles(self,particles,flag=""):
    for i in range(len(particles.poses)):
      print(flag,"--",i,"of",len(particles.poses),"-----x--->",particles.poses[i].position.x,"-----y--->",particles.poses[i].position.y)

  def initParticle(self):
    poseArray = PoseArray()
    poseArray.header.stamp = rospy.Time.now()
    poseArray.header.frame_id = "/map"
    Zorientation=0
    for i in range(0, 500):
      Zorientation=(Zorientation + 0.0872665)%(2*math.pi)
      somePose = Pose()
      somePose.position.x = random.uniform(-1.5,+1.5)#(-33, +33) 
      somePose.position.y = random.uniform(-1.5,+1.5)#(-59, +59)
      somePose.position.z = 0.0
      
      quaternion = tf.transformations.quaternion_from_euler(0, 0, random.uniform(0,(2.0*math.pi)))
      somePose.orientation.x = quaternion[0]
      somePose.orientation.y = quaternion[1]
      somePose.orientation.z = quaternion[2]
      somePose.orientation.w = quaternion[3]

      poseArray.poses.append(somePose)
    return poseArray
     
  def motion(self,particles):
    pos = self.imu_msg.pose.pose.position
    rot = self.imu_msg.pose.pose.orientation
    quaternion = (
    	rot.x,
    	rot.y,
    	rot.z,
    	rot.w)
    DeltaX=(pos.x-self.prevPoseX)
    DeltaY=(pos.y-self.prevPoseY)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    DeltaYaw= euler[2]-self.prevPoseYaw
    DeltaTrans=math.sqrt(math.pow(DeltaX,2)+math.pow(DeltaY,2))
    DeltaRot1=math.atan2(DeltaY,DeltaX) - self.prevPoseYaw
    DeltaRot2=DeltaYaw-DeltaRot1
    if((DeltaTrans>0.1)):#or(abs(DeltaYaw)>0.1)):
      self.prevPoseX=pos.x
      self.prevPoseY=pos.y
      self.prevPoseYaw=euler[2]
      for i in range(len(particles.poses)):
	quaternion = (
    	  particles.poses[i].orientation.x,
    	  particles.poses[i].orientation.y,
    	  particles.poses[i].orientation.z,
    	  particles.poses[i].orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta=euler[2]
        #print("x--->",particles.poses[i].position.x, "y--->",particles.poses[i].position.y)
        particles.poses[i].position.x=particles.poses[i].position.x+DeltaTrans*math.cos(theta+DeltaRot1)
        particles.poses[i].position.y=particles.poses[i].position.y+DeltaTrans*math.sin(theta+DeltaRot1)
        particles.poses[i].position.x+=random.gauss(0,0.1)
        particles.poses[i].position.y+=random.gauss(0,0.1)
        yaw=(theta+DeltaRot1+DeltaRot2)%(2.0*math.pi)
        value_yaw=random.gauss(yaw,0.2)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, value_yaw)
        particles.poses[i].orientation.x=quaternion[0] 
        particles.poses[i].orientation.y=quaternion[1]
        particles.poses[i].orientation.z=quaternion[2]
        particles.poses[i].orientation.w=quaternion[3]
        
        #print("yaw->",particles.poses[i].orientation.z,"+",DeltaRot1,"+",DeltaRot2)
    return particles

  def Gaussian(self, mu, sigma, x):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))
  
  def particles_measurement_prob(self, measurement, particles):
    w=[]
    for i in range(len(particles.poses)):
      w.append(self.measurement_prob(measurement,particles.poses[i].position.x,particles.poses[i].position.y))
    return w;
    
  def measurement_prob(self, measurement, x, y):
        
    # compute errors
    error_x = measurement[0] - x
    error_y = measurement[1] - y

    # calculate Gaussian
    error = math.exp(- (error_x ** 2) / (self.sense_noise ** 2) / 2.0) /math.sqrt(2.0 * math.pi * (self.sense_noise ** 2))
    error *= math.exp(- (error_y ** 2) / (self.sense_noise ** 2) / 2.0) /math.sqrt(2.0 * math.pi * (self.sense_noise ** 2))
    return error
  
  def resampling_wheel(self, particles, w):
    p3 = PoseArray()
    p3.header.stamp = rospy.Time.now()
    p3.header.frame_id = "/map"
    beta=0.0
    index=int(random.random()*(len(particles.poses)))
    mw=max(w)
    for i in range((len(particles.poses))):
      beta+=random.random()*2.0*mw
      while beta>w[index]:
        beta=beta - w[index]
        index=(index+1)%(len(particles.poses))
      somePose = Pose()
      somePose.position.x = particles.poses[index].position.x
      somePose.position.y = particles.poses[index].position.y
      somePose.position.z = 0.0
      somePose.orientation.x = particles.poses[index].orientation.x
      somePose.orientation.y = particles.poses[index].orientation.y
      somePose.orientation.z = particles.poses[index].orientation.z
      somePose.orientation.w = particles.poses[index].orientation.w
      p3.poses.append(somePose)
    return p3

  def get_position(self, particles):
    x = 0.0
    y = 0.0
    #orientation = 0.0
    for i in range(len(particles.poses)):
        x += particles.poses[i].position.x
        y += particles.poses[i].position.y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        # orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) + p[0].orientation - pi)
    p4=PointStamped()
    p4.header.stamp = rospy.Time.now()
    p4.header.frame_id = "/map"
    p4.point.x=x / len(particles.poses)
    p4.point.y=y / len(particles.poses)
    p4.point.z=0.0
    return p4
 
  def virtualsensorData(self, x, y):
    return [random.gauss(x,self.sense_noise),random.gauss(y,self.sense_noise)]
    
  def imu_cb(self, msg):
    self.imu_msg = msg  
  def gps_cb(self, msg):
    self.gps_msg = msg  
  def odom_cb(self, msg):
    self.odom_msg = msg


if __name__ == '__main__':
  try:
    loc = MCL()
  except rospy.ROSInterruptException:
    pass
