#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, Pose, PointStamped, PoseStamped
import random
import math
import time
from map import Map


class map_publisher():
  def __init__(self):
    rospy.init_node('map_publisher')
    pub = rospy.Publisher("/particleMap", PoseArray)
    rate = rospy.Rate(60.0)
    rospy.loginfo('map_publisher node successfully started')
    
    Mappa = Map()

    Mappa.load_map("/home/lorenzo/smart_localization/src/sensor_publisher/data/map.geojson")

    Mappa.create_vector_map(1)

    particles=self.initParticle_msg(Mappa.vector_map)
    
    #map_path=self.initPath_msg(Mappa.vector_map)
    
    while not rospy.is_shutdown():

      pub.publish(particles)  

      rate.sleep()

  def printParticles(self,particles,flag=""):
    for i in range(len(particles.poses)):
      print(flag,"--",i,"of",len(particles.poses),"-----x--->",particles.poses[i].position.x,"-----y--->",particles.poses[i].position.y)

  def initPath_msg(self, vector_map):
    #path to send the Rviz
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "/map"
    for i in range(len(vector_map)):
      Zorientation=vector_map[i][4]
      somePose = PoseStamped()
      somePose.header.frame_id = "odom"
      somePose.header.stamp=rospy.Time.now()
      somePose.pose.position.x = vector_map[i][0]
      somePose.pose.position.y = vector_map[i][1]
      somePose.pose.position.z = 0.0
      
      quaternion = tf.transformations.quaternion_from_euler(0, 0, Zorientation)
      somePose.pose.orientation.x = quaternion[0]
      somePose.pose.orientation.y = quaternion[1]
      somePose.pose.orientation.z = quaternion[2]
      somePose.pose.orientation.w = quaternion[3]

      path.poses.append(somePose)
    return path
    

  def initParticle_msg(self, vector_map):
    poseArray = PoseArray()
    poseArray.header.stamp = rospy.Time.now()
    poseArray.header.frame_id = "/map"
    for i in range(len(vector_map)):
      Zorientation=vector_map[i][4]
      somePose = Pose()
      somePose.position.x = vector_map[i][0]
      somePose.position.y = vector_map[i][1]
      somePose.position.z = 0.0
      
      quaternion = tf.transformations.quaternion_from_euler(0, 0, Zorientation)
      somePose.orientation.x = quaternion[0]
      somePose.orientation.y = quaternion[1]
      somePose.orientation.z = quaternion[2]
      somePose.orientation.w = quaternion[3]

      poseArray.poses.append(somePose)
    return poseArray
     
  


if __name__ == '__main__':
  try:
    loc = map_publisher()
  except rospy.ROSInterruptException:
    pass
