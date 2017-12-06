#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
import time
from math import sqrt, atan2, asin, degrees, radians, pi, cos, sin
import utm




class Step_Counter_to_position():

  def __init__(self):
    rospy.init_node('Step_Counter_to_position')
    self.imu_msg = Imu()
    self.Step_Counter_msg = Int32()

    pub = rospy.Publisher("/Step_Counter_to_position", Odometry)

    rospy.Subscriber('/imu/data', Imu, self.imu_cb)
    rospy.Subscriber('/Step_Counter', Int32, self.Step_Counter_cb)

    rate = rospy.Rate(25.0)
    rospy.loginfo('Step_Counter_to_position successfully started')

    P = np.mat(np.diag([0.5]*3))
    
    ########  TODO
    # starting point in utm coordinate 
    x,y,n,b=utm.from_latlon(45.19428, 11.30919)
    # lunghezza passo in metri
    lunghezza_passo = 0.74
    prev_step=0
   

    while not rospy.is_shutdown():

      step = self.Step_Counter_msg.data
      rot = self.imu_msg.orientation
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
      prev_step=step

      rate.sleep()

  def imu_cb(self, msg):
    self.imu_msg = msg  
  def Step_Counter_cb(self, msg):
    self.Step_Counter_msg = msg  








if __name__ == '__main__':
  try:
    orient = Step_Counter_to_position()
  except rospy.ROSInterruptException:
    pass
