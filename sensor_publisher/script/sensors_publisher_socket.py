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

from flask import Flask 
from flask_sockets import Sockets

app = Flask(__name__) 
sockets = Sockets(app)

"""

publish data in a syncronized manner

global pub1
pub1=False
global pub2
pub2=False
global pub3
pub3=False
global pub4
pub4=False

@sockets.route('/<name>') 
def echo_socket(ws, name=""):
	while True:
		message = ws.receive()
		ws.send(message)
		if(name=="accelerometer"):
			print("1")
			global pub1
			pub1 =True        		
		if(name=="gyroscope"):
			print("2")
			global pub2
			pub2 =True
		if(name=="magnetometer"):
			print("3")
			global pub3
			pub3 =True
		if(name=="orientation"):
			print("4")
			global pub4
			pub4 =True
		print(pub1,pub2,pub3,pub4)
		if(pub1==True and pub2==True and pub3==True and pub4==True):
			pub1=False
			pub2=False
			pub3=False
			pub4=False
			print("miao")

"""


# publish data in a unsyncronized manner

global accel
accel = None
global gyro
gyro = None
global mag
magn = None
global ori
ori = None

fuse = Fusion()

imu_pub = rospy.Publisher('imu/data_raw', Imu)
mag_pub = rospy.Publisher('imu/mag', MagneticField)

@sockets.route('/<name>') 
def echo_socket(ws, name=""):
	while True:
		message = ws.receive()
		if(name=="accelerometer"):
			global accel
			accel = [float(i) for i in (message.split(','))]   		
		if(name=="gyroscope"):
			global gyro
			gyro = [float(i) for i in (message.split(','))]
		if(name=="magnetometer"):
			global mag
			mag = [float(i) for i in (message.split(','))]
		if(name=="orientation"):
			global ori
			ori = [float(i) for i in (message.split(','))]
		if(ori is not None and accel is not None and gyro is not None and mag is not None):
			#fuse.update(accel, gyro, mag)
			#quaternion = tf.transformations.quaternion_from_euler(fuse.pitch, fuse.roll, fuse.heading)
			quaternion = tf.transformations.quaternion_from_euler(ori[0], ori[1], ori[2])
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

			imu_pub.publish(imu)
			mag_pub.publish(imu_mag)
		ws.send(message)




			
		
	
    
if __name__ == '__main__':
  try:
    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler
    rospy.init_node('sensor_publisher_socket')
    server = pywsgi.WSGIServer(('0.0.0.0', 5000), app, handler_class=WebSocketHandler)
    server.serve_forever()
  except rospy.ROSInterruptException:
    pass
