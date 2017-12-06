# a simple magnetometer calibration 

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
import matplotlib.pyplot as plt

from flask import Flask 
from flask_sockets import Sockets

app = Flask(__name__) 
sockets = Sockets(app)





fuse = Fusion()
global mag_x
mag_x=[]
global mag_y
mag_y=[]
global mag_z
mag_z=[]
@sockets.route('/<name>') 
def echo_socket(ws, name=""):
	while True:
		message = ws.receive()
		if(name=="magnetometer"):
			magnetometer = [float(i) for i in (message.split(','))]
			global mag_x
			global mag_y
			global mag_z
			mag_x.append(magnetometer[0])
			mag_y.append(magnetometer[1])
			mag_z.append(magnetometer[2])
			print(magnetometer)


                if(len(mag_x)==1000):
			mag_bias_x  = (max(mag_x) + min(mag_x))/2  # get average x axis max chord length in counts
 			mag_bias_y  = (max(mag_y) + min(mag_y))/2  # get average y axis max chord length in counts
 			mag_bias_z  = (max(mag_z) + min(mag_z))/2  # get average z axis max chord length in counts



			mag_scale_x  = (max(mag_x) - min(mag_x))/2  # get average x axis max chord length in counts
 			mag_scale_y  = (max(mag_y) - min(mag_y))/2  # get average y axis max chord length in counts
 			mag_scale_z  = (max(mag_z) - min(mag_z))/2  # get average z axis max chord length in counts
 		

 			avg_rad = mag_scale_x + mag_scale_y + mag_scale_z
 			avg_rad /= 3.0


 			cal_x = avg_rad/(mag_scale_x)
 			cal_y = avg_rad/(mag_scale_y)
 			cal_z = avg_rad/(mag_scale_z)

			plt.subplot(321)
			plt.scatter(mag_x,mag_y,marker="o",)
			plt.subplot(323)
			plt.scatter(mag_y,mag_z,marker="v",)
			plt.subplot(325)
			plt.scatter(mag_x,mag_z,marker="s",)
			plt.subplot(322)
			mag_x_calibrated= [x+mag_bias_x+cal_x for x in mag_x]
			mag_y_calibrated= [y+mag_bias_y+cal_y for y in mag_y]
			mag_z_calibrated= [z+mag_bias_z+cal_z for z in mag_z]
			plt.scatter(mag_x_calibrated,mag_y_calibrated,marker="o",)
			plt.subplot(324)
			plt.scatter(mag_y_calibrated,mag_z_calibrated,marker="v",)
			plt.subplot(326)
			plt.scatter(mag_x_calibrated,mag_z_calibrated,marker="s",)

			plt.show()

			print("CALIBRATION OFFSET VALUE = ", mag_bias_x,mag_bias_y,mag_bias_z)			
			print("CALIBRATION SCALE VALUE = ", cal_x,cal_y,cal_z)
			print("CALIBRATION DONE")
			break

		ws.send(message)






			
		
	
    
if __name__ == '__main__':
  try:
    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler
    server = pywsgi.WSGIServer(('0.0.0.0', 5000), app, handler_class=WebSocketHandler)
    server.serve_forever()
  except rospy.ROSInterruptException:
    pass
