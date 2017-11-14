import numpy as np
import pygeoj
import folium
from scipy.interpolate import interp1d
import math
import utm
import matplotlib.pyplot as plt

class Map:
    
    def __init__(self):
        self.map = []
        self.testfile= []
        self.vector_map= []
        
    def load_map(self, path):
        
        testfile = pygeoj.load(filepath=path)
        
        for feature in testfile:
            self.map.append(feature.geometry.coordinates)
                
        return self.map
    
    def harvesine_distance_lat_long(self,lat0,lon0,lat1,lon1):
        R = 6371 # raggio terra in metri
        lat0rad=math.radians(lat0)
        lat1rad=math.radians(lat1)
        DeltaLat=math.radians(lat1-lat0)
        DeltaLon=math.radians(lon1-lon0)
        a= (math.sin(DeltaLat/2) * math.sin(DeltaLat/2)) + (math.cos(lat0rad) * math.cos(lat1rad) * math.sin(DeltaLon/2)*math.sin(DeltaLon/2))
        c= 2 * math.atan2(math.sqrt(a),math.sqrt(1-a))
        dist= (R * c)*1000
        dist= round(dist,2)
        
        return dist
    
    def distance_utm_lat_long(self,lat0,lon0,lat1,lon1):
        x0,y0,n,b = utm.from_latlon(lat0,lon0)
        x1,y1,n,b = utm.from_latlon(lat1,lon1)
        
        dist = math.sqrt((pow(x1-x0,2)+pow(y1-y0,2)))
        return dist
    
    def distance_two_points(self,x0,y0,x1,y1):
        dist=math.sqrt((pow(x1-x0,2)+pow(y1-y0,2)))
        return dist
    
    def theta_two_points(self,x0,y0,x1,y1):
        theta = math.atan2(y1 - y0, x1 - x0);
        if (theta < 0.0):
            theta += 2*math.pi
        return theta
        
    def create_vector_map(self, meter_step):
        
        for element in self.map:
            #print(element)
            for i in range(1,len(element)):
                lat0= element[i-1][1]
                lon0= element[i-1][0]
                lat1= element[i][1]
                lon1= element[i][0]
                x0,y0,n,b=utm.from_latlon(lat0,lon0)
                #self.vector_map.append((x0,y0,n,b,))
                x1,y1,n,b=utm.from_latlon(lat1,lon1)
                #dist_harvesine=self.harvesine_distance_lat_long(lat0,lon0,lat1,lon1)  
                dist_utm=self.distance_utm_lat_long(lat0,lon0,lat1,lon1)
                #print(utm.from_latlon(lat0,lon0),utm.from_latlon(lat1,lon1),dist_harvesine, dist_utm)
                theta = self.theta_two_points(x0,y0,x1,y1)
                if (theta < 0.0):
                    theta += 2*math.pi
                self.vector_map.append((x0,y0,n,b,theta))
		
                x_prev=x0
                y_prev=y0
                for i in range(int(dist_utm)):
                    new_x = x_prev + meter_step * math.cos(theta)
                    new_y = y_prev + meter_step * math.sin(theta)
                    #print(x_prev,y_prev,new_x,new_y, self.distance_two_points(x_prev,y_prev,new_x,new_y), self.theta_two_points(x_prev,y_prev,new_x,new_y))
                    self.vector_map.append((new_x,new_y,n,b,self.theta_two_points(x_prev,y_prev,new_x,new_y)))
                    x_prev=new_x
                    y_prev=new_y
    
    
    def plot_utm(self):
        x=[]
        y=[]
        
        for element in self.vector_map:
            x.append(element[0])
            y.append(element[1])
            #print(element[0],element[1])
        
        plt.plot(y, x, 'ro')
        plt.axis([min(y), max(y), min(x), max(x)])
        plt.show()
        
        return
        
                
    
        

    def output_html_map(self, fileName):
        
        mapit = folium.Map( location=[45.19338, 11.31001], zoom_start=22 )
        
        for element in self.map:
            for coord in element:
                folium.Marker(([coord[1],coord[0]])).add_to( mapit )
            
        mapit.save( fileName )
        
        return mapit
    
    def output_html_utm_map(self, fileName):
        
        mapit = folium.Map( location=[45.19338, 11.31001], zoom_start=22 )
        for element in self.vector_map:
                
                coord= utm.to_latlon(element[0],element[1],element[2],element[3])
                folium.Marker(([coord[0],coord[1]])).add_to( mapit )
            
        mapit.save( fileName )
        
        return mapit
