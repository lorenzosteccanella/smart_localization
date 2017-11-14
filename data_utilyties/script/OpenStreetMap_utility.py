import numpy as np
import pygeoj
import folium

coordinates=[]

mapit = None

testfile = pygeoj.load(filepath="../data/map.geojson")

for feature in testfile:

  coordinates.append(feature.geometry.coordinates)

print(coordinates)

for lat_long in coordinates:
  
  mapit = folium.Map( location=[ lat_long[0], lat_long[1] ] )

