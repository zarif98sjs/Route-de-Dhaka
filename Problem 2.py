# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 11:46:26 2019

@author: USER
"""
import pandas as pd
import numpy as np
import Graph as g
import pickle
import RL.Reinforcement_learning as rl
import numpy as np
import simplekml
import math

def deg2rad(deg):
        return deg * (math.pi / 180)


def getDistanceFromLatLon(lat1, lon1, lat2, lon2):
        R = 6371
        dLat = deg2rad(lat2 - lat1)
        dLon = deg2rad(lon2 - lon1)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * math.sin(dLon / 2) * math.sin(dLon / 2)
    
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = R * c
        return d



# Input
data_file = "Dataset/Roadmap-Dhaka.csv"
data_file = "Dataset/Routemap-DhakaMetroRail.csv"


# Delimiter
data_file_delimiter = ','
#column_names = list(range(22))
#
## Loop the data lines
#with open(data_file, 'r') as temp_f:
#    # Read the lines
#    lines = temp_f.readlines()
#    largest_column_count = 0
#
#    for l in lines:
#        # Count the column count for the current line
#        column_count = len(l.split(data_file_delimiter)) + 1
#
#        # Set the new most column count
#        largest_column_count = column_count if largest_column_count < column_count else largest_column_count
#
## Close file
#temp_f.close()
#
## Generate column names (will be 0, 1, 2, ..., largest_column_count - 1)
#column_names = [i for i in range(0, largest_column_count)]
column_names = list(range(156))
df_metro = pd.read_csv(data_file, header=None, delimiter=data_file_delimiter, names=column_names)

with open('Dictionaries/node_dict.p', 'rb') as fp:
    nodes = pickle.load(fp)
with open('Dictionaries/latlong_dict.p', 'rb') as fp:
    lat_long = pickle.load(fp)
with open('Dictionaries/middle_nodes_dict.p', 'rb') as fp:
    middle_nodes_dict = pickle.load(fp)


################## SAVING DATA ############################

#metro_dict = {} # (start,end):(index, (Sname,Ename), distance)
#for i in range(len(df_metro)):
#    val = [i]
#    distance = 0
#    (lat,lon) = (df_metro.iloc[i][1],df_metro.iloc[i][2])
#    start = (nodes[(lat,lon)])
#    for j in range(3,largest_column_count,2):
#        if type(df_metro.iloc[i][j]) == str:
#            try:
#                f = float(df_metro.iloc[i][j])
#                continue
#            except:
#                (Sname,Ename) = (df_metro.iloc[i][j],df_metro.iloc[i][j+1])
#                #print(Sname + " " +str(j))
#                val.append((Sname,Ename))
#                break
#        
#        (lat2,lon2) = (df_metro.iloc[i][j],df_metro.iloc[i][j+1])
#        distance += getDistanceFromLatLon(lat,lon,lat2,lon2)
#        lat,lon = lat2,lon2
#    end = (nodes[(lat,lon)])
#    val.append(distance)
#    metro_dict[(start,end)] = val
#
#with open('Dictionaries/metro_dict.p', 'wb') as fp:
#    pickle.dump(metro_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)


#######################################################################

with open('Dictionaries/metro_dict.p', 'rb') as fp:
    metro_dict = pickle.load(fp)
        



########################## SAVING DISTANCE ############################

#with open('edges_no_index.txt') as f:
#    u = []
#    v = []
#    weights = []
#    while True:
#        line = f.readline()
#        if line == '':
#            break
#        x = line.split()
#        u.append(int(x[0]))
#        v.append(int(x[1]))
#        weights.append(float(x[2]))
#        
#distance_dict = {}
#for i in range(len(u)):
#    distance = 0
#    start = lat_long[u[i]]
#    for j in range(len(middle_nodes_dict[(u[i],v[i])])):
#        end = lat_long[middle_nodes_dict[(u[i],v[i])][j]]
#        distance += getDistanceFromLatLon(start[0], start[1], end[0], end[1])
#        start = end
#    end = lat_long[v[i]]
#    distance += getDistanceFromLatLon(start[0], start[1], end[0], end[1])
#    distance_dict[(u[i], v[i])] = distance
#    
#
#with open('Dictionaries/distance_dict.p', 'wb') as fp:
#    pickle.dump(distance_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)
#
#

#######################################################################

with open('Dictionaries/distance_dict.p', 'rb') as fp:
    distance_dict = pickle.load(fp)
        
























