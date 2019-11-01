# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 23:09:39 2019

@author: USER
"""
import pandas as pd
import numpy as np
import Graph as g
import pickle
#import RL.Reinforcement_learning as rl
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



with open('Dictionaries/node_dict.p', 'rb') as fp:
    nodes = pickle.load(fp)
with open('Dictionaries/latlong_dict.p', 'rb') as fp:
    lat_long = pickle.load(fp)
with open('Dictionaries/middle_nodes_dict.p', 'rb') as fp:
    middle_nodes_dict = pickle.load(fp)

#data_file = "Dataset/Roadmap-Dhaka.csv"
#data_file_u = "Dataset/Routemap-UttaraBus.csv"
#data_file_b = "Dataset/Routemap-BikolpoBus.csv"
#data_file_r = "Dataset/Routemap-DhakaMetroRail.csv"
#
#
## Delimiter
#data_file_delimiter = ','
#
##
### Loop the data lines
##with open(data_file_b, 'r') as temp_f:
##    # Read the lines
##    lines = temp_f.readlines()
##    largest_column_count = 0
##
##    for l in lines:
##        # Count the column count for the current line
##        column_count = len(l.split(data_file_delimiter)) + 1
##
##        # Set the new most column count
##        largest_column_count = column_count if largest_column_count < column_count else largest_column_count
##
### Close file
##temp_f.close()
##
### Generate column names (will be 0, 1, 2, ..., largest_column_count - 1)
##column_names = [i for i in range(0, largest_column_count)]
#column_names_u = list(range(96))
#column_names_b = list(range(128))
#column_names_r = list(range(156))
#df_uttara = pd.read_csv(data_file_u, header=None, delimiter=data_file_delimiter, names=column_names_u)
#df_bikolpo = pd.read_csv(data_file_b, header=None, delimiter=data_file_delimiter, names=column_names_b)
#df_metro = pd.read_csv(data_file_r, header=None, delimiter=data_file_delimiter, names=column_names_r)
#
#
#
#
#middle_u_dict = {}
#middle_b_dict = {}
#middle_r_dict = {}
#
#with open('Dictionaries/node_dict.p', 'rb') as fp:
#    nodes = pickle.load(fp)
#with open('Dictionaries/latlong_dict.p', 'rb') as fp:
#    lat_long = pickle.load(fp)
#with open('Dictionaries/middle_nodes_dict.p', 'rb') as fp:
#    middle_nodes_dict = pickle.load(fp)
#
#
################### SAVING DATA ############################
#
#bus_u_dict = {} # (start,end):(index, (Sname,Ename), distance)
#for i in range(len(df_uttara)):
#    val = [i]
#    middle_nodes = []
#    distance = 0
#    (lat,lon) = (df_uttara.iloc[i][1],df_uttara.iloc[i][2])
#    start = (nodes[(lat,lon)])
#    for j in range(3,96,2):
#        if type(df_uttara.iloc[i][j]) == str:
#            try:
#                f = float(df_uttara.iloc[i][j])
#                continue
#            except:
#                (Sname,Ename) = (df_uttara.iloc[i][j],df_uttara.iloc[i][j+1])
#                #print(Sname + " " +str(j))
#                val.append((Sname,Ename))
#                break
#        
#        (lat2,lon2) = (df_uttara.iloc[i][j],df_uttara.iloc[i][j+1])
#        if len(middle_nodes)==0 or middle_nodes[-1] != nodes[(lat2,lon2)]:
#            middle_nodes.append(nodes[(lat2,lon2)])
#        distance += getDistanceFromLatLon(lat,lon,lat2,lon2)
#        lat,lon = lat2,lon2
#    end = (nodes[(lat,lon)])
#    val.append(distance)
#    val.append('Uttara Bus')
#    bus_u_dict[(start,end)] = val
#    middle_u_dict[(start,end)] = middle_nodes[:-1]
#
#with open('Dictionaries/bus_u_dict.p', 'wb') as fp:
#    pickle.dump(bus_u_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)
#with open('Dictionaries/middle_u_dict.p', 'wb') as fp:
#    pickle.dump(middle_u_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)    
#    
#    
#bus_b_dict = {} # (start,end):(index, (Sname,Ename), distance)
#for i in range(len(df_bikolpo)):
#    val = [i]
#    middle_nodes = []
#    distance = 0
#    (lat,lon) = (df_bikolpo.iloc[i][1],df_bikolpo.iloc[i][2])
#    start = (nodes[(lat,lon)])
#    for j in range(3,128,2):
#        if type(df_bikolpo.iloc[i][j]) == str:
#            try:
#                f = float(df_bikolpo.iloc[i][j])
#                continue
#            except:
#                (Sname,Ename) = (df_bikolpo.iloc[i][j],df_bikolpo.iloc[i][j+1])
#                # print(Sname + " " +str(j))
#                val.append((Sname,Ename))
#                break
#        
#        (lat2,lon2) = (df_bikolpo.iloc[i][j],df_bikolpo.iloc[i][j+1])
#        if len(middle_nodes)==0 or middle_nodes[-1] != nodes[(lat2,lon2)]:
#            middle_nodes.append(nodes[(lat2,lon2)])
#        distance += getDistanceFromLatLon(lat,lon,lat2,lon2)
#        lat,lon = lat2,lon2
#    end = (nodes[(lat,lon)])
#    val.append(distance)
#    val.append('Bikolpo Bus')
#    bus_b_dict[(start,end)] = val
#    middle_b_dict[(start,end)] = middle_nodes[:-1]
#
#with open('Dictionaries/bus_b_dict.p', 'wb') as fp:
#    pickle.dump(bus_b_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)
#with open('Dictionaries/middle_b_dict.p', 'wb') as fp:
#    pickle.dump(middle_b_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)
#
#
#metro_dict = {} # (start,end):(index, (Sname,Ename), distance)
#for i in range(len(df_metro)):
#    val = [i]
#    distance = 0
#    middle_nodes = []
#    (lat,lon) = (df_metro.iloc[i][1],df_metro.iloc[i][2])
#    start = (nodes[(lat,lon)])
#    for j in range(3,156,2):
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
#        if len(middle_nodes)==0 or middle_nodes[-1] != nodes[(lat2,lon2)]:
#            middle_nodes.append(nodes[(lat2,lon2)])
#        distance += getDistanceFromLatLon(lat,lon,lat2,lon2)
#        lat,lon = lat2,lon2
#    end = (nodes[(lat,lon)])
#    val.append(distance)
#    metro_dict[(start,end)] = val
#    middle_r_dict[(start,end)] = middle_nodes[:-1]
#
#with open('Dictionaries/metro_dict.p', 'wb') as fp:
#    pickle.dump(metro_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)
#
#with open('Dictionaries/middle_r_dict.p', 'wb') as fp:
#    pickle.dump(middle_r_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)
#######################################################################



with open('Dictionaries/metro_dict.p', 'rb') as fp:
    metro_dict = pickle.load(fp)
with open('Dictionaries/middle_r_dict.p', 'rb') as fp:
    middle_r_dict = pickle.load(fp)
with open('Dictionaries/bus_u_dict.p', 'rb') as fp:
    bus_u_dict = pickle.load(fp)
with open('Dictionaries/middle_u_dict.p', 'rb') as fp:
    middle_u_dict = pickle.load(fp)
with open('Dictionaries/bus_b_dict.p', 'rb') as fp:
    bus_b_dict = pickle.load(fp)
with open('Dictionaries/middle_b_dict.p', 'rb') as fp:
    middle_b_dict = pickle.load(fp)
                                             




























