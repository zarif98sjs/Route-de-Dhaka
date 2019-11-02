# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 03:49:14 2019

@author: USER
"""

#import sys, os
#sys.path.append(os.path.abspath(os.path.join('..', 'Graph')))
import sys
sys.path.append('../')
import pandas as pd
import numpy as np
import Graph as g
import pickle
#import RL.Reinforcement_learning as rl
import numpy as np
import simplekml
import math
import time


CAR_FARE = 20
METRO_FARE = 5
BUS_FARE = 7

with open('../Dictionaries/node_dict.p', 'rb') as fp:
    nodes = pickle.load(fp)
with open('../Dictionaries/latlong_dict.p', 'rb') as fp:
    lat_long = pickle.load(fp)
with open('../Dictionaries/middle_nodes_dict.p', 'rb') as fp:
    middle_nodes_dict = pickle.load(fp)
with open('../Dictionaries/metro_dict.p', 'rb') as fp:
    metro_dict = pickle.load(fp)
with open('../Dictionaries/middle_r_dict.p', 'rb') as fp:
    middle_r_dict = pickle.load(fp)
with open('../Dictionaries/bus_u_dict.p', 'rb') as fp:
    bus_u_dict = pickle.load(fp)
with open('../Dictionaries/middle_u_dict.p', 'rb') as fp:
    middle_u_dict = pickle.load(fp)
with open('../Dictionaries/bus_b_dict.p', 'rb') as fp:
    bus_b_dict = pickle.load(fp)
with open('../Dictionaries/middle_b_dict.p', 'rb') as fp:
    middle_b_dict = pickle.load(fp)
with open('../Dictionaries/distance_dict.p', 'rb') as fp:
    distance_dict = pickle.load(fp)
                                         
    

with open('../edges_no_index.txt') as f:
    u = []
    v = []
    weights = []
    typ ={}
    while True:
        line = f.readline()
        if line == '':
            break
        x = line.split()
        u.append(int(x[0]))
        v.append(int(x[1]))
        weights.append( distance_dict[(u[-1],v[-1])])
        typ[(int(x[0]),int(x[1]))] = 1
        typ[(int(x[1]),int(x[0]))] = 1

for i in bus_u_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append( bus_u_dict[i][2])
    typ[(i[0],i[1])] = 2
    typ[(i[1],i[0])] = 2

for i in bus_b_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append( bus_b_dict[i][2])
    typ[(i[0],i[1])] = 2
    typ[(i[1],i[0])] = 2

for i in metro_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append( metro_dict[i][2])
    typ[(i[0],i[1])] = 3
    typ[(i[1],i[0])] = 3



edges = [g.Edge(u[i], v[i], weights[i], True) for i in range(len(u))]
graph= g.Graph(nodes.values(), edges, lat_long,[],typ)


################## WRITE CODE HERE #################################
def get_mod_time(t ):
    if t[1]<=15:
        return 15- t[1]
    if t[1]<=30:
        return 30- t[1]
    if t[1]<=45:
        return 45- t[1]
    if t[1]<=60:
        return 60- t[1]
def add_minute(tt, m):
    x= tt[0]
    y= tt[1]
    x+= math.floor(m/60)
    m= m%60
    y += m
    if(y>=60):
        x+= math.floor(y/60)
        y =y%60
    if(x>=24):
        x %=24
    return (x,y)
def print_path_info2(cst,path,ini_time):
        sz = len(path)
        text =[]
        text.append("Source:  ("+str(lat_long[path[0]][0])+ ", "+str(lat_long[path[0]][1])+")")
        text.append("Destination:  ("+str(lat_long[path[sz-1]][0])+ ", "+str(lat_long[path[sz-1]][1])+")")
        now_time = ini_time
        for i in range(sz-1):
            if i==0:
                if (path[i],path[i+1]) in metro_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(5*graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Metro from Source "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(7*graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Uttara Bus from Source "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(7*graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Bikolpo Bus from Source "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(20*graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Car from Source ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
            elif i== sz-2:
                if (path[i],path[i+1]) in metro_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(5*graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Metro from "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(7*graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Uttara Bus from  "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(7*graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Bikolpo Bus from  "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(20*graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Car from ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                
            else:
                if (path[i],path[i+1]) in metro_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(5*graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Metro from "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(7*graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Uttara Bus from  "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(7*graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Bikolpo Bus from  "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Time : "+str(now_time[0])+" : " +str(now_time[1])+". Cost: "+str(20*graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Car from  ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
            now_time = add_minute(now_time,math.floor(graph.adj_weights[(path[i],path[i+1])]/30*60))
        return text
    
    
    
orig_node = 1
dest_node = 5
ini_time = (13,56)
dist, parent = graph.dijkstra(orig_node, ini_time)
path = graph.get_path(parent,dest_node)
print(path)
print_text = print_path_info2(dist[dest_node],path,ini_time)

