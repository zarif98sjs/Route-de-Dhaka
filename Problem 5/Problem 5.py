# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 03:49:14 2019

@author: USER
"""

#import sys, os
#sys.path.append(os.path.abspath(os.path.join('..', 'Graph')))
import pandas as pd
import numpy as np
import Graph as g
import pickle
#import RL.Reinforcement_learning as rl
import numpy as np
import simplekml
import math
import time

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
        weights.append( distance_dict[(u[-1],v[-1])] /10)
        typ[(int(x[0]),int(x[1]))] = 1
        typ[(int(x[1]),int(x[0]))] = 1

for i in bus_u_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append( bus_u_dict[i][2]/10)
    typ[(i[0],i[1])] = 2
    typ[(i[1],i[0])] = 2

for i in bus_b_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append( bus_b_dict[i][2]/10)
    typ[(i[0],i[1])] = 2
    typ[(i[1],i[0])] = 2

for i in metro_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append( metro_dict[i][2]/10)
    typ[(i[0],i[1])] = 3
    typ[(i[1],i[0])] = 3



edges = [g.Edge(u[i], v[i], weights[i], True) for i in range(len(u))]
graph= g.Graph(nodes.values(), edges, lat_long,[],typ)


################## WRITE CODE HERE #################################
def get_nearest(lat, long):
    lowest = None
    dist_low = float('inf')
    same = False
    for i in nodes.values():
        curr = lat_long[i]
        curr_dist = graph.getDistanceFromLatLon(lat,long,curr[0],curr[1])
        if curr_dist < dist_low:
            dist_low = curr_dist
            lowest = i
            lowest_lat,lowest_long = curr[0],curr[1]
        if (lowest_lat,lowest_long)==(lat,long):
            same = True
            #print('same')
            break
    return lowest,same


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
    z = tt[2]
    z+= m- math.floor(m)
    m =  m- math.floor(m)
    if(z>=60):
        z-=60
        y+=1
    x+= math.floor(m/60)
    m= m%60
    y += m
    if(y>=60):
        x+= math.floor(y/60)
        y =y%60
    if(x>=24):
        x %=24
    return (x,y,z)


def print_path_info2(cst,path,ini_time,same_o,same_d, org_node, dest_node):
        sz = len(path)
        text =[]
        if not same_o:
            text.append("Source:  ("+str(org_node[0])+ ", "+str(org_node[1])+")")
        else:   
            text.append("Source:  ("+str(lat_long[path[0]][0])+ ", "+str(lat_long[path[0]][1])+")")
        if not same_d:
            text.append("Destination:  ("+str(dest_node[0])+ ", "+str(dest_node[1])+")")
        else:
            text.append("Destination:  ("+str(lat_long[path[sz-1]][0])+ ", "+str(lat_long[path[sz-1]][1])+")")
        now_time = ini_time
        for i in range(sz-1):
            if i==0 and not same_o:
                dis = getDistanceFromLatLon(org_node[0],org_node[1],
                                            lat_long[path[0]][0],lat_long[path[0]][1])
                now_time = add_minute(now_time, (dis/2)*60)
                text.append("Time : "+str(now_time[0])+" : " +str(round(round(now_time[1])))+". Cost: "+
                            str(dis/2)+"Time.  Walk from Source ("+str(org_node[0])+", "+str(org_node[1])+") to "+
                            " ("+str(lat_long[path[0]][0])+", "+str(lat_long[path[0]][1])+")" )           
            
            elif i==0:
                if (path[i],path[i+1]) in metro_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time .  Ride Metro from Source "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time .  Ride Uttara Bus from Source "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time .  Ride Bikolpo Bus from Source "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time . Ride Car from Source ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
            
            elif i== sz-2 and not same_d:
                dis = getDistanceFromLatLon(dest_node[0],dest_node[1],
                                            lat_long[path[-1]][0],lat_long[path[-1]][1])
                now_time = add_minute(now_time, (dis/2)*60)
                text.append("Time : "+str(now_time[0])+" : " +str(round(round(now_time[1])))+". Cost: "+str(dis/60)+"Time.  Walk from ("+str(lat_long[path[-1]][0])+", "+
                            str(lat_long[path[-1]][1])+") to Destination "+
                            " ("+str(dest_node[0])+", "+str(dest_node[1])+")" )           
            
            
            elif i== sz-2:
                if (path[i],path[i+1]) in metro_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time . Ride Metro from "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time .  Ride Uttara Bus from  "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time .  Ride Bikolpo Bus from  "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time . Ride Car from ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                
            else:
                if (path[i],path[i+1]) in metro_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time . Ride Metro from "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time .  Ride Uttara Bus from  "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    now_time = add_minute(now_time ,get_mod_time(now_time ))
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time .  Ride Bikolpo Bus from  "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Time : "+str(now_time[0])+" : " +str(round(now_time[1]))+". Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"time . Ride Car from  ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
            now_time = add_minute(now_time, graph.adj_weights[(path[i],path[i+1])]*60)
        return text
    
iteration =1
while True:
    print("Enter the origin node (Enter -1 -1 to stop the program) : ")
    org_x,org_y = map(float, input().split())
    if org_x==-1 and org_y ==-1:
        break
    print("Enter the Destination node : ")
    dest_x,dest_y = map(float, input().split())
    
    print("Enter Starting time : ")
    when, ampm = input().split()
    hour,minute = map(int,when.split(':'))
    if ampm == "PM":
        hour += 12
    
    same_o = False
    same_d = False
    orig_node, same_o = get_nearest(org_x,org_y)
    dest_node, same_d =  get_nearest(dest_x,dest_y)
    start = time.time()
    ini_time = (hour,minute)
    dist, parent = graph.dijkstra(orig_node, ini_time)
    path = graph.get_path(parent,dest_node)
    #print(path)
    end = time.time()
    print("Time taken for Dijkstra algorithm is : "+str(end-start))
    if len(path) ==1:
        print("Path not Found")
        continue    
    
    co_ords = []
    for i in range(len(path)):
        if i!= (len(path)-1):
            (lat,long) = lat_long[path[i]]
            co_ords.append((lat,long))
            if (path[i],path[i+1]) in middle_nodes_dict:
                for j in range(len(middle_nodes_dict[(path[i],path[i+1])])):
                    (lat,long) = lat_long[middle_nodes_dict[path[i],path[i+1]][j]]
                    co_ords.append((lat,long))
            if (path[i],path[i+1]) in middle_b_dict:
                for j in range(len(middle_b_dict[(path[i],path[i+1])])):
                    (lat,long) = lat_long[middle_b_dict[path[i],path[i+1]][j]]
                    co_ords.append((lat,long))
            if (path[i],path[i+1]) in middle_u_dict:
                for j in range(len(middle_u_dict[(path[i],path[i+1])])):
                    (lat,long) = lat_long[middle_u_dict[path[i],path[i+1]][j]]
                    co_ords.append((lat,long))
            if (path[i],path[i+1]) in middle_r_dict:
                for j in range(len(middle_r_dict[(path[i],path[i+1])])):
                    (lat,long) = lat_long[middle_r_dict[path[i],path[i+1]][j]]
                    co_ords.append((lat,long))
        else:
            (lat,long) = lat_long[path[i]]
            co_ords.append((lat,long))
    
    if not same_o and not same_d:
        co_ords = [(org_x, org_y)] + co_ords + [(dest_x, dest_y)]
    elif not same_o:
        co_ords = [(org_x, org_y)] + co_ords
    elif not same_d:
        co_ords = co_ords + [(dest_x, dest_y)]
        
        
    print_text = print_path_info2(dist[dest_node],path,(ini_time[0],ini_time[1],0),same_o,same_d, (org_x,org_y),
                                  (dest_x,dest_y))
    ###Path text file e dekhale ekhane likhbo########
    g.write_file('Problem_5_it_'+str(iteration)+'.txt',5,print_text)
    
    kml = simplekml.Kml()
    for i in range(len(co_ords)-1): 
        line = kml.newlinestring(name=str(co_ords[i]) + " "+ str(co_ords[i+1]),
                                 coords = [co_ords[i],co_ords[i+1]])
        line.style.linestyle.width = 3
        line.style.linestyle.color = simplekml.Color.red
    kml.save('Problem_5_it_'+str(iteration)+'.kml')
    iteration+=1
    
