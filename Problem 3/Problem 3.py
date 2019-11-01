# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 23:09:39 2019

@author: USER
"""
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



with open('../Dictionaries/node_dict.p', 'rb') as fp:
    nodes = pickle.load(fp)
with open('../Dictionaries/latlong_dict.p', 'rb') as fp:
    lat_long = pickle.load(fp)
with open('../Dictionaries/middle_nodes_dict.p', 'rb') as fp:
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
    
    while True:
        line = f.readline()
        if line == '':
            break
        x = line.split()
        u.append(int(x[0]))
        v.append(int(x[1]))
        weights.append(20* distance_dict[(int(x[0]) , int(x[1]))] )

for i in bus_u_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append(7* bus_u_dict[i][2])

for i in bus_b_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append(7* bus_b_dict[i][2])

for i in metro_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append(5* metro_dict[i][2])
        
edges = [g.Edge(u[i], v[i], weights[i], True) for i in range(len(u))]
graph= g.Graph(nodes.values(), edges, lat_long,[])

def get_nearest(lat, long):
    lowest = None
    dist_low = float('inf')
    for i in nodes.values():
        curr = lat_long[i]
        curr_dist = getDistanceFromLatLon(lat,long,curr[0],curr[1])
        if curr_dist < dist_low:
            dist_low = curr_dist
            lowest = i
    return lowest

def print_path_info2(cst,path):
        sz = len(path)
        text =[]
        text.append("Minimum cost is "+str(cst)+"৳ ")
        text.append("Source:  ("+str(lat_long[path[0]][0])+ ", "+str(lat_long[path[0]][1])+")")
        text.append("Destination:  ("+str(lat_long[path[sz-1]][0])+ ", "+str(lat_long[path[sz-1]][1])+")")
        for i in range(sz-1):
            if i==0:
                if (path[i],path[i+1]) in metro_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Metro from Source "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Uttara Bus from Source "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Bikolpo Bus from Source "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Car from Source ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
            elif i== sz-2:
                if (path[i],path[i+1]) in metro_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Metro from "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Uttara Bus from  "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Bikolpo Bus from  "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Car from ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to Destination ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                
            else:
                if (path[i],path[i+1]) in metro_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Metro from "+str( metro_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( metro_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_u_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Uttara Bus from  "+str( bus_u_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_u_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                elif (path[i],path[i+1]) in bus_b_dict:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ .  Ride Bikolpo Bus from  "+str( bus_b_dict[(path[i],path[i+1])][1][0])+"("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to "+str( bus_b_dict[(path[i],path[i+1])][1][1])+ " ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
                
                else:
                    text.append("Cost: "+str(graph.adj_weights[(path[i],path[i+1])])+"৳ . Ride Car from  ("+str(lat_long[path[i]][0])+", "+str(lat_long[path[i]][1])+") to ("+str(lat_long[path[i+1]][0])+", "+str(lat_long[path[i+1]][1])+")" )
            
        return text

iteration =1
while True:
    print("Enter the origin node (Enter -1 -1 to stop the program) : ")
    org_x,org_y = map(float, input().split())
    if org_x==-1 and org_y ==-1:
        break
    print("Enter the Destination node : ")
    dest_x,dest_y = map(float, input().split())
    
    orig_node = get_nearest(org_x,org_y)
    dest_node =  get_nearest(dest_x,dest_y)
    orig_node = 40058
    dest_node = 26435
    start = time.time()
    dist, parent = graph.dijkstra(orig_node)
    path = graph.get_path(parent,dest_node)
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
        else:
            (lat,long) = lat_long[path[i]]
            co_ords.append((lat,long))

    
    
    
    path_txt = print_path_info2(dist[dest_node],path)
    print(path_txt)
    ###Path text file e dekhale ekhane likhbo########
    
    kml = simplekml.Kml()
    for i in range(len(co_ords)-1): 
        line = kml.newlinestring(name=str(co_ords[i]) + " "+ str(co_ords[i+1]),
                                 coords = [co_ords[i],co_ords[i+1]],
                                 description = path_txt[i+3])
        line.style.linestyle.width = 3
        line.style.linestyle.color = simplekml.Color.red
    kml.save('Problem_3_it_'+str(iteration)+'.kml')
    iteration+=1


