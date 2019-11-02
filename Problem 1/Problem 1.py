# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 19:31:59 2019

@author: USER
"""

# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 11:46:26 2019

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
import time
import geopandas
import matplotlib.pyplot as plt
from shapely.geometry import LineString
import networkx as nx
import osmnx as ox

        

#################  READ DATA ######################################
# Input
#ata_file = "Dataset/Roadmap-Dhaka.csv"

# Delimiter
#data_file_delimiter = ','
#column_names = list(range(22))

# Read csv
#df = pd.read_csv(data_file, header=None, delimiter=data_file_delimiter, names=column_names)
# print(df)


with open('../Dictionaries/node_dict.p', 'rb') as fp:
    nodes = pickle.load(fp)
with open('../Dictionaries/latlong_dict.p', 'rb') as fp:
    lat_long = pickle.load(fp)
with open('../Dictionaries/middle_nodes_dict.p', 'rb') as fp:
    middle_nodes_dict = pickle.load(fp)
        

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
        weights.append(float(x[2]))
        



############################## UNUSED ALGO ##################################

#
#start = time.time()
#path2 = graph.astar(orig_node,dest_node)
#end = time.time()
#print("Time taken for astar algorithm is : "+str(end-start))
#

    
#start = time.time()
#R = graph.get_connections_weights_as_dictionary()
#orig = orig_node
#dest = dest_node
#
#if end not in R.keys():
#    R[end] = {start:0}
#
#alpha = 0.7 # learning rate
#epsilon = 0.1 #greedy policy
#episodes = 1000
#model = rl.RL(R,100)
#result = model.result(alpha,epsilon,episodes,orig,dest)
#end = time.time()
#print("Time taken for RL algorithm is : "+str(end-start))


#####################################################################



    




edges = [g.Edge(u[i], v[i], weights[i], True) for i in range(len(u))]
graph= g.Graph(nodes.values(), edges, lat_long,[])
#G =  nx.MultiDiGraph()
#for i in range(len(u)):
#    temp_list = []
#    temp_list.append(lat_long[u[i]])
#    if (u[i],v[i]) in middle_nodes_dict:
#        for j in middle_nodes_dict[(u[i],v[i])]:
#            temp_list.append(lat_long[j])
#    
#    temp_list.append(lat_long[v[i]])
#    #print(temp_list)
#    ls = LineString(temp_list)
#    G.add_edge(u[i],v[i],geometry = ls)
    
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

iteration =1
while True:
    print("Enter the origin node (Enter -1 -1 to stop the program) : ")
    org_x,org_y = map(float, input().split())
    if org_x==-1 and org_y ==-1:
        break
    print("Enter the Destination node : ")
    dest_x,dest_y = map(float, input().split())
    
    same_o = False
    same_d = False
    orig_node, same_o = get_nearest(org_x,org_y)
    dest_node, same_d =  get_nearest(dest_x,dest_y)
    #orig_node =1 
    #dest_node =7
    
    #orig_node = 90.393877 23.733190
    #dest_node = 90.383620 23.737934  90.383342 23.737137
    # 90.38333399999999 23.739582000000002
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
    
    if not same_o and not same_d:
        co_ords = [(org_x, org_y)] + co_ords + [(dest_x, dest_y)]
    elif not same_o:
        co_ords = [(org_x, org_y)] + co_ords
    elif not same_d:
        co_ords = co_ords + [(dest_x, dest_y)]
        
        
    path_txt = graph.print_path_info_latlong(co_ords, same_o, same_d)
    ###Path text file e dekhale ekhane likhbo########
    g.write_file('Problem_1_it_'+str(iteration)+'.txt',1,path_txt)
    
    kml = simplekml.Kml()
    for i in range(len(co_ords)-1): 
        line = kml.newlinestring(name=str(co_ords[i]) + " "+ str(co_ords[i+1]),
                                 coords = [co_ords[i],co_ords[i+1]],
                                 description = path_txt[i+2])
        line.style.linestyle.width = 3
        line.style.linestyle.color = simplekml.Color.red
    kml.save('Problem_1_it_'+str(iteration)+'.kml')
    iteration+=1









