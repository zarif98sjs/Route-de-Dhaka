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
import bi_dir as bd


def write_file(filename, path_list):
    with open(filename,'w+') as f:
        f.write("Problem no : 1\n")
        f.write(path_list[0]+'\n')
        f.write(path_list[1]+'\n')
        for i in range(2,len(path_list)):
            f.write(path_list[i]+'\n')
        


#################  READ DATA ######################################
# Input
data_file = "Dataset/Roadmap-Dhaka.csv"

# Delimiter
data_file_delimiter = ','
column_names = list(range(22))

# Read csv
df = pd.read_csv(data_file, header=None, delimiter=data_file_delimiter, names=column_names)
# print(df)


with open('Dictionaries/node_dict.p', 'rb') as fp:
    nodes = pickle.load(fp)
with open('Dictionaries/latlong_dict.p', 'rb') as fp:
    lat_long = pickle.load(fp)
with open('Dictionaries/middle_nodes_dict.p', 'rb') as fp:
    middle_nodes_dict = pickle.load(fp)
        

with open('edges_no_index.txt') as f:
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
bdi = bd.Graph(nodes.values(), edges, lat_long, [])
'''
G =  nx.MultiDiGraph()
for i in range(len(u)):
    temp_list = []
    temp_list.append(lat_long[u[i]])
    if (u[i],v[i]) in middle_nodes_dict:
        for j in middle_nodes_dict[(u[i],v[i])]:
            temp_list.append(lat_long[j])
    
    temp_list.append(lat_long[v[i]])
    #print(temp_list)
    ls = LineString(temp_list)
    G.add_edge(u[i],v[i],geometry = ls)
'''
    
def get_nearest(lat, long):
    lowest = None
    dist_low = float('inf')
    for i in nodes.values():
        curr = lat_long[i]
        curr_dist = graph.getDistanceFromLatLon(lat,long,curr[0],curr[1])
        if curr_dist < dist_low:
            dist_low = curr_dist
            lowest = i
    return lowest

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
    print(orig_node)
    print(orig_node)
    start = time.time()
    dist, parent = graph.dijkstra(orig_node)
    path = graph.get_path(parent,dest_node)
    #print(path)
    end = time.time()
    print("Time taken for Dijkstra algorithm is : "+str(end-start))
    
    
    start = time.time()
    bdi.bi_dijkstra(orig_node, dest_node)
    
    #print(path)
    end = time.time()
    print("Time taken for Dijkstra algorithm is : "+str(end-start))
    
    
    co_ords = []
    for i in range(len(path)-1):
        (lat,long) = lat_long[path[i]]
        #it += 1
        #print(str(lat)+","+str(long)+","+"0")
        co_ords.append((lat,long))
        if (path[i],path[i+1]) in middle_nodes_dict:
            for j in range(len(middle_nodes_dict[(path[i],path[i+1])])):
                (lat,long) = lat_long[middle_nodes_dict[path[i],path[i+1]][j]]
                #print(str(lat)+","+str(long)+","+"0")
                co_ords.append((lat,long))
               
    (lat,long) = lat_long[path[i+1]] 
    #print(str(lat)+","+str(long)+","+"0")
    co_ords.append((lat,long))
    
    
    # origin 90.383309 23.738697
    # dest 90.363845 23.834136
    path_txt = graph.print_path_info_latlong(co_ords)
    write_file('Problem_1 '+str(iteration) + '.txt',path_txt )
    ###Path text file e dekhale ekhane likhbo########
    
    kml = simplekml.Kml()
    for i in range(len(co_ords)-1): 
        line = kml.newlinestring(name=str(co_ords[i]) + " "+ str(co_ords[i+1]),
                                 coords = [co_ords[i],co_ords[i+1]],
                                 description = path_txt[i+2])
        line.style.linestyle.width = 3
        line.style.linestyle.color = simplekml.Color.red
    kml.save('Problem_1 '+str(iteration)+'.kml')
    iteration+=1









