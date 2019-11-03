# -*- coding: utf-8 -*-
"""
Created on Thu Oct 31 23:14:47 2019

@author: USER
"""

import RL.Reinforcement_learning as rl
import Graph as g
import pandas as pd
import osmnx as ox
import networkx as nx
import simplekml
import pandas as pd
import time






#df = pd.read_csv()

############# USING OSMNX FOR INPUT ######################
G = ox.graph_from_place('Piedmont, California, USA', network_type='drive')
nodes, edges = ox.graph_to_gdfs(G)


orig_node = ox.get_nearest_node(G, (37.828903, -122.245846))
dest_node = ox.get_nearest_node(G, (37.812303, -122.215006))



############## Networkx built in #########################
start = time.time()
route = nx.shortest_path(G, orig_node, dest_node, weight='length')
end = time.time()
print("Time taken for built in Dijsktra algorithm is : "+str(end-start))


############## SETTING UP GRAPH ###########################
nodes = list(G.nodes())
EDGES = [g.Edge(edges.iloc[i]['u'],
              edges.iloc[i]['v'], 
              edges.iloc[i]['length'],
              not edges.iloc[i]['oneway']) for i in range(len(edges))]

graph = g.Graph(nodes,EDGES,[],[])


############### DIJKSTRA ##################################
start = time.time()
dist , parent =  graph.dijkstra(orig_node)
path = graph.get_path(parent,dest_node)
end = time.time()
print("Time taken for Dijkstra algorithm is : "+str(end-start))

    

############### Reinforcement Learning ##################################
start = time.time()
R = graph.get_connections_weights_as_dictionary()
orig = orig_node
dest = dest_node

if end not in R.keys():
    R[end] = {start:0}

alpha = 0.7 # learning rate
epsilon = 0.1 #greedy policy
episodes = 1000
model = rl.RL(R,0)
result = model.result(alpha,epsilon,episodes,orig,dest)
end = time.time()
print("Time taken for RL algorithm is : "+str(end-start))




################### KML ###################################
kml = simplekml.Kml()

a = []
index_list = []
for i in range(len(path)-1):
    edges_u = edges[edges['u']==path[i]]
    index = edges_u[edges_u['v']==path[i+1]].index.tolist()
    if len(index)==0:
        print("Can't find edge for : "+str(path[i])+" "+str(path[i+1]))
    index_list.append(index[0])


edges = pd.DataFrame(edges,index=index_list)
for j in range(len(edges)):
    line = kml.newlinestring(name=str(j),coords = 
                             [(edges.iloc[j].geometry.xy[0][i],edges.iloc[j].geometry.xy[1][i])
                             for i in range(len(edges.iloc[j].geometry.xy[0]))])
    line.style.linestyle.width = 3
    line.style.linestyle.color = simplekml.Color.red
    
    a.append([(edges.iloc[j].geometry.xy[0][i],edges.iloc[j].geometry.xy[1][i])
                             for i in range(len(edges.iloc[j].geometry.xy[0]))])
kml.save('1st.kml')


















