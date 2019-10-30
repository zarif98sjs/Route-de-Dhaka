# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 03:35:09 2019

@author: USER
"""

import Reinforcement_learning as rl
import Graph as g
import networkx as nx
import osmnx as ox

#nodes = [1,2,3,4,5,6]
#source =  [1,1,1,2,2,2,3,3,3,4,4,4,5,5,5,5,6,6]
#dest =    [2,3,4,1,3,5,1,2,5,1,5,6,2,3,4,6,4,5]
#weights = [3,4,2,3,4,2,4,4,6,2,1,4,2,6,1,2,4,2]


### TODO: For case of unidirection graphs, just point from
# end to start using 0 as weights. shouldnt change
#nodes = [1,2,3,4]
#source =  [1,1,2,3,4] 
#dest =    [2,3,4,4,1]
#weights = [3,5,7,3,0] 




G = ox.graph_from_place('Piedmont, California, USA', network_type='drive')
nodes_proj, edges = ox.graph_to_gdfs(G, edges=True)
#edges = [g.Edge(source[i],dest[i],weights[i]) for i in range(len(source))]

orig_node = ox.get_nearest_node(G, (37.828903, -122.245846))
dest_node = ox.get_nearest_node(G, (37.812303, -122.215006))

nodes = list(G.nodes())
edges = [g.Edge(edges.iloc[i]['u'],
                  edges.iloc[i]['v'], 
                  edges.iloc[i]['length'],
                  not edges.iloc[i]['oneway']) for i in range(len(edges))]
    
R = g.Graph(nodes,edges).get_connections_weights_as_dictionary()
#route = nx.shortest_path(G, orig_node, dest_node, weight='length')
start = orig_node
end = dest_node

(start, end) = (end, start)

alpha = 0.7 # learning rate
epsilon = 0.1 #greedy policy
episodes = 1000
model = rl.RL(R,0)
result = model.result(alpha,epsilon,episodes,start,end)
ox.plot_graph_route(G, route, node_size=0)
ox.plot_graph_route(G, result['path'], node_size=0)


