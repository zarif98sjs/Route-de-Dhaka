# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 17:40:26 2019

@author: User
"""

import pandas as pd
import geopandas
import matplotlib.pyplot as plt
from shapely.geometry import LineString
import networkx as nx
import osmnx as ox

ls = LineString([(0, 0), (1, 1),(1,4)])
print(ls)

G =  nx.MultiDiGraph()
#G.add_node(1)
#G.add_node(2)
G.add_edge(1,2, geometry = ls)
print(G.edges(data=True))
#G.add_edge(3,4, weight = 5)
#G.add_edge(3,4)
print()
print(type(G))

#ox.graph_to_gdfs(G,edges=True)