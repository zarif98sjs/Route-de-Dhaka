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
    
    while True:
        line = f.readline()
        if line == '':
            break
        x = line.split()
        u.append(int(x[0]))
        v.append(int(x[1]))
        weights.append(CAR_FARE* distance_dict[(u[-1],v[-1])])

for i in bus_u_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append(BUS_FARE* bus_u_dict[i][2])

for i in bus_b_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append(BUS_FARE* bus_b_dict[i][2])

for i in metro_dict:
    u.append(i[0])
    v.append(i[1])
    weights.append(METRO_FARE* metro_dict[i][2])


edges = [g.Edge(u[i], v[i], weights[i], True) for i in range(len(u))]
graph= g.Graph(nodes.values(), edges, lat_long,[])


################## WRITE CODE HERE #################################



