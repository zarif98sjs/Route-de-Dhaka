# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 21:59:26 2019

@author: USER
"""

import pandas as pd
import numpy as np
import Graph as g
import numpy as np
import simplekml

# Input
data_file = "Dataset/Roadmap-Dhaka.csv"

# Delimiter
data_file_delimiter = ','

# The max column count a line in the file could have
largest_column_count = 0
column_names = list(range(22))

# Read csv
df = pd.read_csv(data_file, header=None, delimiter=data_file_delimiter, names=column_names)

nodes = []

for i in range(len(df)):
    nodes.append([])
    if i%100 == 0:
        print(str(i)+" done ")
    for j in range(21,0,-1):
        if not np.isnan(df.iloc[i][j]):
            break
    weight = np.around(df.iloc[i][j],6)
    j = j-2
    for k in range(1,j,2):
        #print(df.iloc[i][k], df.iloc[i][k+1])
        (lat,long) = (df.iloc[i][k], df.iloc[i][k+1])
        nodes[i].append((lat,long))
        
        
kml = simplekml.Kml()
for i in range(len(nodes)-20000): 
    if i%100 == 0:
        print(str(i)+" done ")
    
    line = kml.newlinestring(name=str(i),
                             coords = nodes[i])
    line.style.linestyle.width = 3
    line.style.linestyle.color = simplekml.Color.red
kml.save('all_edges.kml')

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
