# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 11:46:26 2019

@author: USER
"""
import pandas as pd
import numpy as np
import Graph as g
import pickle
import RL.Reinforcement_learning as rl
import numpy as np
import simplekml

# Input
data_file = "Dataset/Roadmap-Dhaka.csv"

# Delimiter
data_file_delimiter = ','

# The max column count a line in the file could have
largest_column_count = 0

## Loop the data lines
#with open(data_file, 'r') as temp_f:
#    # Read the lines
#    lines = temp_f.readlines()
#    largest_column_count = 0
#
#    for l in lines:
#        # Count the column count for the current line
#        column_count = len(l.split(data_file_delimiter)) + 1
#
#        # Set the new most column count
#        largest_column_count = column_count if largest_column_count < column_count else largest_column_count
#
## Close file
#temp_f.close()
#
## Generate column names (will be 0, 1, 2, ..., largest_column_count - 1)
#column_names = [i for i in range(0, largest_column_count)]
column_names = list(range(22))

# Read csv
df = pd.read_csv(data_file, header=None, delimiter=data_file_delimiter, names=column_names)
# print(df)


#################### for storing data #################################
#nodes = {}
#lat_long = {}
#count = 0
#middle_nodes_dict = {}
#
#
#
#with open("edges_no_weight.txt","w+") as f:
#    for i in range(len(df)):
#        if i%100 == 0:
#            print(str(i)+" done ")
#        for j in range(21,0,-1):
#            if not np.isnan(df.iloc[i][j]):
#                break
#        weight = np.around(df.iloc[i][j],6)
#        j = j-2
#        middle_nodes = []
#        for k in range(1,j,2):
#            #print(df.iloc[i][k], df.iloc[i][k+1])
#            (lat,long) = (df.iloc[i][k], df.iloc[i][k+1])
#            if (lat,long) not in nodes:
#                nodes[(lat,long)] = count
#                lat_long[count] = (lat,long)
#                count += 1
#            if k==1:
#                first_node = (lat,long)
#            elif k==(j-1):
#                last_node = (lat,long)
#            else:
#                middle_nodes.append(nodes[(lat,long)])
#        middle_nodes_dict[(nodes[first_node], nodes[last_node])] = middle_nodes
#        
#        f.write(str(nodes[first_node]) + " " + str(nodes[last_node]) + " "
#                 + str(np.around(weight,6)) + "\n")
#        
#    
#            
#
#with open('Dictionaries/node_dict.p', 'wb') as fp:
#    pickle.dump(nodes, fp, protocol=pickle.HIGHEST_PROTOCOL)
#with open('Dictionaries/latlong_dict.p', 'wb') as fp:
#    pickle.dump(lat_long, fp, protocol=pickle.HIGHEST_PROTOCOL)
#with open('Dictionaries/middle_nodes_dict.p', 'wb') as fp:
#    pickle.dump(middle_nodes_dict, fp, protocol=pickle.HIGHEST_PROTOCOL)
#    


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
        
        
import time
edges = [g.Edge(u[i], v[i], weights[i], True) for i in range(len(u))]
graph= g.Graph(nodes.values(), edges, lat_long,[])
orig_node = 899
dest_node = 515
start = time.time()
dist, parent = graph.dijkstra(orig_node)
path = graph.get_path(parent,dest_node)
end = time.time()
print("Time taken for Dijkstra algorithm is : "+str(end-start))




start = time.time()
path2 = graph.astar(orig_node,dest_node)
end = time.time()
print("Time taken for astar algorithm is : "+str(end-start))


    
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

it = 0
co_ords = []
for i in range(len(path)-1):
    (lat,long) = lat_long[path[i]]
    it += 1
    print(str(lat)+","+str(long)+","+"0")
    co_ords.append((lat,long))
    if (path[i],path[i+1]) in middle_nodes_dict:
        for j in range(len(middle_nodes_dict[(path[i],path[i+1])])):
            (lat,long) = lat_long[middle_nodes_dict[path[i],path[i+1]][j]]

            print(str(lat)+","+str(long)+","+"0")
            co_ords.append((lat,long))
            it += 1
(lat,long) = lat_long[path[i+1]] 
print(str(lat)+","+str(long)+","+"0")
co_ords.append((lat,long))
it += 1


path_txt = graph.print_path_info_latlong(co_ords)


kml = simplekml.Kml()
for i in range(len(co_ords)-1): 
    line = kml.newlinestring(name=str(co_ords[i]) + " "+ str(co_ords[i+1]),
                             coords = [co_ords[i],co_ords[i+1]],
                             description = path_txt[i+2])
    line.style.linestyle.width = 3
    line.style.linestyle.color = simplekml.Color.red
kml.save('1st.kml')



#for i in path2:
#    (lat,long) = lat_long[i]
#    print(str(lat)+","+str(long)+","+"0")







