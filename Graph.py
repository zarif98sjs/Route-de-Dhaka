# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 01:51:53 2019

@author: USER
"""

class Edge:
    def __init__(self, source, dest, length, bidirectional = False):
        self.source = source
        self.dest = dest
        self.length = length
        self.bidirectional = bidirectional
    
    


class Graph:
    
    def __init__(self, nodes, edges):
        """
        constructor takes in nodes as list of integars
        and edges as lists of Edge class objects 
        """
        self.nodes = nodes
        self.edges = edges
        self.connections = {} # for RL
        self.connected_weights = {} # for RL    
        
        
        
    ############################# RL PART ################################
        
    # helper function 
    def add_to_dict(self, key, value):
        if key in self.connections.keys():
                if value not in self.connections[key]:
                    self.connections[key].append(value)
        else:
            self.connections[key] = [value]
        
    def get_connections_as_dictionary(self):
        for edge in self.edges:
            self.add_to_dict(edge.source, edge.dest)
            if edge.bidirectional:
                self.add_to_dict(edge.dest, edge.source)
        return self.connections
    
    # helper function
    def add_to_weights_dict(self, node1, node2, weight):
        if node1 in self.connected_weights.keys():
            if node2 in self.connected_weights[node1]:
                # always choose the least weight for two same nodes
                if self.connected_weights[node1][node2] > weight: 
                    self.connected_weights[node1][node2] = weight 
            else:
                self.connected_weights[node1][node2] = weight
        else:
            self.connected_weights[node1] = {node2 : weight}
    
    
    def get_connections_weights_as_dictionary(self):
        for edge in self.edges:
            self.add_to_weights_dict(edge.source, edge.dest, edge.length)
            if edge.bidirectional:
                self.add_to_weights_dict(edge.dest, edge.source, edge.length)
        return self.connected_weights
    
    ############################# RL PART END ################################
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        