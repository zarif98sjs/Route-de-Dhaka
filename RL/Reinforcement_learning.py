# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 02:40:10 2019

@author: USER
"""
import copy
import Q_learning as ql


##### Here the input Graph has to be bidirectional otherwise wont work ############
##### Find case which wont work for this scenario ################################


class RL:
    def __init__(self,Rewards,initial_Q_val=100):
        self.R = Rewards
        self.Q = copy.deepcopy(Rewards)
        for source in self.Q.keys():
            for dest in self.Q[source].keys():
                self.Q[source][dest] = initial_Q_val
                
                
    def get_node_with_minQ(self, q):
        min_val = min(q.values())
        for key in q.keys():
            if q[key] == min_val:
                return key
        
    def best_path_and_cost(self, start, end):
        path = [start]
        cost = []
        node = start
        while node != end:  # TODO : find a way to exit code when end doesn't exist
            temp = self.get_node_with_minQ(self.Q[node])
            if temp in path: # a cycle shouldnt happen in a shortest path
                print("No path detected")
                break
            path.append(temp)
            cost.append(float(self.R[node][temp]))
            node = temp
        return (path, cost)
    
    
    def result(self, alpha, epsilon, episodes, start, end):
        self.Q = ql.Q(self.R, self.Q, alpha, epsilon, episodes).Q_Routing(start,end)
        (path,cost) = self.best_path_and_cost(start,end)
        return{
                "path":path,
                "cost":cost,
                "total_cost":sum(cost),
                "Q_values":self.Q
                }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
            
            
            
        
        