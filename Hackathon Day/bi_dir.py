# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 22:09:18 2019

@author: User
"""

    def bi_dijkstra(self, source,target):
        """
        :param source: takes the source from where dijkstra to be run
        :return: the dictionary of distance from source to all other nodes
                and also the path in a list
        """
        
        res = 1e10
        
        processed = set()
        
        distS = {}
        distT = {}
        
        #visitedS = {}
        #visitedT = {}
        
        parentS = {}
        parentT = {}
        
        priority_queueS = []
        priority_queueT = []
        # initializing dist to inf and visited to false and parent to -1
        for i in self.nodes:
            distS[i] = 1e9
            distT[i] = 1e9
            #visitedS[i] = False
            #visitedT[i] = False
            parentS[i] = -1
            parentT[i] = -1

        distS[source] = 0
        distT[target] = 0
        
        heapq.heappush(priority_queueS, (0, source))
        heapq.heappush(priority_queueT, (0, target))
        while True:
            if(len(priority_queueS)==0 or len(priority_queueT)==0):
                break
            a = priority_queueS[0][1]
            heapq.heappop(priority_queueS)
            for j in self.adj[a]:
                b = j[0]
                w = j[1]
                print(b)
                if distS[a] + w < distS[b]:  ### this is the main comparison
                    parent[b] = a  # To track the path
                    dist[b] = dist[a] + w
                    heapq.heappush(priority_queueS, (distS[b], b))
                    
            if distS[a]+distT[a]<res:
                res = distS[a]+distT[a]
            
            if(a in processed):
                break
            else:
                processed.add(a)
                
            ra =priority_queueT[0][1]
            heapq.heappop(priority_queueT)
            for j in self.adj[ra]:
                rb = j[0]
                rw = j[1]
                print("->",rb)
                if distT[ra] + rw < distT[rb]:
                    parent[ra] = rb
                    distT[rb] = distT[ra] + rw
                    heapq.heappush(priority_queueT,(distT[rb],rb))
            
            if distS[ra]+distT[ra]<res:
                res = distS[ra]+distT[ra]
                
            
            if(ra in processed):
                break
            else:
                processed.add(ra)

        print("BIDIJKSTRA : ",res)