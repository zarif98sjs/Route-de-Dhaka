# -*- coding: utf-8 -*-
"""
Created on Thu Oct 31 14:46:49 2019

@author: User
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Oct 30 14:41:33 2019
@author: Zarif
"""
import heapq
import numpy as np
import networkx as nx
import osmnx as ox

inf = 1e20

HMap = {}
revHMap = {}
conEdge = {}
conPoint = set([])

f = open("asd2.txt","w")

class Distance:
    def __init__(self):
        self.contractId = -1
        self.sourceId = -1
        
        self.forwqueryId = -1
        self.revqueryId = -1
        
        self.distance = inf
        self.revDistance = inf
        self.queryDist = inf
        
class Processed:
    def __init__(self):
        self.forwProcessed = False
        self.revProcessed = False
        
        self.forwqueryId = -1
        self.revqueryId = -1
            
        
class Vertex:
    def __init__(self,vertexNum):
        self.vertexNum = vertexNum
        
        self.inEdges = []
        self.outEdges = []
        self.inECost = []
        self.outECost = []
        
        self.distance = Distance()
        self.processed = Processed()
        
        self.edgeDiff = 0
        self.delNeighbours = 0
        self.shortcutCover = 0
        self.importance = 0
        
        self.contracted = False
        self.orderPos = 0
        
    def __lt__(self,other):
        return self.orderPos>other.orderPos
        
class PreProcess:
    def __init__(self):
        self.PQImp = []
        self.queue = []
        
    def computeImportance1(self,graph):
        #key = lambda x:x.importance
        for i in range(len(graph)):
            graph[i].edgeDiff = (len(graph[i].inEdges)*len(graph[i].outEdges)) - len(graph[i].inEdges) - len(graph[i].outEdges)         
            graph[i].shortcutCover = len(graph[i].inEdges) + len(graph[i].outEdges)
            graph[i].importance = graph[i].edgeDiff*14 + graph[i].shortcutCover*25 + graph[i].delNeighbours*10
            heapq.heappush(self.PQImp,(graph[i].importance,graph[i]))
        
    def computeImportance2(self,graph,vertex):
        vertex.edgeDiff = (len(vertex.inEdges)*len(vertex.outEdges)) - len(vertex.inEdges) - len(vertex.outEdges)         
        vertex.shortcutCover = len(vertex.inEdges) + len(vertex.outEdges)
        vertex.importance = vertex.edgeDiff*14 + vertex.shortcutCover*25 + vertex.delNeighbours*10
        
    def preProcess(self,graph):
        nodeOrdering = np.zeros(shape = len(graph),dtype=np.int64)
        extractNum = 0
        
        while len(self.PQImp)!=0 :
            vertex = heapq.heappop(self.PQImp)[1]
            self.computeImportance2(graph,vertex)
            
            if len(self.PQImp)!=0 and vertex.importance > self.PQImp[0][1].importance:
                heapq.heappush(self.PQImp,(vertex.importance,vertex))
                continue
            
            nodeOrdering[extractNum] = vertex.vertexNum
            vertex.orderPos = extractNum
            extractNum += 1
            
            self.contractNode(graph,vertex,extractNum-1)
            
        return nodeOrdering
    
    def calNeighbours(self,graph,inEdges,outEdges):
        for i in range(len(inEdges)):
            temp = inEdges[i]
            graph[temp].delNeighbours+=1;
            
        for i in range(len(outEdges)):
            temp = outEdges[i]
            graph[temp].delNeighbours+=1;
            
    def contractNode(self,graph,vertex,contractId):
        inEdges = vertex.inEdges
        inECost = vertex.inECost
        outEdges = vertex.outEdges
        outECost = vertex.outECost
        
        vertex.contracted = True
        inMax = 0
        outMax = 0
        
        self.calNeighbours(graph,vertex.inEdges,vertex.outEdges)
        
        for i in range(len(inECost)):
            if graph[inEdges[i]].contracted==True:
                continue
            if inECost[i]>inMax:
                inMax = inECost[i]
        
        for i in range(len(outECost)):
            if graph[outEdges[i]].contracted==True:
                continue
            if outECost[i]>outMax:
                outMax = outECost[i]
                
        maxx = inMax + outMax
        
        for i in range(len(inEdges)):
            inVertex = inEdges[i]
            incost = inECost[i]
            
            if (graph[inVertex].contracted)==True:
                continue
            
            self.dijkstra(graph,inVertex,maxx,contractId,i)
            
            for j in range(len(outEdges)):
                outVertex = outEdges[j]
                outcost = outECost[j]
                
                if graph[outVertex].contracted==True:
                    continue
            
                if graph[outVertex].distance.contractId!=contractId or graph[outVertex].distance.sourceId!=i or (graph[outVertex].distance.distance>(incost+outcost)):
                    print("In : ",revHMap[inVertex] , " , Out : ",revHMap[outVertex] ," , Current : ",revHMap[vertex.vertexNum])
                    stri = "In : "+str(inVertex)+ " , Out : "+str(outVertex)+" , Current : "+str(vertex.vertexNum) +"\n"
                    f.write(stri)
                    inn = inVertex
                    outt = outVertex
                    midd = vertex.vertexNum
                    conEdge[(inn,outt)] = midd
                    conPoint.add(midd)
                    graph[inVertex].outEdges.append(outVertex)
                    graph[inVertex].outECost.append(incost+outcost)
                    graph[outVertex].inEdges.append(inVertex)
                    graph[outVertex].inECost.append(incost+outcost)
        
               
                
    def dijkstra(self,graph,source,maxcost,contractId,sourceId):
        
        self.queue.clear()
        graph[source].distance.distance = 0
        graph[source].distance.contractId = contractId
        graph[source].distance.sourceId = sourceId
        
        heapq.heappush(self.queue,(graph[source].distance.distance,graph[source]))
        #print(queue)
        
        i = 0
        while len(self.queue)!=0:
            vertex = heapq.heappop(self.queue)[1]
            if i>3 or vertex.distance.distance>maxcost :
                return
            self.relaxEdges(graph,vertex.vertexNum,contractId,sourceId)
            
    def relaxEdges(self,graph,vertex,contractId,sourceId):
        vertexList = graph[vertex].outEdges
        costList = graph[vertex].outECost
        
        for i in range(len(vertexList)):
            temp = vertexList[i]
            cost = costList[i]
            
            if graph[temp].contracted==True:
                continue
            
            if self.checkId(graph,vertex,temp)==True or graph[temp].distance.distance > graph[vertex].distance.distance + cost:
                graph[temp].distance.distance = graph[vertex].distance.distance + cost
                graph[temp].distance.contractId = contractId
                graph[temp].distance.sourceId = sourceId
                
                ##need to change the delete into log(n)
                #print(queue)
                #print("-->",graph[temp].vertexNum)
                el = (graph[temp].distance.distance,graph[temp])
                if(el in self.queue):
                    self.queue.remove(el)
                    heapq.heapify(self.queue)
                heapq.heappush(self.queue,(graph[temp].distance.distance,graph[temp]))
                
    def checkId(self,graph,source,target):
        if graph[source].distance.contractId != graph[target].distance.contractId or graph[source].distance.sourceId != graph[target].distance.sourceId:
            return True
        return False
    
    def processing(self,graph):
        self.computeImportance1(graph)
        nodeOrdering = self.preProcess(graph)
        return nodeOrdering
    
class BidirectionalDijkstra:
    def __init__(self):
        self.forwQ = []
        self.revQ = []
        self.parent = {}
        
    def computeDist(self,graph,source,target,queryId,nodeOrdering):
        graph[source].distance.queryDist = 0
        graph[source].distance.forwqueryId = queryId
        graph[source].processed.forwqueryId = queryId
        
        graph[target].distance.revDistance = 0
        graph[target].distance.revqueryId = queryId
        graph[target].processed.revqueryId = queryId
        
        heapq.heappush(self.forwQ,(graph[source].distance.queryDist,graph[source]))
        heapq.heappush(self.revQ,(graph[target].distance.revDistance,graph[target]))
        
        estimate = inf
        
        while len(self.forwQ)!=0 or len(self.revQ)!=0:
            if len(self.forwQ)!=0:
                vertex1 = heapq.heappop(self.forwQ)[1]
                if vertex1.distance.queryDist<=estimate :
                    self.relaxEdges(graph,vertex1.vertexNum,"f",nodeOrdering,queryId)
                if vertex1.processed.revqueryId == queryId and vertex1.processed.revProcessed==True:
                    if (vertex1.distance.queryDist + vertex1.distance.revDistance) < estimate:   
                        estimate = vertex1.distance.queryDist + vertex1.distance.revDistance
                        print(vertex1.distance.queryDist)
                        print(vertex1.distance.revDistance)
            
            if len(self.revQ)!=0:
                vertex2 = heapq.heappop(self.revQ)[1]
                if vertex2.distance.revDistance<=estimate :
                    self.relaxEdges(graph,vertex2.vertexNum,"r",nodeOrdering,queryId)
                if vertex2.processed.forwqueryId == queryId and vertex2.processed.forwProcessed==True:
                    if (vertex2.distance.revDistance + vertex2.distance.queryDist) < estimate:
                        estimate = vertex2.distance.queryDist + vertex2.distance.revDistance
                     
        if estimate==inf:
            return -1
        
        print("Est ----> ",estimate)
        print(self.parent)
        self.find_route(source,target)
        '''
        path = []
        now = target
        while now!=source:
            path.append(revHMap[now])
            ''''''
            cur = revHMap[now]
            temp = revHMap[self.parent[now]]
            el = (cur,temp)
            while el in conEdge:
                path.append(conEdge[el])
                el = (cur,conEdge[el])
            ''''''
            now = self.parent[now]
        path.append(revHMap[source])
        path.reverse()
        
        print("Path ",path)
        '''
        return estimate
    
    def relaxEdges(self,graph,vertex,stri,nodeOrdering,queryId):
        if stri=="f":
           vertexList = graph[vertex].outEdges
           costList = graph[vertex].outECost
           graph[vertex].processed.forwProcessed = True
           graph[vertex].processed.forwqueryId = queryId
           
           for i in range(len(vertexList)):
               temp = vertexList[i]
               #print("Vertex : ",vertex ," Temp : ",temp)
               cost = costList[i]
               
               if graph[vertex].orderPos < graph[temp].orderPos:
                   if graph[vertex].distance.forwqueryId != graph[temp].distance.forwqueryId or (graph[temp].distance.queryDist > (graph[vertex].distance.queryDist + cost)):
                      graph[temp].distance.forwqueryId = graph[vertex].distance.forwqueryId
                      graph[temp].distance.queryDist = graph[vertex].distance.queryDist + cost
                      self.parent[temp] = vertex
                      print(temp,"  -> ",vertex)
                      #print("Here")
                      #print("Dist  :  ",graph[temp].distance.queryDist)
                      el = (graph[temp].distance.queryDist,graph[temp])
                      if (el in self.forwQ):
                          self.forwQ.remove(el)
                          heapq.heapify(self.forwQ)
                      heapq.heappush(self.forwQ,(graph[temp].distance.queryDist,graph[temp]))
               
        else:
            vertexList = graph[vertex].inEdges
            costList = graph[vertex].inECost
            graph[vertex].processed.revProcessed = True
            graph[vertex].processed.revqueryId = queryId
           
            for i in range(len(vertexList)):
               temp = vertexList[i]
               #print("@Vertex : ",vertex ," Temp : ",temp)
               cost = costList[i]
               
               if graph[vertex].orderPos < graph[temp].orderPos:
                   if graph[vertex].distance.revqueryId != graph[temp].distance.revqueryId or (graph[temp].distance.revDistance > (graph[vertex].distance.revDistance + cost)): 
                       graph[temp].distance.revqueryId = graph[vertex].distance.revqueryId
                       graph[temp].distance.revDistance = graph[vertex].distance.revDistance + cost
                       self.parent[vertex] = temp
                       print(vertex,"  -->  ",temp)
                       #print("BackHere")
                       #print("Dist  :  ",graph[temp].distance.revDistance)
                       el = (graph[temp].distance.revDistance,graph[temp])
                       if (el in self.revQ):
                           self.revQ.remove(el)
                           heapq.heapify(self.revQ)
                       heapq.heappush(self.revQ,(graph[temp].distance.revDistance,graph[temp]))


    def uncompress(self,start,end,path):
        #print("aaaaaaaaa")
        el = (start,end)
        print(revHMap[start],revHMap[end])
        if (el in conEdge)==False:
            if (start in conPoint)==False:
                path.append(revHMap[start])
            print("Painai :",revHMap[start],revHMap[end])
            return
        print("-------> ",revHMap[start],revHMap[end])
        #print("yesssss")
        mid = conEdge[el]
        self.uncompress(start,mid,path)
        #print("MId :",mid)
        path.append(revHMap[mid])
        self.uncompress(mid,end,path)

    def find_route(self,source,target):
        #print("conEdge :" ,conEdge)
        path = []
        now = target
        while now!=source:
            #path.append(revHMap[now])
            el = (now,self.parent[now])
            if (el in conEdge)==False:
                path.append(revHMap[now])
                #now = self.parent(now)
            else:
                self.uncompress(now,self.parent[now],path)
            now = self.parent[now]
        #path.append(revHMap[source])
        path.reverse()
        print(path)
        print(len(path))
        

eps = 1e-6

def take_input():
    fp = open("input.txt", "r")
    edge_list = []
    n = 0
    m = 0
    flag = False
    while True:
        line = fp.readline()
        if line == '':
            break
        temp = []
        for x in line.split(' '):
            num = float(x)
            temp.append(num)
        if not flag:
            n = np.int64(temp[0])
            m = np.int64(temp[1])
            flag = True
        else:
            edge_list.append( (np.int64(temp[0]), np.int64(temp[1]), temp[2],int(temp[3])) )

    fp.close()

    return n, m, edge_list

                    
if __name__ == '__main__':
    
    
    #G = ox.graph_from_place('Piedmont, California, USA', network_type='drive')
    #nodes_proj, edges = ox.graph_to_gdfs(G, edges=True)
    #edges = [g.Edge(source[i],dest[i],weights[i]) for i in range(len(source))]
    
    #orig_node = ox.get_nearest_node(G, (37.828903, -122.245846))
    #dest_node = ox.get_nearest_node(G, (37.812303, -122.215006))
    #route = nx.shortest_path(G, orig_node, dest_node, weight='length')
    
    n, m, lis = take_input()
    print(n)
    print(m)
    #print(lis)
     
    #vertex = Vertex()
    graph = []
    
    for i in range(n):
        graph.append(Vertex(i))
        
    
    
    cnt = 0
    
    for i in range(m):
        x = lis[i][0]
        y = lis[i][1]
        cost = lis[i][2]
        type = lis[i][3]   
        if (x in HMap.keys())==False:
            HMap[x] = cnt
            revHMap[cnt] = x
            cnt += 1
        if (y in HMap.keys())==False:
            HMap[y] = cnt
            revHMap[cnt] = y
            cnt += 1
            
        x = HMap[x]
        y = HMap[y]
        
        graph[x].outEdges.append(y)
        graph[x].outECost.append(cost)
        graph[y].inEdges.append(x)
        graph[y].inECost.append(cost)
        
        if type==1:
            graph[y].outEdges.append(x)
            graph[y].outECost.append(cost)
            graph[x].inEdges.append(y)
            graph[x].inECost.append(cost)
          
    #print(HMap)
    #print(revHMap)
    # origin , dest
    #53075602 53035698
        
    process = PreProcess()
    nodeOrdering = process.processing(graph)
        
    bd = BidirectionalDijkstra()
            
    #u = 53075602
    #v = 53035698
    #print(bd.computeDist(graph,HMap[u],HMap[v],i,nodeOrdering))
    
    t = np.int64(input())
        
    for i in range(t):
        u = np.int64(input())
        v = np.int64(input())
        print(bd.computeDist(graph,HMap[u],HMap[v],i,nodeOrdering))