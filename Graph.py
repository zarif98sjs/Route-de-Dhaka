# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 01:51:53 2019

@author: SHERLOCK
"""
import heapq


class Edge:
    def __init__(self, source, dest, length, bidirectional=False):
        self.source = source
        self.dest = dest
        self.length = length
        self.bidirectional = bidirectional


class Graph:

    def __init__(self, nodes, edges, positions):
        """
        constructor takes in nodes as list of integars
        and edges as lists of Edge class objects
        and also creates an empty dictionary

        Position is needed for A* algorithm
        If not needed can be deleted later.
        """
        self.nodes = nodes
        self.edges = edges
        self.connections = {}  # for RL
        self.connected_weights = {}  # for RL
        self.adj = {}
        self.inDegree = {}
        self.outDegree = {}
        self.positions = {}
        self.adj_weights = {}
        for i in nodes:
            self.inDegree[i] = 0
            self.outDegree[i] = 0
            self.adj[i] = []

        for i in range(len(nodes)):
            self.positions[nodes[i]] = positions[i]

        for i in edges:
            self.add_edge(i.source, i.dest, i.length, i.bidirectional)

    ########################## ADDED BY APURBA ###########################
    def get_vertices(self):
        """ returns the vertices of a graph """
        return list(self.adj.keys())

    def get_edges(self):
        """
        :return: the list of tuples that shows all the edges with weights
        """
        nodes = self.get_vertices()
        ret = []
        for i in nodes:
            for j in self.adj[i]:
                ret.append((i, j[0], j[1]))

        return ret

    def add_edge(self, u, v, w, bidirectional=False):
        """
        :param u: source node
        :param v: destination node
        :param w: weight
        :param bidirectional: If true this will also create an edge from v to u with weight w
        :return: creates an edge from u to v with weight w
        """
        self.adj[u].append((v, w))
        self.adj_weights[(u, v)] = w
        if bidirectional is True:
            self.adj[v].append((u, w))
            self.adj_weights[(v, u)] = w

        self.inDegree[v] += 1
        self.outDegree[u] += 1
        if bidirectional is True:
            self.inDegree[u] += 1
            self.outDegree[v] += 1

    def dijkstra(self, source):
        """
        :param source: takes the source from where dijkstra to be run
        :return: the dictionary of distance from source to all other nodes
                and also the path in a list
        """
        dist = {}
        visited = {}
        parent = {}
        priority_queue = []
        # initializing dist to inf and visited to false and parent to -1
        for i in self.nodes:
            dist[i] = float("inf")
            visited[i] = False
            parent[i] = -1

        dist[source] = 0
        heapq.heappush(priority_queue, (0, source))
        while len(priority_queue) != 0:
            a = priority_queue[0][1]
            heapq.heappop(priority_queue)
            if visited[a] is True:
                continue
            visited[a] = True
            for j in self.adj[a]:
                b = j[0]
                w = j[1]
                if dist[a] + w < dist[b]:  ### this is the main comparison
                    parent[b] = a  # To track the path
                    dist[b] = dist[a] + w
                    heapq.heappush(priority_queue, (dist[b], b))

        return dist, parent

    def get_path(self, parent, cur, path=None):
        """
        Get the path from source to destination grabbed from the parent dictionary of dijkstra
        :param parent: the parent array
        :param cur: current node
        :param path: the returned path list
        :return: a list of the nodes from source to destination
        """
        if path is None:
            path = []
        if parent[cur] == - 1:
            path.append(cur)
            return path
        self.get_path(parent, parent[cur], path)
        path.append(cur)
        return path

    def astar(self, start, end):
        """
        :param start: the start node
        :param end: the end node
        :return: the path from the start to end node
        """

        # first one is f,second one is g , third one is h, fourth one is node number and last one is position
        start_node = (0, 0, 0, start, self.positions[start])
        end_node = (0, 0, 0, end, self.positions[end])
        end_pos = self.positions[end]
        parent = {}
        open_list = []  # this is the heap to maintain least f
        closed_list = {}
        parent[start] = -1
        heapq.heappush(open_list, start_node)

        while len(open_list) != 0:
            current_node = open_list[0]
            current_node_number = current_node[3]
            heapq.heappop(open_list)
            closed_list[current_node_number] = True

            # found the node
            if current_node_number == end:
                print("Path found")
                path = []
                current = current_node[3]
                while current != -1:
                    path.append(current)
                    current = parent[current]
                return path[::-1]  # Return reversed path
            else:
                # Generate children
                children = []
                for i in self.adj[current_node[3]]:  # Adjacent squares

                    # Get node position
                    node_position = self.positions[i[0]]

                    # Create new node
                    new_node = (0, 0, 0, i[0], node_position)

                    # Append
                    children.append(new_node)

                # Loop through children
                for child in children:
                    child_node_number = child[3]
                    child_pos = child[4]
                    if child_node_number in closed_list.keys():
                        continue

                    # Create the f, g, and h values
                    g = current_node[1] + self.adj_weights[(current_node_number, child_node_number)]
                    h = ((child_pos[0] - end_pos[0]) ** 2) + (
                            (child_pos[1] - end_pos[1]) ** 2)
                    f = g + h

                    # Child is already in the open list
                    for open_node in open_list:
                        if child_node_number == open_node[3] and g > open_node[1]:
                            continue

                    # Add the child to the open list
                    # Set parent
                    parent[child_node_number] = current_node_number
                    insert_node = (f, g, h, child_node_number, child_pos)
                    heapq.heappush(open_list, insert_node)

        print("Path not found")
        return None

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
            self.connected_weights[node1] = {node2: weight}

    def get_connections_weights_as_dictionary(self):
        for edge in self.edges:
            self.add_to_weights_dict(edge.source, edge.dest, edge.length)
            if edge.bidirectional:
                self.add_to_weights_dict(edge.dest, edge.source, edge.length)
        return self.connected_weights

    ############################# RL PART END ################################


if __name__ == "__main__":
    nodes = [1, 2, 3, 4, 5]
    positions = [(0, 0), (-5, 0), (-5, 2), (0, 9), (1, 1)]
    source = [1, 1, 1, 4, 4, 3]
    dest = [2, 4, 5, 5, 3, 2]
    weights = [5, 9, 1, 9.05, 8.6, 2]
    ##############################################################################################

    edges = [Edge(source[i], dest[i], weights[i], True) for i in range(len(source))]
    g = Graph(nodes, edges, positions)

    print(g.get_edges())
    print(g.positions)
    dist, parent = g.dijkstra(1)
    print(dist)
    print(parent)
    for i in nodes:
        print("Path from 1 to ", i, ": ", end=" ")
        print(g.get_path(parent, i))

    for i in nodes:
        print(g.astar(1, i))
