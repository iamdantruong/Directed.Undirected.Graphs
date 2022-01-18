# Course: CS261 - Data Structures
# Author: Dan Truong
# Assignment: 6
# Description: This python file contains the directed graph ADT and its methods.

class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        This function adds a vertex to the graph. The vertex 'key' is its index
        in the list.
        """
        self.adj_matrix.append([])
        self.v_count+=1
        for num in range(0, self.v_count):
            self.adj_matrix[self.v_count-1].append(0)
        for num in range(0, self.v_count-1):
            self.adj_matrix[num].append(0)

        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        This method takes as parameters a source and destination vertices and a weight
        and updates the graphs to add an edge between the two vertices.
        """
        if src< self.v_count and dst < self.v_count and src!=dst:
            self.adj_matrix[src][dst]=weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        This method takes as parameters a source and destination vertices and
        removes the edge between the two.
        """
        if src>= self.v_count or dst >= self.v_count or src<0 or dst< 0:
            return
        if self.adj_matrix[src][dst]!=0:
            self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        This method returns a list of all vertices in the graph.
        """
        return list(num for num in range(0, self.v_count))

    def get_edges(self) -> []:
        """
        This method returns a list of all the edges in the graph.
        """
        edges= []
        for x in range(0, self.v_count):
            for y in range(0, self.v_count):
                if self.adj_matrix[x][y]!=0:
                    edges.append((x,y,self.adj_matrix[x][y]))
        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        This method takes as a parameter a list of vertices and traverses through the list
        in that order. Returns True if the list is a valid path and False otherwise.
        """
        matrix= self.adj_matrix
        if len(path)==0:
            return True
        else:
            index=0
            vertex = path[index]
            while index< len(path)-1:
                next= path[index+1]
                if matrix[vertex][next] == 0:
                    return False
                else:
                    vertex= next
                    index+=1
            return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        This method takes as parameters a start and end vertex and performs a
        depth first search between the two. It returns a list of the order of
        vertices it traverses through. The end vertex is optional and the function
        stops early if it reaches the end vertex.
        """
        matrix= self.adj_matrix
        visited= []
        visited_boo= [False for i in range(self.v_count)]
        stack=[]
        stack.append(v_start)
        while (len(stack)):
            s = stack[-1]
            stack.pop()
            if visited_boo[s]!= True:
                visited.append(s)
                visited_boo[s]= True
                if s == v_end:
                    break
            for i in range(len(matrix[s])-1,-1,-1):
                next= matrix[s][i]
                if next!=0 and visited_boo[i]!= True:
                    stack.append(i)
        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        This method takes as parameters a start and end vertex and performs a
        breadth first search between the two. This method utilizes a
        helper method and passes it the two vertices, a list of visited vertices, and
        a queue. The end vertex is optional and the function
        stops early if it reaches the end vertex.
        """
        visited = []
        queue = []

        return self.bfs_helper(v_start, v_end, visited, queue)

    def bfs_helper(self, v_start, v_end, visited, queue):
        """
        This is a helper method to bfs. It takes as a parameter a start/end vertex,
        a list of visited vertices, and a queue to traverse through the graph.
        It returns a list of the order of vertices it traverses through.
        """
        matrix= self.adj_matrix
        visited.append(v_start)
        queue.append(v_start)
        neighbors=[]

        if v_start == v_end:
            return visited

        while queue:
            next = queue.pop(0)
            for num in range(0, len(matrix[next])):
                if matrix[next][num]!=0:
                    neighbors.append(num)
            for neighbor in neighbors :
                if neighbor not in visited:
                    visited.append(neighbor)
                    queue.append(neighbor)
                    if neighbor == v_end:
                        return visited
        return visited

    def has_cycle(self):
        """
        This method traverses through the graph and uses a helper method
        to determine if the graph has a cycle. Returns True if it contains
        a cycle and False otherwise. It passes a list of visited vertices, a stack,
        and a vertex to the helper method cycle_helper.
        """
        visited = [False]*(self.v_count+1)
        stack = [False]*(self.v_count+1)

        for v in range(self.v_count):
            if visited[v]== False:
                if self.cycle_helper(v, visited, stack)== True:
                    return True
        return False

    def cycle_helper(self, v, visited, stack):
        """This helper method traverses through the list and determines if
        the graph contains a cycle. It marks the vertex it visits as visited
        then visits the neighbors and recursively searches if its visited
        the same node before to see if there is a cycle."""
        visited[v] = True
        stack[v] = True
        matrix= self.adj_matrix
        neighbors = []
        for num in range(0, self.v_count):
            if matrix[v][num]!=0:
                neighbors.append(num)

        for neighbor in neighbors:
            if visited[neighbor] == False:
                if self.cycle_helper(neighbor, visited, stack):
                    return True
            elif stack[neighbor] == True:
                return True

        stack[v]= False
        return False

    def dijkstra(self, src: int) -> []:
        """
        This method takes as a parameter a starting vertex and performs
        a dijkstra algorithm for finding the shortest path toeach node.
        This method returns a list of the distances to each node in the vertex.
        If a vertex is not reachable, the distance is infinity. This method
        uses a helper methodd called find_min to find the next vertex that has
        a minimum value.
        """
        shortest_path= [False]*self.v_count
        matrix= self.adj_matrix
        dist= [float('inf')]*self.v_count
        dist[src] = 0
        for num in range(self.v_count):
            u = self.find_min(dist, shortest_path)
            shortest_path[u]= True

            for v in range(self.v_count):
                if matrix[u][v]>0 and shortest_path[v]== False and dist[v] > dist[u] + matrix[u][v]:
                    dist[v]= dist[u]+ matrix[u][v]

        return dist

    def find_min(self, dist, shortest_path):
        """This is a helper method that takes as a parameter the list of
        distances and the shortest_path list to find the index of the
        next vertex with the shortest distance."""
        min= float('inf')
        min_index= 0
        for v in range(self.v_count):
            if dist[v] < min and shortest_path[v]== False:
                min= dist[v]
                min_index= v
        return min_index

if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
