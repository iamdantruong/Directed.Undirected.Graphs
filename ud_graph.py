# Course: 
# Author: Dan Truong
# Assignment: 6
# Description: This python file contains an undirected graph ADT its methods


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Adds new vertex to the graph. Does not allow for duplicate vertex keys
        """
        keys= self.adj_list.keys()
        if v not in keys:
            self.adj_list[v]= []
        
    def add_edge(self, u: str, v: str) -> None:
        """
        Takes as a parameter 2 vertices and adds an edge between them.
        Does not allow for a loop edge.
        """
        keys = self.adj_list.keys()
        dict= self.adj_list
        if u==v:
            return
        self.add_vertex(u)
        self.add_vertex(v)
        if u not in dict[v]:
            dict[v].append(u)
        if v not in dict[u]:
            dict[u].append(v)
        

    def remove_edge(self, v: str, u: str) -> None:
        """
        Takes as a paramter 2 vertices and removes the edge
        between them if it exists.
        """
        keys = self.adj_list.keys()
        dict= self.adj_list
        if u in keys and v in keys:
            if v in dict[u]:
                dict[u].remove(v)
            if u in dict[v]:
                dict[v].remove(u)
        

    def remove_vertex(self, v: str) -> None:
        """
        Takes as a parameter a vertex and removes that vertex and its
        associated edges.
        """
        keys = self.adj_list.keys()
        dict= self.adj_list
        if v in keys:
            for vertex in dict[v]:
                dict[vertex].remove(v)
            del dict[v]

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)
        """
        return list(self.adj_list.keys())
       

    def get_edges(self) -> []:
        """
        Returns a list of the edges in the graph.
        """
        edges = []
        keys= self.adj_list.keys()
        dict= self.adj_list
        for key in keys:
            for vertex in dict[key]:
                if (key,vertex) not in edges and (vertex,key) not in edges:
                    edges.append((key,vertex))
        return edges
        

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise
        """
        keys= self.adj_list.keys()
        dict= self.adj_list
        if len(path)==0:
            return True
        else:
            index=0
            vertex = path[index]
            while index< len(path)-1:
                next= path[index+1]
                if vertex not in keys or next not in dict[vertex]:
                    return False
                if next in dict[vertex]:
                    vertex= next
                    index+=1
            if vertex not in keys:
                return False
            return True


    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order.
        """
        visited= []
        stack=[]
        if v_start not in self.adj_list.keys():
            return visited
        return self.rec_dfs(v_start, v_end, visited,stack)

    def rec_dfs(self, v_start, v_end, visited, stack):
        visited.append(v_start)
        if v_start== v_end:
            return visited

        for neighbor in sorted(self.adj_list[v_start]):
            stack.append(neighbor)
        for neighbor in sorted(self.adj_list[v_start]):
            if neighbor== stack[0]:
                stack.pop(0)
            if neighbor not in visited:
                return self.rec_dfs(neighbor, v_end, visited, stack)
        while len(stack)>0:
            if stack[0] in visited:
                stack.pop(0)
            else:
                return self.rec_dfs(stack[0], v_end, visited,stack)

        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order
        """
        visited= []
        queue= []
        if v_start not in self.adj_list.keys():
            return visited

        return self.bfs_helper(v_start, v_end, visited, queue)

    def bfs_helper(self, v_start, v_end, visited, queue):
        """
        Helper method to BFS. Takes a start and end vertex, and a visited list and a queue to
        traverse through the graph breadth first. Returns the list of visited vertices.
        """
        visited.append(v_start)
        queue.append(v_start)

        if v_start==v_end:
            return visited

        while queue:
            next=queue.pop(0)
            for neighbor in sorted(self.adj_list[next]):
                if neighbor not in visited:
                    visited.append(neighbor)
                    queue.append(neighbor)
                    if neighbor== v_end:
                        return visited

        return visited
        

    def count_connected_components(self):
        """
        Return number of connected components in the graph. This
        method utilizes a helper method called counting_helper
        that traverses through the graph.
        """
        keys= self.adj_list.keys()
        visited= dict()
        count=0
        for key in keys:
            visited[key]= False

        for key in visited.keys():
            if visited[key]==False:
                self.counting_helper(visited, key)
                count+=1
        return count

    def counting_helper(self, visited, key):
        """A helper method to count_connected_components that performs
        DFS to navigate the undirected graph."""
        visited[key]=True
        for p in self.adj_list[key]:
            if visited[p] == False:
                self.counting_helper(visited, p)


    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise. This method
        uses a helper method called cycle_helper to traverse through the graph.
        """
        keys= self.adj_list.keys()
        visited= dict()
        for key in keys:
            visited[key]= False

        for key in visited.keys():
            if visited[key] == False:
                if self.cycle_helper(key, visited, -1)== True:
                    return True
        return False

    def cycle_helper(self, v, visited, parent):
        """This method takes as a parameter a vertex, a dictionary of visited vertices, and
        the parent vertex and traverses through the graph. """
        visited[v]= True
        for i in self.adj_list[v]:
            if visited[i]== False:
                if self.cycle_helper(i, visited, v):
                    return True
            elif parent!=i:
                return True
        return False


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
