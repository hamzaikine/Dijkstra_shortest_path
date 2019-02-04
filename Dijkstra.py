import sys

class Vertex:

    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        self.previous = None
        self.visited = False
        self.distance = sys.maxsize

    def set_adjacent_node(self, neighbor, distance = 0):
        self.adjacent[neighbor] = distance

    def get_adjacent_nodes(self):
        return self.adjacent.keys()


    def set_previous(self, prev):
        self.previous = prev

    def get_id(self):
        return self.id

    def get_neighbor_distance(self, to):
        return self.adjacent[to]

    def set_distance(self, dist):
         self.distance = dist

    def get_distance(self):
         return self.distance

    def set_visited(self):
        self.visited = True

    def get_visited(self):
        return self.visited


    def __str__(self):
        return str(self.id) + ' adjacent nodes: ' + str([x.id for x in self.adjacent])





class Graph():

    def __init__(self):
        self.vertices_nodes = {}
        self.num_vertices = 0
        self.previous = None

    def __iter__(self):
        return iter(self.vertices_nodes.keys())

    def __str__(self):
        return str(list(self.vertices_nodes))

    def add_vertex(self, node):
        vertex = Vertex(node)
        self.vertices_nodes[node] = vertex
        return vertex

    def add_edge(self, frm, to, dist):
         if frm not in self.vertices_nodes:
               self.add_vertex(frm)

         if to not in self.vertices_nodes:
               self.add_vertex(to)

         self.vertices_nodes[frm].set_adjacent_node(self.vertices_nodes[to], dist)
         self.vertices_nodes[to].set_adjacent_node(self.vertices_nodes[frm], dist)

    def get_vertex(self,v):
        if v in self.vertices_nodes:
            return self.vertices_nodes[v]
        else:
            return None

    def get_vertices(self):
        return self.vertices_nodes.keys()


    # def set_pevious(self, current):
    #     self.previous = current
    #
    # def get_previous(self):
    #     return self.previous


def shortest(v, path):
        if v.previous:
            path.append(v.previous.get_id())
            shortest(v.previous, path)

        return





import heapq


def dijkstra(aGraph, start, target):
    #print('''Dijkstra's shortest path''')
    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(), v.get_id()) for v in aGraph.vertices_nodes.values()]
    print(unvisited_queue)
    heapq.heapify(unvisited_queue)

    print(len(unvisited_queue))
    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        # current is node id, need to get the vertex
        g.get_vertex(current).set_visited()

        # for next in v.adjacent:
        for next in g.get_vertex(current).adjacent:

            # if visited, skip
            if next.visited:
                continue
            new_dist = g.get_vertex(current).get_distance() + g.get_vertex(current).get_neighbor_distance(next)


            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(g.get_vertex(current))
                print('updated : current = %s next = %s new_dist = %s' \
                % (g.get_vertex(current).get_id(), next.get_id(), next.get_distance()))
            else:
                print('not updated : current = %s next = %s new_dist = %s' \
                % (g.get_vertex(current).get_id(), next.get_id(), next.get_distance()))

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(), v.get_id()) for v in aGraph.vertices_nodes.values() if not v.get_visited()]
        heapq.heapify(unvisited_queue)


if __name__ == '__main__':

    g = Graph()

    g.add_vertex('a')
    g.add_vertex('b')
    g.add_vertex('c')
    g.add_vertex('d')
    g.add_vertex('e')
    g.add_vertex('f')

    g.add_edge('a', 'b', 7)
    g.add_edge('a', 'c', 9)
    g.add_edge('a', 'f', 14)
    g.add_edge('b', 'c', 10)
    g.add_edge('b', 'd', 15)
    g.add_edge('c', 'd', 11)
    g.add_edge('c', 'f', 2)
    g.add_edge('d', 'e', 6)
    g.add_edge('e', 'f', 9)

    print('Graph data:')
    for v in g.vertices_nodes.values():
        for w in v.get_adjacent_nodes():
            vid = v.get_id()
            wid = w.get_id()
            print('( %s , %s, %3d)' % (vid, wid, v.get_neighbor_distance(w)))

    print(g)
    dijkstra(g, g.get_vertex('a'), g.get_vertex('e'))

    target = g.get_vertex('e')
    path = [target.get_id()]
    shortest(target, path)
    print("The shortest Path: %s" % path[::-1])

