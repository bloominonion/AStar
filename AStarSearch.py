'''/////////////////////////////////////////////////////////////////////////

 Name        : AStarSearch.py
 Description : Implementation for A-Star searh algorithm
 
 ///////////////////////////////////////////////////////////////////////////'''
__script__ = 'AStarSearch.py';
__date__ = '07-May-2018';

import binary_heap
import math
from textwrap import dedent

class Point(object):
    ''' Point class for storing x/y/z and calculating node distance. '''
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "{}, {}, {}".format(self.x, self.y, self.z)

    def __eq__(self, other):
        return (abs(self.x - other.x)
              + abs(self.y - other.y)
              + abs(self.z - other.z)) < 0.01

    def distance(self, other):
        return math.sqrt((other.x-self.x)**2 
                       + (other.y-self.y)**2 
                       + (other.z-self.z)**2)

class Node(binary_heap.heap_item):
    ''' Node class for searching with using A* algorithm. '''
    def __init__(self, pt=None, dist_func=None, data=None):
        self.loc = pt
        self.data = data
        self.g_cost = 0
        self.h_cost = math.inf
        self.prev_node = None
        self._neighbors = []
        self.dist_func = None
        if dist_func is not None:
            self.dist_func = dist_func
        super(Node, self).__init__()

    def __eq__(self, other):
        # Compare two nodes
        if self.loc is None or other.loc is None:
            if self.data is not None and other.data is not None:
                return self.data == other.data
            return False
        return self.loc == other.loc 

    def __str__(self):
        return dedent('''
                        Useable: {}
                        g_cost : {}
                        h_cost : {}
                      '''.format(self.walkable, self.g_cost, self.h_cost))

    def get_neighbors(self):
        if callable(self._neighbors):
            return self._neighbors(self)
        else:
            return self._neighbors

    def set_neighbors(self, neigh):
        self._neighbors = neigh

    neighbors = property(get_neighbors, set_neighbors)

    def set_useable(self, value):
        # Sets a node's usability
        self.walkable = bool(value)

    def cost(self):
        # Gets the total cost of a node
        return self.g_cost + self.h_cost

    def distance(self, other):
        # Gets the distance between two nodes
        if self.dist_func is not None:
            return self.dist_func(self,other)
        if self.loc is None:
            return math.inf
        return self.loc.distance(other.loc)

    def update(self, h_cost=None, g_cost=None):
        # Update the h/gcost of a node, or just update the stored value
        if h_cost is not None:
            self.h_cost = h_cost
        if g_cost is not None:
            self.g_cost = g_cost
        self.value = self.cost()


class PathFinding:
    def __init__(self, draw_callback=None):
        self.no_draw = True
        if draw_callback is not None:
            if not callable(draw_callback):
                raise TypeError("draw_callback must be callable.")
            self.draw = draw_callback
            self.no_draw = False
        else:
            self.draw = lambda *args, **kwargs:None

    def _get_path(self, lastNode):
        result = [] 
        while lastNode is not None:
            result.append(lastNode)
            lastNode = lastNode.prev_node

        result.reverse()
        return result

    def _get_path_points(self, lastNode):
        path = self._get_path(lastNode)
        return [node.loc for node in path]

    def find_path(self, begin:Node, end:Node):
        if not self.no_draw:
            self.draw()

        open_set = binary_heap.min_heap()
        closed_set = []
        open_set.append(begin)
        curNode = None
        solutionFound = False
        while len(open_set):
            curNode = open_set.remove_first() 
            closed_set.append(curNode)

            if not self.no_draw:
                path = self._get_path(curNode)
                self.draw(open=open_set, closed=closed_set, path=path)

            if curNode == end:
                print ("Found the end of the yellow brick road!")
                solutionFound = True
                break

            for neighbor in curNode.neighbors:
                if not neighbor.walkable or neighbor in closed_set:
                    continue

                new_cost = curNode.g_cost + curNode.distance(neighbor)
                if new_cost < neighbor.g_cost or not open_set.contains(neighbor):
                    neighbor.update(g_cost=new_cost, h_cost=neighbor.distance(end))
                    neighbor.prev_node = curNode

                    if not open_set.contains(neighbor):
                        open_set.append(neighbor)
                    else:
                        open_set.update_item(neighbor)

        if solutionFound:
            return self._get_path(curNode)
        else:
            return []

def main():
    import difflib
    from itertools import permutations
    start_string = 'dfaceb'
    desired_string = 'abcdef'
    opts = []
    str_perm = set([''.join(p) for p in permutations(start_string)])

    def dist(self, other):
        delta = difflib.SequenceMatcher(None, self.data, other.data)
        return len(delta.get_matching_blocks())

    def get_neighbors(node):
        idx = opts.index(node)
        if idx == 0:
            return [opts[idx+1]]
        if idx == len(opts)-1:
            return [opts[idx-1]]
        return [opts[idx-1], opts[idx+1]]

    for perm in str_perm:
        opts.append(Node(data=perm, dist_func=dist))
        opts[-1].neighbors = get_neighbors


    a = Node(data=start_string, dist_func=dist)
    b = Node(data=desired_string, dist_func=dist)

    a.neighbors = get_neighbors

    pather = PathFinding()
    res = pather.find_path(a, b)
    print ("Result")
    [print (node.data) for node in res]
   
if __name__ == '__main__':
    main()