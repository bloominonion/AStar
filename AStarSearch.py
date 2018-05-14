'''/////////////////////////////////////////////////////////////////////////

 Name        : AStarSearch.py
 Description : Implementation for A-Star searh algorithm
 
 ///////////////////////////////////////////////////////////////////////////'''
__script__ = 'AStarSearch.py';
__date__ = '07-May-2018';

import binary_heap
import math
import numpy
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from skimage.transform import resize
from copy import copy
from pathlib import Path
from textwrap import dedent
from os import system

DRAW_PROGRESS = True

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
    def __init__(self, pt=None):
        self.loc = pt
        self.cell = None
        self.g_cost = 0
        self.h_cost = math.inf
        self.prev_node = None
        self.neighbors = []
        super(Node, self).__init__()

    def __eq__(self, other):
        # Compare two nodes
        return self.loc == other.loc 

    def __str__(self):
        return dedent('''
                        Cell   : {} ({})
                        Useable: {}
                        g_cost : {}
                        h_cost : {}
                      '''.format(self.cell, self.loc, self.walkable, self.g_cost, self.h_cost))

    def add_neighbor(self, neighbor):
        if isinstance(neighbor, Node):
            self.neighbors.append(neighbor)
        else:
            raise TypeError("Neighbors must be of the same type.")

    def set_useable(self, value):
        # Sets a node's usability
        self.walkable = bool(value)

    def cost(self):
        # Gets the total cost of a node
        return self.g_cost + self.h_cost

    def distance(self, other):
        # Gets the distance between two nodes
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

class Map(object):
    '''
    Map class for generating and managing the storage and relations of Nodes
    '''
    def __init__(self, nx, ny):
        self._nx = nx
        self._ny = ny
        self.node_map = [Node() for i in range(nx*ny)]
        for i, node in enumerate(self.node_map):
            node.cell = i
            node.loc = Point(*self._from_cell(i), 0)
            node.neighbors = self.get_neighbors(node)
        self.plt, self.ax = None, None
        self.lastFig = 0
        self.orig, self.targ = None,None

    def set_pts(self, orig=None, targ=None):
        # Sets the start/target points for drawing purposes
        self.orig = orig
        self.targ = targ

    def draw(self, open=None, closed=None, path=None):
        # Draws the map as a png
        if self.plt is None and self.ax is None:
            self.plt, self.ax = plt.subplots()

        openCells, closedCells, pathCells = [], [], []

        if open is not None:
            openCells = sorted([open[i].cell for i in range(len(open))])
        if closed is not None:
            closedCells = sorted([x.cell for x in closed])
        if path is not None:                
            pathCells = sorted([x.cell for x in path])
        pltArr = numpy.array(numpy.zeros(len(self.node_map)))
        for i, node in enumerate(self.node_map):
            nSet = 0
            if not node.walkable:
                pltArr[i] = -1
                nSet += 1
            if i in openCells:
                pltArr[i] = 1
                nSet += 1
            if i in closedCells:
                pltArr[i] = 2
                nSet += 1
            if i in pathCells:
                pltArr[i] = 5
            if nSet > 1:
                raise ValueError("Cell set more than one time for valid/hit/processed")

        # Set the origin and target points
        if self.orig is not None:
            pltArr[self.orig.cell] = 9
        if self.targ is not None:
            pltArr[self.targ.cell] = 10
        
        lg = max(self._nx, self._ny)
        newSz = (int((self._ny/lg)*500), int((self._nx/lg)*500))
        # Reshape to 2D array and flip Y-axis since images are from top-left
        # and cartesian space is from bottom-left
        pltArr = pltArr.reshape(self._ny, -1)[::-1,:]
        pltArr = resize(pltArr, newSz, order=0)

        # self.ax.imshow(pltArr, interpolation='none')
        plt.imsave("Fig{:03}.png".format(self.lastFig), pltArr)
        self.lastFig += 1


    def _bound_check(self, x, y):
        # Checks a value is within the board size
        if x > self._nx or x < 0:
            raise ValueError("Cell X value ({}) out of bounds. ({}:{})".format(x, 0, self._nx))
        if y > self._ny or y < 0:
            raise ValueError("Cell Y value ({}) out of bounds. ({}:{})".format(y, 0, self._ny))

    def _to_cell(self, x, y):
        # Converts X/Y to encoded cell number
        return int(x) + int(y*self._nx)

    def _from_cell(self, cell):
        # Converts cell number back to x/y
        tmp = cell
        x = tmp%self._nx
        tmp /= self._nx
        y = tmp%self._ny
        return [int(x), int(y)]

    def get_node(self, x, y):
        # Gets a node based on grid coordinate
        return self.node_map[self._to_cell(x, y)]

    def set_cell(self, x, y, val):
        # Sets a cell to useable or not
        self._bound_check(x, y)
        cell = self._to_cell(x, y)
        self.node_map[cell].set_useable(val)

    def _clamp(self, min, max, val):
        # Clamps a value to be between min and max values
        if val < min:
            return min
        elif val > max:
            return max
        return val

    def get_neighbors(self, node:Node):
        # Gets the neighbors of a given node.
        x0, y0 = self._from_cell(node.cell)
        xran = [self._clamp(0,self._nx-1, x0-1), self._clamp(0, self._nx-1, x0+1)]
        yran = [self._clamp(0,self._ny-1, y0-1), self._clamp(0, self._ny-1, y0+1)]
        neigh = []
        for x in range(xran[0], xran[1]+1):
            for y in range(yran[0], yran[1]+1):
                if x == x0 and y == y0:
                    continue
                neigh.append(self.get_node(x, y))

        return neigh

    def get_node_from_point(self, pt:Point):
        # Finds the grid node that represents a point
        self._bound_check(pt.x, pt.y)
        cell = self._to_cell(pt.x, pt.y)
        return self.node_map[cell]


class PathFinding:
    def __init__(self, graph:Map):
        self.graph = graph
        # self.plt, self.ax = plt.subplots()

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

    def find_path(self, a:Point, b:Point):
        p = Path('.').resolve()
        [frame.unlink() for frame in p.glob("*.png")]

        start = self.graph.get_node_from_point(a)
        target = self.graph.get_node_from_point(b)
        start.h_cost = a.distance(b)
        target.g_cost = a.distance(b)
        print ("Start:", start.loc)
        print ("Taget:", target.loc)

        self.graph.set_pts(orig=start, targ=target)
        if DRAW_PROGRESS:
            self.graph.draw()

        open_set = binary_heap.min_heap()
        closed_set = []
        open_set.append(start)
        curNode = None
        solutionFound = False
        while len(open_set):
            curNode = open_set.remove_first() 
            closed_set.append(curNode)

            if DRAW_PROGRESS:
                path = self._get_path(curNode)
                self.graph.draw(open=open_set, closed=closed_set, path=path)

            if curNode is target:
                print ("Found the end of the yellow brick road!")
                solutionFound = True
                break

            for neighbor in curNode.neighbors:
                if not neighbor.walkable or neighbor in closed_set:
                    continue

                new_cost = curNode.g_cost + curNode.distance(neighbor)
                if new_cost < neighbor.g_cost or not open_set.contains(neighbor):
                    neighbor.update(g_cost=new_cost, h_cost=neighbor.distance(target))
                    neighbor.prev_node = curNode

                    if not open_set.contains(neighbor):
                        open_set.append(neighbor)
                    else:
                        open_set.update_item(neighbor)

        if solutionFound:
            # Build Path
            result = self._get_path(curNode)

            result[0] = start
            result[0].loc = a
            result[-1] = target
            result[-1].loc = b

            return [node.loc for node in result]
        else:
            return []

def main():

    board = Map(11, 6)
    board.set_cell(3,4,False)
    board.set_cell(3,3,False)
    board.set_cell(4,3,False)
    board.set_cell(5,3,False)
    board.set_cell(6,3,False)
    board.set_cell(7,3,False)

    pather = PathFinding(board)
    a = Point(7, 1, 0)
    b = Point(4, 4, 0)
    res = pather.find_path(a, b)

    if DRAW_PROGRESS:
        system('"C:/Program Files/ImageMagick-7.0.7-Q16/magick.exe" convert *.png test.gif')
        x, y = [],[]
        for pt in res:
            x.append(pt.x)
            y.append(pt.y)

        y.reverse()
        x.reverse()
        plt.plot(x, y)
        plt.plot(a.x, a.y, b.x, b.y, marker='o', color='red')

        plt.axis([int(min(x)-1.5), int(max(x) + 1.5), int(min(y)-1.5), int(max(y)+1.5)])
        # plt.show()
   
if __name__ == '__main__':
    main()