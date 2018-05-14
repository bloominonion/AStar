'''/////////////////////////////////////////////////////////////////////////

 Name        : GameBoard.py
 Description : Testing and useage of AStar algorithm implementation
 
 ///////////////////////////////////////////////////////////////////////////'''
__script__ = 'GameBoard.py';
__date__ = '14-May-2018';

import numpy
from AStarSearch import PathFinding, Node, Point
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from skimage.transform import resize
from pathlib import Path
from os import system

class Map(object):
    '''
    Map class for generating and managing the storage and relations of Nodes
    '''
    def __init__(self, nx, ny):
        self._nx = nx
        self._ny = ny
        self.node_map = [Node() for i in range(nx*ny)]
        for i, node in enumerate(self.node_map):
            node.data = i
            node.loc = Point(*self._from_cell(i), 0)
            node.neighbors = self.get_neighbors(node)
        self.plt, self.ax = None, None
        self.lastFig = 0
        self.orig, self.targ = None,None

    def set_pts(self, orig:Node=None, targ:Node=None):
        # Sets the start/target points for drawing purposes
        self.orig = orig
        self.targ = targ

    def draw(self, open=None, closed=None, path=None):
        # Draws the map as a png
        if self.plt is None and self.ax is None:
            p = Path('.').resolve()
            [frame.unlink() for frame in p.glob("*.png")]
            self.plt, self.ax = plt.subplots()

        openCells, closedCells, pathCells = [], [], []

        if open is not None:
            openCells = sorted([open[i].data for i in range(len(open))])
        if closed is not None:
            closedCells = sorted([x.data for x in closed])
        if path is not None:                
            pathCells = sorted([x.data for x in path])
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
            pltArr[self.orig.data] = 9
        if self.targ is not None:
            pltArr[self.targ.data] = 10
        
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
        x0, y0 = self._from_cell(node.data)
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


def test_board():
    board = Map(11, 6)
    board.set_cell(3,4,False)
    board.set_cell(3,3,False)
    board.set_cell(4,3,False)
    board.set_cell(5,3,False)
    board.set_cell(6,3,False)
    board.set_cell(7,3,False)

    a = Point(7, 1, 0)
    b = Point(4, 4, 0)
    aNode = board.get_node_from_point(a)
    bNode = board.get_node_from_point(b)
    board.set_pts(orig=aNode, targ=bNode)
    
    pather = PathFinding(draw_callback=board.draw)
    res = pather.find_path(aNode, bNode)

    res[0].loc = a
    res[-1].loc = b

    print ("Path:")
    for node in res:
        print ("  ", node.loc)

    if True:
        system('"C:/Program Files/ImageMagick-7.0.7-Q16/magick.exe" convert *.png test.gif')
        x, y = [],[]
        for node in res:
            x.append(node.loc.x)
            y.append(node.loc.y)

        y.reverse()
        x.reverse()
        plt.plot(x, y)
        plt.plot(a.x, a.y, b.x, b.y, marker='o', color='red')

        plt.axis([int(min(x)-1.5), int(max(x) + 1.5), int(min(y)-1.5), int(max(y)+1.5)])
        # plt.show()
   
if __name__ == '__main__':
    test_board()