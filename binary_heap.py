import math
from copy import copy

class heap_item(object):
    '''
    Items to be used in a binary heap
    '''
    def __init__(self, value=None):
        self._index = None
        self._value = value
        self.walkable = True

    def __str__(self):
        if self._value is None:
            return "None"
        return str(self._value)

    def __lt__(self, other):
        return self.value < other.value

    def __gt__(self, other):
        return self.value > other.value

    def __eq__(self, other):
        return (self.value == other.value
                and self._index == other._index)

    @property
    def valid(self):
        return (self._index is not None 
               and self._index >= 0)

    def set_value(self, val):
        self._value = val

    def get_value(self):
        return self._value

    value = property(get_value, set_value)

    def get_left(self, max=None):
        if not self.valid:
            return None

        val = 2 * self._index + 1
        if max is not None:
            if val >= max:
                return None
        return val

    def get_right(self, max=None):
        if not self.valid:
            return None

        val = 2 * self._index + 2
        if max is not None:
            if val >= max:
                return None
        return val

    left = property(get_left)
    right = property(get_right)

    @property
    def parent(self):
        return int((self._index-1)//2)

    @property
    def children(self):
        return [self.left, self.right]
    

class min_heap(object):
    def __init__(self, size=0):
        if size > 0:
            self._items = [heap_item() for i in range(size)]
        else:
            self._items=[]
        for i, item in enumerate(self._items):
            item._index = i
        self._count = size

    def __len__(self):
        return self._count

    def __getitem__(self, idx):
        if idx < self._count:
            return self._items[idx]

    def __setitem__(self, idx, val):
        if not isinstance(val, heap_item):
            raise TypeError("Items assigned to binary tree must be of type heap_item")
        if idx < self._count:
            self._items[idx] = val
            self.update_item(val)
        else:
            raise IndexError("Invalid tree index: " + idx)

    def __str__(self):
        # Prints the tree
        if self._count<1:
            return ""
        levels = int(self._count/2)
        res = ""
        cur_level = [self._items[0]]
        lev_num = -1
        LR = ["/","\\"]
        while cur_level:
            spaces = (levels - 2*lev_num) * "  "
            lev_num += 1
            res += ' '.join(spaces + str(node).center(4,' ') for i,node in enumerate(cur_level)) + "\n"
            res += ' '.join(spaces + LR[i%2].center(4,' ') for i in range(len(cur_level))) + "\n"
            next_level = []
            for node in cur_level:
                l, r = node.get_left(self._count), node.get_right(self._count)
                if l is not None:
                    next_level.append(self._items[l])
                if r is not None:
                    next_level.append(self._items[r])
            cur_level = next_level
        return res

    def append(self, item:heap_item):
        # Adds an item to the tree, resorting as needed
        item._index = self._count
        self._items.append(item)
        self._sort_up(self._count)
        self._count += 1

    def _sort_up(self, index):
        # Sorts the tree, moving values from the bottom of the tree up
        item = self._items[index]
        if not item.valid:
            return

        while item.parent >= 0:
            parent = self._items[item.parent]
            if item < parent:
                self._swap(item, parent)
            else:
                break


    def _sort_down(self, index):
        # Sorts the tree, from the top value, moving it down the tree
        if self._count == 0:
            return

        item = self._items[index]
        if not item.valid:
            return
        while True:
            ltChild = item.left
            rtChild = item.right
            swapIndex = 0

            if ltChild < self._count:
                swapIndex = ltChild
                if rtChild < self._count:
                    if self._items[ltChild] > self._items[rtChild]:
                        swapIndex = rtChild
                if item > self._items[swapIndex]:
                    self._swap(item, self._items[swapIndex])
                else:
                    return
            else:
                return

    def update_item(self, item):
        self._sort_up(item._index)

    def remove_first(self):
        # Removes the first item in the tree, resorting as needed
        if self._count <= 0:
            return None
        firstItem = self._items[0]
        self._count -= 1
        self._items[0] = self._items[self._count]
        self._items[0]._index = 0
        del(self._items[self._count])
        self._sort_down(0)
        firstItem._index = None
        return firstItem

    def contains(self, item):
        # Checks if a given tree item is in the tree
        if item._index is None:
            return False
        if item._index >= self._count:
            return False
        return self._items[item._index] == item

    def _swap(self, a, b):
        # Swaps two tree items
        aI = a._index
        bI = b._index
        a._index = bI
        b._index = aI
        (self._items[aI], self._items[bI]) = (self._items[bI], self._items[aI])


def main():
    elements = [12, 8, 6, 7, 25, 3, 5, 11, 11]
    heap = min_heap(size=len(elements))
    print (elements)
    for i, e in enumerate(elements):
        heap[i].value = e
        heap.update_item(heap[i])
    print(heap)

if __name__ == '__main__':
    main()