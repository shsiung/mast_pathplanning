import numpy as np

from scipy import linalg
from prim_complete import PrimComplete
from collections import deque

class TravellingSalesmanMST:

    def __init__(self, startPoint, destinations):

        # find minimum spanning trees
        self.startPoint = tuple(startPoint)
        points = [self.startPoint] + map(tuple, destinations)

        self.prim = PrimComplete(points)
        self.prim.mst()

        # create adjacency list from MST edges
        self.tree = {}
        for edge in self.prim.mstEdges:
            (dest, source) = edge
            if source not in self.tree:
                self.tree[source] = [dest]
            else:
                self.tree[source].append(dest)



    def tspPath(self):
        # define root to be start point
        stack = deque([self.startPoint])
        self.path = []

        # pre-order traversal of mst
        while(len(stack) != 0):
            current = stack.pop()
            self.path.append(current)

            if current in self.tree:
                distVertexPairs = []

                # put all neighbours in a list
                for vertex in self.tree[current]:
                    diff = np.array(vertex) - np.array(current)
                    distVertexPairs.append((linalg.norm(diff), vertex))

                # sort list based on distance to current
                distVertexPairs.sort(reverse = True)

                # push all neighbours to stack
                for pair in distVertexPairs:
                    stack.append(pair[1])

        return self.path

