import numpy as np
import heapq as hp
import matplotlib.pyplot as plt

from scipy import linalg
from double_list import DoubleList

# Prim's algorithm for points in an XY plane
class PrimComplete:

    ## Constructor
    #
    #  @param points Array of xy points
    def __init__(self, points):
        self.vertices = map(tuple, points)

        # create pairs of form (dist, v, nv), where dist is distance of the
        # vertex (v) to the nearest vertex (nv) in the mst so far
        func = lambda x : (linalg.norm(np.array(x) - np.array(self.vertices[0])), x, self.vertices[0])
        args = self.vertices[1:]
        self.heap = map(func, args)

        # create list of vertices not covered
        self.unused = DoubleList()
        for x in self.heap:
            self.unused.append(x)

        # create heap of vertices
        hp.heapify(self.heap)

        # list of edges, represented as pairs of vertices
        self.mstEdges = []
        self.coveredVertices = {self.vertices[0]: True}


    def chooseMinVertex(self):
        # find min vertex and create edge
        minVertex = hp.heappop(self.heap)
        recentVertex = minVertex[1]
        newEdge = minVertex[1:]

        # if vertex is not yet used
        if recentVertex not in self.coveredVertices:
            # update edge set
            self.mstEdges.append(newEdge)

            # mark vertex as used
            self.coveredVertices[recentVertex] = True

            # update heap and list of unused vertices
            currentNode = self.unused.head

            while currentNode is not None:

                # remove vertex that was just added
                if currentNode.data == minVertex:
                    self.unused.remove(currentNode)
                else:
                    (currDist, v, nv) = currentNode.data
                    tentativeDist = linalg.norm(np.array(v) - np.array(recentVertex))

                    # update heap and list of unused if unused vertex is nearer to
                    # vertex that was just used
                    if tentativeDist < currDist:
                        newTriplet = (tentativeDist, v, recentVertex)
                        hp.heappush(self.heap, newTriplet)

                        self.unused.remove(currentNode)
                        self.unused.replace(currentNode, newTriplet)

                currentNode = currentNode.next

    def mst(self):
        while(len(self.heap) > 0):
            self.chooseMinVertex()

        return self.mstEdges
