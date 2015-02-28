import numpy as np
import matplotlib.pyplot as plt

from tsp_approx import TravellingSalesmanMST

numV = 15
randX = np.random.random_sample(numV)
randY = np.random.random_sample(numV)
randZ = np.random.random_sample(numV)

coords = np.matrix.transpose(np.array([randX, randY, randZ]))
startPoint = np.random.random_sample(3)
tsp = TravellingSalesmanMST(startPoint, coords)
coordsT = np.matrix.transpose(np.array(coords))
print coordsT

path = tsp.tspPath()
print path

plt.figure(figsize = (8.0, 9.0))

plt.subplot(2, 1, 1)

plt.title('Minimum Spanning Tree')

plt.plot(startPoint[0], startPoint[1], 'o')
plt.plot(coordsT[0], coordsT[1], 'o')
for x in tsp.prim.mstEdges:
    lineT = np.matrix.transpose(np.array(x))
    plt.plot(lineT[0], lineT[1], 'r-', lw=1)

plt.axis('equal')
plt.margins(.1*(max(coordsT[0]) - min(coordsT[0])),.1*(max(coordsT[1]) - min(coordsT[1])))


plt.subplot(2, 1, 2)

plt.title('MST Approximation')

plt.plot(startPoint[0], startPoint[1], 'o')
plt.plot(coordsT[0], coordsT[1], 'o')
for ind in xrange(len(path) - 1):
    x = path[ind : ind+2]
    lineT = np.matrix.transpose(np.array(x))
    plt.plot(lineT[0], lineT[1], 'k-', lw=1)

plt.plot([path[0][0], path[-1][0]], [path[0][1], path[-1][1]], 'k--', lw=1)

plt.axis('equal')
plt.margins(.1*(max(coordsT[0]) - min(coordsT[0])),.1*(max(coordsT[1]) - min(coordsT[1])))

plt.show()
