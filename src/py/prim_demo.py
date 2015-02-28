import numpy as np
import matplotlib.pyplot as plt

from prim_complete import PrimComplete

numV = 30
randX = np.random.random_sample(numV)
randY = np.random.random_sample(numV)

coords = np.matrix.transpose(np.array([randX, randY]))
coordsT = np.matrix.transpose(np.array(coords))
pc = PrimComplete(coords)

pc.mst()

plt.clf()
plt.plot(coordsT[0], coordsT[1], 'o')

for x in pc.mstEdges:
    lineT = np.matrix.transpose(np.array(x))
    plt.plot(lineT[0], lineT[1], 'k-', lw=1)

plt.axes().set_aspect('equal', 'datalim')
plt.margins(.1*(max(coordsT[0]) - min(coordsT[0])),.1*(max(coordsT[1]) - min(coordsT[1])))

plt.show()

