import numpy as np
import matplotlib.pyplot as plt

from min_hamiltonian import MinHamiltonian

numV = 15
MinHamiltonian.populationSize = 200
MinHamiltonian.debug = True
MinHamiltonian.greedyRatio = 0

randX = np.random.random_sample(numV)
randY = np.random.random_sample(numV)

coords = np.matrix.transpose(np.array([randX, randY]))
startPoint = np.array([-0.1,0.5])
endPoint = np.array([1.1,0.5])

mhp = MinHamiltonian(startPoint, coords, endPoint)
mhp.minPath()
#
#MinHamiltonian.greedyRatio = 0.75
#mhp.minPath()
