import numpy as np
import matplotlib.pyplot as plt

from scipy import linalg
from copy import deepcopy
from sys import stdout

class MinHamiltonian:

    populationSize = 75
    maxIterations = 50
    convergenceThreshold = 4

    mutateProbability = 0.15
    crossFactor = 0.5

    debug = False
    greedyRatio = 0.5

    def __init__(self, startPoint, destinations, endPoint):
        self.startPoint = startPoint
        self.destinations = destinations
        self.endPoint = endPoint


    def pathCost(self, path):
        cost = 0

        # sum distances between successive points
        for ind in xrange(len(path) - 1):
            diff = np.array(path[ind]) - np.array(path[ind+1])
            cost = cost + linalg.norm(diff)

        # add distances of end points to start and end
        diffStart = np.array(self.startPoint) - np.array(path[0])
        diffEnd = np.array(self.endPoint) - np.array(path[-1])
        cost = cost + linalg.norm(diffStart)
        cost = cost + linalg.norm(diffEnd)

        return cost


    def plot(self):
        startPoint = self.startPoint
        endPoint = self.endPoint

        plt.plot(startPoint[0], startPoint[1], 'o', color = 'r')
        plt.plot(endPoint[0], endPoint[1], 'o', color = 'g')

        coordsT = np.matrix.transpose(np.array(self.destinations))
        plt.plot(coordsT[0], coordsT[1], 'o', color = 'b')

        for ind in xrange(len(self.bestPath) - 1):
            x = self.bestPath[ind : ind+2]
            lineT = np.matrix.transpose(np.array(x))
            plt.plot(lineT[0], lineT[1], 'k-', lw=1)

        plt.plot([self.bestPath[0][0], startPoint[0]], [self.bestPath[0][1], startPoint[1]], 'r-', lw=1)
        plt.plot([self.bestPath[-1][0], endPoint[0]], [self.bestPath[-1][1], endPoint[1]], 'g-', lw=1)

        plt.axis('equal')
        plt.margins(0.1, 0.1)



    def greedy(self, destinations, start, random = False, alternate = False):
        dest = map(tuple, destinations)

        pathForward = []
        pathBackward = []

        currentForward = start
        currentBackward = self.endPoint

        while len(dest) != 0:
            # alternate between going forwards and backwards
            forwardFlag = len(dest) % 2 == 0 if not alternate else True

            # set current node and path based of forwardFlag
            current    = currentForward if forwardFlag else currentBackward
            greedyPath = pathForward     if forwardFlag else pathBackward

            func = lambda x : 1.0 / (linalg.norm(np.array(x) - np.array(current)) ** 0.25)
            goodness = np.array(map(func, dest))

            probabilities = goodness / np.sum(goodness)
            indicies = np.arange(len(dest))

            if random:
                nextIndex = np.random.choice(indicies, p = probabilities)
                nextPoint = dest[nextIndex]
            else:
                scoreAndIndicies = zip(goodness, indicies)
                (score, nextIndex) = max(scoreAndIndicies)
                nextPoint = dest[nextIndex]

            greedyPath.append(np.array(nextPoint))
            dest.remove(nextPoint)

            if forwardFlag:
                currentForward = nextPoint
            else:
                currentBackward = nextPoint


        pathBackward.reverse()

        return np.array(pathForward + pathBackward)


    def mutate(self, path):
        if np.random.randint(0, 2) == 1:
            return self.shuffle(path)

        else:
            startInd = np.random.randint(0, len(path)-1)

            startPoint = path[startInd]
            destinations = path[startInd+1:]

            partialPath =\
                    self.greedy(destinations, startPoint, random=True, alternate=(startInd%2==0))

            return np.concatenate((path[:startInd+1], partialPath))


    def evolve(self):
        newPopulation = self.population

        for x in xrange(self.populationSize):
            for y in xrange(x + 1, self.populationSize):
                newOrg = self.cross(self.population[x], self.population[y])

                mutateOdds = np.random.rand()

                if mutateOdds <= self.mutateProbability:
                    newOrg = self.mutate(newOrg)

                newPopulation.append(newOrg)

        func = lambda x : (self.pathCost(x), x)
        costsAndPaths = map(func, newPopulation)

        keyfunc = lambda x : x [0]
        costsAndPaths.sort(key = keyfunc)

        selected = costsAndPaths[0:self.populationSize]
        (minCostTent, minPathTent) = selected[0]

        if minCostTent < self.leastCost:
            self.leastCost = minCostTent
            self.bestPath = minPathTent

        self.population = map(lambda x : x[1], selected)


    def progressBar(self):
        nBars = 40

        nLines = nBars if self.converged else nBars * self.currentGen / self.maxIterations
        nSpaces = nBars - nLines

        return '\r|{0}| Min: {1:.3f}'.format('=' * nLines + ' ' * nSpaces, self.leastCost)


    def minPath(self):
        self.currentGen = 0
        self.converged = False

        nGreedy = int(round(self.populationSize * self.greedyRatio))
        self.population = [self.greedy(self.destinations,self.startPoint,random=(x<2), alternate=(x%2==1))\
                for x in xrange(nGreedy)]

        while len(self.population) != self.populationSize:
            self.population.append(MinHamiltonian.shuffle(self.destinations))


        self.leastCost = float('inf')

        if self.debug:
            plt.ion()
            plt.show()

        costHistory = []

        for x in xrange(self.maxIterations):
            self.currentGen = self.currentGen + 1
            self.evolve()

            costHistory.append(self.leastCost)
            self.converged = len(costHistory) > self.convergenceThreshold\
                    and len(set(costHistory[-self.convergenceThreshold:])) == 1

            if self.debug:
                stdout.write(self.progressBar())
                stdout.flush()

                plt.clf()
                self.plot()
                plt.draw()

            if self.converged:
                break

        if self.debug:
            stdout.write('\n')
            plt.ioff()
            plt.clf()
            self.plot()
            plt.show()

        return self.bestPath


    @staticmethod
    def shuffle(path):
        pathCopy = deepcopy(path)
        np.random.shuffle(pathCopy)

        return pathCopy


    def cross(self, p1, p2):
        choice = np.random.randint(0, 2)

        pathCopy = deepcopy(p1) if choice == 0 else deepcopy(p2)
        pathSrc = p1 if choice == 1 else p2

        # calculate name of swaps to do
        numSwaps = np.random.randint(1, max(self.populationSize * self.crossFactor, 2))
        desIndicies = np.random.randint(0, len(pathCopy), numSwaps)

        for desIndex in desIndicies:
            MinHamiltonian.findAndSwap(pathCopy, pathSrc[desIndex], desIndex)

        return pathCopy


    # move a vertex in path to the desired index by swapping
    @staticmethod
    def findAndSwap(path, vertex, desIndex):
        srcIndex = np.where(path == vertex)[0][0]
        path[srcIndex] = path[desIndex]
        path[desIndex] = vertex
