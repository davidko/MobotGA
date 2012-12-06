#!/usr/bin/python
import os
import random
import subprocess

class Chromosome:
  def __init__(self, index = 0):
    self.index = index
    ''' initChromosome is a list of chromosome values '''
    self.data = []
    for i in range (0, 120):
      self.data.append( random.randint(0, 200) )
    self.__fitness = 0

  def __str__(self):
    mystr = str(self.fitness())
    mystr = mystr + ': '
    for i in self.data:
      mystr = mystr + str(i)
      mystr = mystr + ' '
    return mystr

  def getIndex(self):
    return self.index

  def loadFile(self, filename):
    f = open(filename)
    i = 0
    for line in f:
      self.data[i] = int(line)
      i = i+1
    f.close()
    self.filename = filename
    self.calculate_fitness()

  def writeFile(self, filename):
    f = open(filename, "w")
    for i in self.data:
      f.write(str(i));
      f.write('\n')
    f.close()

  def crossover(self, c):
    '''c is another chromosome'''
    x = random.randint(1, 39);
    newChromosome = Chromosome()
    for i in range (0, x):
      newChromosome.data[i] = self.data[i]
    for i in range (x, 40):
      newChromosome.data[i] = c.data[i]
    return newChromosome

  def fitness(self):
    return self.__fitness

  def calculate_fitness(self):
    ''' returns the distance traveled in 30 seconds in meters '''
    command = ['../main',  '--disable-graphics',  '--load-coefs', './{}'.format(self.filename)]
    print command
    output = subprocess.check_output(command)
    self.__fitness = float(output)

class Population:
  def __init__(self):
    self.members = []

  def newPopulation(self, dirname, num):
    random.seed()    
    os.mkdir(dirname)
    for i in range (0, num):
      c = Chromosome()
      c.writeFile('{}/chromosome{}.txt'.format(dirname, str(i).zfill(3)))
      self.members.append(c)

  def loadDir(self, dirname):
    self.members = [];
    files = os.listdir(dirname)
    files.sort()
    i = 0
    for f in files:
      c = Chromosome(i)
      c.loadFile('{}/{}'.format(dirname, f))
      self.members.append(c)
      i = i + 1
    self.members.sort(key = lambda m: m.fitness())
    return population

  def writeFitnesses(self, filename):
    f = open(filename, "w")
    for p in self.members:
      f.write(str(p.getIndex()) + ' ' + str(p.fitness()) + '\n')
    f.close()

  def min(self):
    return self.members[0].fitness()

  def avg(self):
    a = 0.0
    for m in self.members:
      a = a + m.fitness();
    a = a / len(self.members)
    return a

  def max(self):
    return self.members[-1].fitness()

if __name__ == '__main__':
  statsfile = open('GAstats.txt', 'w')
  # Load the population
  i = 0
  population = Population()
  population.loadDir('gen{}'.format(str(i).zfill(3)))
  population.writeFitnesses('fitnesses{}'.format(str(i).zfill(3)))
  # sort the population by fitness 
  statsfile.write('{} {} {}\n'.format(population.min(), population.avg(), population.max()))
  statsfile.close()

