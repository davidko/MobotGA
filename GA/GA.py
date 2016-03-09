#!/usr/bin/python
import os
import random
import subprocess
import math
import threading
import hashlib

known_fitnesses = {}
class Chromosome:
  instances_lock = threading.Condition()
  num_instances = 0
  max_instances = 4
  def __init__(self, index = 0):
    self.index = index
    ''' initChromosome is a list of chromosome values '''
    self.data = []
    for i in range (0, 120):
      self.data.append( random.randint(0, 255) )
    self.__fitness = 0
    self.thread = None

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

  def asyncLoadFile(self, *_args):
    self.thread = threading.Thread(
        target=self.asyncLoadFile_thread,
        args=_args)
    self.thread.start()

  def asyncLoadFile_thread(self, filename):
    Chromosome.instances_lock.acquire()
    while Chromosome.num_instances >= Chromosome.max_instances:
        Chromosome.instances_lock.wait()
    Chromosome.num_instances += 1
    Chromosome.instances_lock.release()

    self.loadFile(filename)

    Chromosome.instances_lock.acquire()
    Chromosome.num_instances -= 1
    Chromosome.instances_lock.notify()
    Chromosome.instances_lock.release()


  def writeFile(self, filename):
    f = open(filename, "w")
    for i in self.data:
      f.write(str(i));
      f.write('\n')
    f.close()

  def crossover(self, c):
    '''c is another chromosome'''
    x = random.randint(1, 119);
    newChromosome = Chromosome()
    for i in range (0, x):
      newChromosome.data[i] = self.data[i]
    for i in range (x, 120):
      newChromosome.data[i] = c.data[i]
    return newChromosome

  def fitness(self):
    return self.__fitness

  def calculate_fitness(self):
    ''' returns the distance traveled in 30 seconds in meters '''
    ''' See if the fitness is already known '''
    m = hashlib.md5()
    m.update(str(self.data))
    try:
        self.__fitness = known_fitnesses[m.digest()]
        print('Using saved fitness: {0}'.format(self.__fitness))
    except:
        command = ['../main',  '--disable-graphics',  '--load-coefs', './{}'.format(self.filename)]
        print command
        output = subprocess.check_output(command)
        positions = output.split()
        distance = 0.0;
        for p in positions:
          distance = distance + float(p)**2
        distance = math.sqrt(distance)
        print distance
        self.__fitness = distance
        known_fitnesses[m.digest()] = distance

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
    files = filter(lambda x: x.endswith(".txt"), files)
    files.sort()
    i = 0
    for f in files:
      c = Chromosome(i)
      #c.loadFile('{}/{}'.format(dirname, f))
      c.asyncLoadFile('{}/{}'.format(dirname, f))
      self.members.append(c)
      i = i + 1
    for m in self.members:
        m.thread.join()

    self.members.sort(key = lambda m: m.fitness(), reverse = True)
    return population

  def writeDir(self, dirname):
    os.mkdir(dirname)
    for m,i in zip(self.members,range(0,len(self.members))):
      m.writeFile('{}/chromosome{}.txt'.format(dirname, str(i).zfill(3)))

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

  def diversity(self):
    # First, calculate centroids
    centroids = [0 for _ in range(len(self.members[0].data))]
    for m in self.members:
        centroids = [x+y for x,y in zip(centroids, m.data)]
    centroids = map(lambda x: float(x)/len(self.members), centroids)
    # Now calculate inertias
    inertia = 0
    for i in range(len(self.members[0].data)):
        for j in range(len(self.members)):
            inertia += (self.members[j].data[i] - centroids[i])**2
    return inertia

  def regen(self):
    '''Create and return a new population generated from the elite members of the last population'''
    newpop = self.members[0:len(self.members)/2]
    eliteNum = len(newpop)
    while len(newpop) < len(self.members):
      m1 = newpop[random.randint(0, eliteNum-1)]
      m2 = newpop[random.randint(0, eliteNum-1)]
      newpop.append( m1.crossover(m2) )
    self.members = list(newpop)

if __name__ == '__main__':
  statsfile = open('GAstats.txt', 'w')
  population = Population()
  population.newPopulation('gen000', 50)
  diversity = 100
  i = 0
  while diversity > 0:
    population.loadDir('gen{}'.format(str(i).zfill(3)))
    population.writeFitnesses('fitnesses{}'.format(str(i).zfill(3)))
    # sort the population by fitness 
    diversity = population.diversity()
    statsfile.write('{} {} {} {}\n'.format(population.min(), population.avg(),
        population.max(), diversity))
    statsfile.flush()
    population.regen()
    population.writeDir('gen{}'.format(str(i+1).zfill(3)))
    i += 1
  statsfile.close()

