#!/usr/bin/python
import os
import random
import subprocess

class Chromosome:
  def __init__(self, initChromosome = None):
    ''' initChromosome is a list of chromosome values '''
    if initChromosome is not None:
      self.data = list(initChromosome)
    else:
      self.data = []
      for i in range (0, 120):
        self.data.append( random.randint(0, 200) )

  def __str__(self):
    mystr = ''
    for i in self.data:
      mystr = mystr + str(i)
      mystr = mystr + ' '
    return mystr

  def loadFile(self, filename):
    f = open(filename)
    i = 0
    for line in f:
      self.data[i] = int(line)
      i = i+1
    f.close()
    self.filename = filename

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
    ''' returns the distance traveled in 30 seconds in meters '''
    command = ['../main',  '--disable-graphics',  '--load-coefs', './{}'.format(self.filename)]
    print command
    output = subprocess.check_output(command)
    print output

def initPopulation():
  random.seed()    
  # Make the directory
  os.mkdir("gen000")
  for i in range (0, 100):
    f = open('gen000/chromosome{}.txt'.format(str(i).zfill(3)), 'w')
    # Fill the chromosome with random stuff
    for j in range (0, 120):
      val = random.randint(0, 200)
      f.write('{} '.format(str(val)))
      f.write('\n')
    f.close()  

if __name__ == '__main__':
  # Load the population
  population = []
  for i in range (0, 100):
    c = Chromosome()
    c.loadFile('gen000/chromosome{}.txt'.format(str(i).zfill(3)))
    population.append(c)
  print len(population)
  for p in population:
    p.fitness()
