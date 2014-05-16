#!/usr/bin/env python3

from evoagent import EvoAgent, EvoAgentException
from evoagent.agent import Chromosome

import random
import tempfile
import subprocess
import math

class GaitChromosome(Chromosome):
    GENE_LENGTH=120
    def __init__(self, geneData=None):
        if geneData is not None and len(geneData) != 120:
            raise PyAgentGAException('Chromosome format incorrect.')
        if geneData is None:
            self.genes = [ random.randint(0, 255) \
                for _ in range(self.GENE_LENGTH)]
        else:
            self.genes = geneData
        pass

    def __str__(self):
        return '\n'.join(map(str, self.genes))

    def __getitem__(self, key):
        return self.genes[key]

    def set(self, data):
        self.genes = list(data)

    def crossover(self, other, cutpoint=None):
        pass

    def compute_fitness(self):
        # First, save fitness to a temporary file
        f = tempfile.NamedTemporaryFile()
        f.write(str(self).encode('utf-8'))
        command = [ '../main',  
                    '--disable-graphics',  
                    '--load-coefs', '{}'.format(f.name)]
        print(command)
        output = subprocess.check_output(command)
        positions = output.split()
        distance = 0.0;
        for p in positions:
          distance = distance + float(p)**2
        distance = math.sqrt(distance)
        print(distance)
        self.__fitness = distance
        f.close()
        return distance

