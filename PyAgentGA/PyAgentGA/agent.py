#!/usr/bin/env python3

import random
import subprocess
import tempfile

class Chromosome():
    """ Chromosome base class. All members defined here must be overridden
    by derived classes """
    def __init__(self):
        pass

    def __str__(self):
        pass

    def __getitem__(self, key):
        """ Get a gene or genes """
        pass

    def set(self, data):
        pass

    def crossover(self, other):
        """ Return Chromosome """
        pass

    def compute_fitness(self):
        """ Computer and store a fitness that can be retrieved via 
        get_fitness()"""
        pass

    def get_fitness(self):
        pass

class AgentException(Exception):
    def __init__(self):
        Exception.__init__(self)

class Agent():
    AGENT_EGG = 0
    AGENT_ALIVE = 1
    AGENT_DEAD = 2

    def __init__(self, agentId):
        self.status = Agent.AGENT_EGG
        self.chromosome = None
        self.agentId = agentId

    def get_id(self):
        return self.agentId

    def get_fitness(self):
        return self.__fitness

    def get_status(self):
        return self.status

    def set_ams(self, ams):
        self.ams = ams

    def set_id(self, agentId):
        self.agentId = agentId

    def set_chromosome(self, chromosome = None):
        self.chromosome = chromosome

    def compute_fitness(self):
        self.__fitness = self.chromosome.compute_fitness()

    def doWork(self):
        self.status = AGENT_ALIVE

