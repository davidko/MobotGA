#!/usr/bin/env python3

from evoagent import EvoAgent
from gait import GaitChromosome

def main():
    agent = EvoAgent('agent1')
    agent.set_chromosome(GaitChromosome())
    agent.compute_fitness()
    print(agent.get_fitness())

if __name__ == '__main__':
    main()
