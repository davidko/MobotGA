#!/usr/bin/env python3

from PyAgentGA import Agent
from gait import GaitChromosome

def main():
    agent = Agent('agent1')
    agent.set_chromosome(GaitChromosome())
    agent.compute_fitness()
    print(agent.get_fitness())

if __name__ == '__main__':
    main()
