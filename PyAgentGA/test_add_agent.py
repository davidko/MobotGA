#!/usr/bin/env python3
from PyAgentGA import Agent
from PyAgentGA.agency import AMS
import time
import pdb

def main():
    ams = AMS()
    ams.start_work()
    ams.add_agent(Agent('agent1'))
    time.sleep(3)
    assert(len(ams) == 1)
    time.sleep(1)
    ams.add_agent(Agent('agent2'))
    assert(len(ams) == 2)
    time.sleep(3)
    ams.stop_work()
    
if __name__ == '__main__':
    main()

