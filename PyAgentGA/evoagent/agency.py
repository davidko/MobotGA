#!/usr/bin/env python3
import Pyro4
import threading
import time
import random

class AMS():
    def __init__(self):
        self.agents = {}
        self.agents_lock = threading.Condition()
        self.thread = threading.Thread(target=self._do_work)

    def add_agent(self, agent):
        self.agents_lock.acquire()
        if agent.get_id() in self.agents.keys():
            agent.set_id(self._new_id())
        self.agents[agent.get_id()] = agent
        self.agents_lock.notify_all()
        self.agents_lock.release()
        pass

    def kill_agent(self, agentID):
        pass

    def __len__(self):
        return len(self.agents)

    def start_work(self):
        self.keep_alive = True
        if not self.thread.is_alive():
            self.thread.start()

    def stop_work(self):
        self.keep_alive = False
        self.thread.join()

    def _do_work(self):
        while self.keep_alive:
            print('There are {0} agents.'.format(len(self.agents)))
            time.sleep(2)

    def _new_id(self):
        """ Generate and return a new valid Agent ID. Lock agents_lock first
        to ensure atomicity between creating the ID and adding the agent."""
        new_id = "Agent" + str(random.randint(0, 32000))
        while new_id in self.agents.keys:
            new_id = "Agent" + str(random.randint(0, 32000))
        return new_id

    def _set_AMS(self, ams):
        self.ams = ams

class EvoAgency():
    def __init__(self):
        self.ams = AMS()
        self.ams.start_work()
        
