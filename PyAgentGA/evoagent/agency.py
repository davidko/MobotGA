#!/usr/bin/env python3
import Pyro4
import threading
import time
import random
import os

import logging

class AMQ(): # Agent message queue
    def __init__(self):
        self.messages = []
        self.cond = threading.Condition()

    def push(self, message):
        self.cond.acquire()
        self.messages.append(message)
        self.cond.notify_all()
        self.cond.release()

    def pop(self):
        self.cond.acquire()
        item = self.messages.pop(0)
        self.cond.release()
        return item

    def waitPop(self, timeout=None):
        self.cond.acquire()
        while len(self.messages == 0):
            self.cond.wait(timeout)
        item = self.messages.pop(0)
        self.cond.release()
        return item

class AMS():
    def __init__(self, name):
        self.name = name
        self.agents = {}
        self.agents_lock = threading.Condition()
        self.thread = threading.Thread(target=self._do_work)
        self.agent_message_queue = AMQ()
        self.log = logging.Logger('AMS')

    def add_agent(self, agent):
        self.log.debug('Got agent: ' + agent.get_id())
        self.agents_lock.acquire()
        if agent.get_id() in self.agents.keys():
            agent.set_id(self._new_id())
        agent._set_ams(self)
        self.agents[agent.get_id()] = agent
        self.agents_lock.notify_all()
        self.agents_lock.release()
        pass

    def get_remote_hosts(self):
        return self.ns.list(prefix='EvoAgency')

    def kill_agent(self, agentID):
        pass

    def __len__(self):
        return len(self.agents)

    def start_work(self):
        self.keep_alive = True
        if not self.thread.is_alive():
            self.thread.start()

    def ping(self):
        return "ping"

    def stop_work(self):
        self.keep_alive = False
        self.thread.join()

    def _do_work(self):
        while self.keep_alive:
            print('There are {0} agents.'.format(len(self.agents)))
            for (agentid, agent) in self.agents.items():
                agent.do_work()
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
    def __init__(self, name = None):
        self.ams = AMS(name)
        self.ams.start_work()
        self.name = name

    def add_agent(self, agent):
        return self.ams.add_agent(agent)

    def mainloop(self):
        # Start Pyro4 daemon
        if self.name is None:
            self.name = 'EvoAgency-' + str(os.getpid())
        self.daemon = Pyro4.Daemon()
        self.ns = Pyro4.locateNS(host='localhost' )
        uri = self.daemon.register(self.ams)
        print(uri)
        self.ns.register(self.name, uri)
        self.daemon.requestLoop()

    def ping(self):
        return 'ping'
        
        
