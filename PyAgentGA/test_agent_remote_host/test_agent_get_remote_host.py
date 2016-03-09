#!/usr/bin/env python3

import evoagent
import threading
import Pyro4
from test_migrate_agent import MyAgent

Pyro4.config.COMMTIMEOUT=1.0
Pyro4.config.SERIALIZER='pickle'
Pyro4.config.SERIALIZERS_ACCEPTED=['json', 'marshal', 'serpent', 'pickle']

def main():
    agency = evoagent.EvoAgency()
    daemonThread = threading.Thread(target=agency.mainloop)
    daemonThread.start()
    agent = MyAgent('agent1')
    agency.add_agent(agent)

if __name__ == "__main__":
    main()
