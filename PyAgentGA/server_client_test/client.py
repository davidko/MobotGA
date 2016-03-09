#!/usr/bin/env python3

import evoagent
import threading
import Pyro4
from test_migrate_agent import MyAgent

#Pyro4.config.COMMTIMEOUT=1.0
#Pyro4.config.SERIALIZER='pickle'

def main():
    uri = input('Enter uri:')
    remote_ams = Pyro4.Proxy(uri)
    print(remote_ams.ping())
    return

    ns = Pyro4.locateNS(host='localhost')
    hosts = ns.list(prefix='EvoAgency')
    agent = MyAgent('agent1')
    host, uri = hosts.popitem()
    print(uri)
    remote_ams = Pyro4.Proxy(uri)
    #remote_ams.add_agent(agent)
    print(remote_ams.ping())

if __name__ == "__main__":
    main()
