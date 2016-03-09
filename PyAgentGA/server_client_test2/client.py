#!/usr/bin/env python3

import evoagent
import threading
import Pyro4
from test_migrate_agent import MyAgent
from MyObject import MyObject

#Pyro4.config.COMMTIMEOUT=1.0
Pyro4.config.SERIALIZER='pickle'

def main():
    uri = input('Enter uri:')
    remote_ams = Pyro4.Proxy(uri)
    m = MyObject()
    print(remote_ams.ping(m))
    return

    ns = Pyro4.locateNS(host='localhost')
    hosts = ns.list(prefix='EvoAgency')
    agent = MyAgent('agent1')
    host, uri = hosts.popitem()
    print(uri)
    remote_ams = Pyro4.Proxy(uri)
    #remote_ams.add_agent(agent)
    m = MyObject()
    print(remote_ams.ping(m))

if __name__ == "__main__":
    main()
