#!/usr/bin/env python3

import evoagent
import threading
import Pyro4

class MyAgent(evoagent.EvoAgent):
    def __init__(self, *args, **kwargs):
        evoagent.EvoAgent.__init__(self, *args, **kwargs)

    def do_work(self):
        print(self.get_ams().get_remote_hosts())
        hosts = self.get_ams().get_remote_hosts()
        import random
        host = random.choice(list(hosts.keys()))
        remote_ams = Pyro4.Proxy(hosts[host])
        try:
            remote_ams.add_agent(self)
        except Pyro4.errors.CommunicationError as e:
            print(e)
            # Remove the offending host from the nameserver
            print('Removing host: {0}'.format(host))
            #self.ams.ns.remove(host)
        

