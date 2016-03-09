#!/usr/bin/env python3

import evoagent
import threading
import Pyro4
from test_migrate_agent import MyAgent

#Pyro4.config.COMMTIMEOUT=1.0

class Blah():
    def ping(self):
        return 'ping'

def main():
    agency = evoagent.EvoAgency()
    #agency = Blah()
    #agency.mainloop()
    daemon = Pyro4.Daemon()
    uri = daemon.register(agency)
    print(uri)
    daemon.requestLoop()


if __name__ == "__main__":
    main()
