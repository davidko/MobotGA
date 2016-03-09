#!/usr/bin/env python3

import evoagent
import threading
import Pyro4
from test_migrate_agent import MyAgent
import time

#Pyro4.config.COMMTIMEOUT=1.0
Pyro4.config.SERIALIZER='pickle'
Pyro4.config.SERIALIZERS_ACCEPTED=['json', 'marshal', 'serpent', 'pickle']

class Blah(object):
    def __init__(self):
        self.remoteobjects = []
        thread = threading.Thread(target = self.thread)
        thread.start()

    def ping(self, remoteobject):
        print('Got remote object')
        self.remoteobjects.append(remoteobject)
        return 'hello'

    def thread(self):
        while True:
            for o in self.remoteobjects:
                print('Running object...')
                o.run()
            time.sleep(2)

def main():
    #agency = evoagent.EvoAgency()
    agency = Blah()
    #agency.mainloop()
    daemon = Pyro4.Daemon()
    uri = daemon.register(agency)
    print(uri)
    daemon.requestLoop()


if __name__ == "__main__":
    main()
