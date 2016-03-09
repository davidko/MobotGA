#!/usr/bin/env python3

import threading
import Pyro4

#Pyro4.config.COMMTIMEOUT=1.0
Pyro4.config.SERIALIZER='pickle'
Pyro4.config.SERIALIZERS_ACCEPTED=['json', 'marshal', 'serpent', 'pickle']

class Blah():
    def __init__(self):
        self.lastmsg = ''

    def ping(self, msg):
        self.lastmsg = msg
        return self.lastmsg + '<>' + msg

def main():
    agency = Blah()
    daemon = Pyro4.Daemon()
    uri = daemon.register(agency)
    print(uri)
    daemon.requestLoop()


if __name__ == "__main__":
    main()
