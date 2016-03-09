#!/usr/bin/env python3

import Pyro4

#Pyro4.config.COMMTIMEOUT=1.0
Pyro4.config.SERIALIZER='pickle'

def main():
    uri = input('Enter uri:')
    remote_ams = Pyro4.Proxy(uri)
    print(remote_ams.ping('yoyoyo'))
    return

if __name__ == "__main__":
    main()
