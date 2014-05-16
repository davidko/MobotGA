#!/usr/bin/env python3

import evoagent
import threading

class MyAgent(evoagent.EvoAgent):
    def __init__(self, *args, **kwargs):
        evoagent.EvoAgent.__init__(self, *args, **kwargs)

    def do_work(self):
        print(self.get_ams().get_remote_hosts())

def main():
    agency = evoagent.EvoAgency()
    daemonThread = threading.Thread(target=agency.mainloop)
    daemonThread.start()
    agent = MyAgent('agent1')
    agency.add_agent(agent)

if __name__ == "__main__":
    main()
