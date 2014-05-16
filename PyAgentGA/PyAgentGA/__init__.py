#!/usr/bin/env python

from PyAgentGA.agent import Agent
from PyAgentGA.agency import Agency

class PyAgentGAException(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)
