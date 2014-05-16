#!/usr/bin/env python

from evoagent.agent import EvoAgent
from evoagent.agency import EvoAgency

class EvoAgentException(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)
