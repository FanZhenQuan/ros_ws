#!/usr/bin/env python
import sys
import os
path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path)


class Robot(object):
    def __init__(self, ns=None, color=None, state=None, c_goal=None, l_goal=None, aff=None, dist=None):
        if not ns or ns == '':
            raise Exception('Robot is being created without a valid namespace')
        self.ns = ns
        self.color = color
        self.state = state
        self.current_goal = c_goal
        self.latest_goal = l_goal
        self.afference = aff
        self.distance = dist
        
    def __str__(self):
        s = '''
    Robot name = %s
    Color = %s
    State = %s
    Current goal = %s
    Latest goal = %s
    Ipoint afference = %s
    Distance to afference = %s
    -------------------''' % (
        self.ns, self.color, self.state,
        self.current_goal, self.latest_goal,
        self.afference, self.distance
    )
        return s
    
    def __repr__(self):
        return self.ns

    def __eq__(self, other):
        if not isinstance(other, Robot):
            # don't attempt to compare against unrelated types
            raise Exception('Argument is not of type Robot')
    
        return self.ns == other.ns