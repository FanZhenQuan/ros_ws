#!/usr/bin/env python

import rospy
import sys
import os
path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path)


class Destination(object):
    
    def __init__(self, name, available=True):
        self.name = name
        self.available = available
        self.idleness = 0
        self.__latest_usage = rospy.Time.now()
        
    def toggle(self):
        self.available = not self.available
        
    def get_idleness(self, travel_time=None):
        """
        :return: seconds elapsed till the latest usage (type: float)
        """
        now = rospy.Time.now()
        if not travel_time:
            return (now - self.__latest_usage).to_sec()
        elif type(travel_time) == float or type(travel_time) == int:
            return (now - self.__latest_usage).to_sec() + travel_time
        else:
            msg = 'travel_time argument is of type %s, should be either float or int' % str(type(travel_time))
            raise Exception(msg)
        
    def __str__(self):
        return "Name: %s, status: %s, idleness: %s" %(self.name, self.available, self.idleness)
    
    def __eq__(self, other):
        return self.name == other.name

    def __le__(self, other):
        if isinstance(other, Destination):
            this = self.get_idleness()
            oth = other.get_idleness()
            
            return this <= oth
        
    def __ge__(self, other):
        if isinstance(other, Destination):
            this = self.get_idleness()
            oth = other.get_idleness()
        
            return this >= oth