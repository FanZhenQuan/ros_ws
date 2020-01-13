#!/usr/bin/env python

import rospy
import tkMessageBox
import time


class Destination(object):
    IDLENESS_THRESH = 3
    
    def __init__(self, name, available=True):
        self.name = name
        self.idleness = 0
        self.__stats = []  # stores every idleness registered
        self.__latest_available = True
        self.__available = available
        self.__latest_usage = rospy.Time.now()
        
    @property
    def available(self):
        return self.__available
    
    @available.setter
    def available(self, value):
        self.__latest_available = self.__available
        self.__available = value
        if(
            self.__latest_available is False and
            self.__available is True
        ):  # destination has been freed, reset idleness
            self.__append_idleness()
            self.reset()
        
    def get_idleness(self, travel_time=None):
        """
        :return: seconds elapsed since latest usage (type: float)
        """
        now = rospy.Time.now()
        if not travel_time:
            return (now - self.__latest_usage).to_sec()
        elif type(travel_time) == float or type(travel_time) == int:
            return (now - self.__latest_usage).to_sec() + travel_time
        else:
            msg = 'travel_time argument is of type %s, should be either float or int' % type(travel_time)
            raise Exception(msg)
        
    def __append_idleness(self):
        # evita di segnare come idleness tempi molto piccoli come 1 sec
        if self.get_idleness() >= self.IDLENESS_THRESH:
            self.__stats.append(self.get_idleness())
        
    def get_stats(self):
        return self.__stats
        
    def reset(self):
        self.idleness = 0
        self.__latest_usage = rospy.Time.now()
        
    def __str__(self):
        return "Name: %s, status: %s, idleness: %s" %(self.name, self.available, self.idleness)
    
    def __eq__(self, other):
        return self.name == other.name
    

class DestinationStatLogger(object):
    DEFAULT_PATH = '/home/davide/ros_ws/src/learning/learning_toponav/idleness/'
    
    def __init__(self, dest_list, environment, path=DEFAULT_PATH):
        if all(isinstance(d, Destination) for d in dest_list):
            self.dest_list = dest_list
        else:
            raise ValueError("Items of dest_list aren't of type <Destination>")
        self.path = path
        self.environment = environment  # office, house ...
        
    @staticmethod
    def show_confirm_gui():
        msg = 'Do you want to save the observed idlenesses of the destinations?'
        return tkMessageBox.askyesno('Dump destinations', msg)
        
    def write_statfile(self):
        datetime = time.strftime("%d-%m@%H:%M", time.localtime())
        filename = "%s-%s.txt" % (datetime, self.environment)
        
        lines = []
        for d in self.dest_list:
            idlenesses_str = [str(i) for i in d.get_stats()]
            line = d.name + ': ' + ', '.join(idlenesses_str) + '\n'
            lines.append(line)
        lines = sorted(lines)
        
        file = open(self.path+filename, 'w')
        file.writelines(lines)
