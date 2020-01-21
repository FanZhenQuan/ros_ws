#!/usr/bin/env python

import rospy
import tkinter as tk
import tkMessageBox
import time
import sys
import os
path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path)


class Destination(object):
    THRESHOLD = 3
    
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
        
    def get_idleness(self):
        """
        :return: seconds elapsed since latest usage (type: float)
        """
        now = rospy.Time.now()
        return (now - self.__latest_usage).to_sec()
        
    def __append_idleness(self):
        if self.get_idleness() >= self.THRESHOLD:
            self.__stats.append(self.get_idleness())
        
    def get_stats(self):
        return self.__stats
        
    def reset(self):
        self.idleness = 0
        self.__latest_usage = rospy.Time.now()
        
    def force_shutdown(self):
        self.__append_idleness()
        self.reset()
        
    def __str__(self):
        return "Name: %s, status: %s, idleness: %s" %(self.name, self.available, self.get_idleness())
    
    def __repr__(self):
        return "Dest %s" % self.name
    
    def __eq__(self, other):
        return self.name == other.name
    
    def __iter__(self):
        yield self.get_idleness()
    

class DestinationStatLogger(object):
    DEFAULT_PATH = '/home/davide/ros_ws/src/learning/learning_toponav/idleness/'
    
    def __init__(self, dest_list, environment, robots_num, path=DEFAULT_PATH):
        if all(isinstance(d, Destination) for d in dest_list):
            self.dest_list = dest_list
        else:
            raise ValueError("Items of dest_list aren't of type <Destination>")
        self.path = path
        self.environment = environment  # office, house ...
        self.robots_num = robots_num
        
    @staticmethod
    def show_confirm_gui():
        msg = 'Do you want to save the observed idlenesses of the destinations?'
        return tkMessageBox.askyesno('Dump destinations', msg)
        
    def write_statfile(self):
        datetime = time.strftime("%d-%m@%H:%M", time.localtime())
        filename = "%s-%s-%sbots.txt" % (datetime, self.environment, self.robots_num)
        
        lines = []
        for d in self.dest_list:
            d.force_shutdown()
            idlenesses_str = [str(i) for i in d.get_stats()]
            line = d.name + ': ' + ', '.join(idlenesses_str) + '\n'
            lines.append(line)
        lines = sorted(lines)
        
        file = open(self.path+filename, 'w')
        file.writelines(lines)