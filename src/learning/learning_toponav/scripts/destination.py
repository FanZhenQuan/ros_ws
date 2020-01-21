#!/usr/bin/env python

import rospy
import numpy as np
import tkMessageBox
import time
import json


class Destination(object):
    THRESHOLD = 3
    
    def __init__(self, name, pose, available=True):
        self.name = name
        self.pose = pose
        self.idleness = None
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
        statistics = {}
        separator = "-----------\n"
        for d in self.dest_list:
            d.force_shutdown()
            
            idlenesses = d.get_stats()
            idlenesses_str = [str(i) for i in idlenesses]
            line = d.name + ': ' + ', '.join(idlenesses_str) + '\n'
            lines.append(line)

            statistics[d.name] = {
                'mean': np.mean(idlenesses),
                'max': np.max(idlenesses),
                'min': np.min(idlenesses)
            }
            
        means = [d['mean'] for d in statistics.values()]
        average_idl = np.mean(means)
        variance_average_idl = np.var(means)
        
        lines = sorted(lines)
        lines.append(separator + json.dumps(statistics, indent=2))
        lines.append(separator + "Average idleness: %s\n" + average_idl)
        lines.append(separator + "Variance idleness: %s\n" + variance_average_idl)
        
        file = open(self.path+filename, 'w')
        file.writelines(lines)
