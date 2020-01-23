#!/usr/bin/env python

import rospy
import numpy as np
import tkMessageBox
import time
import json


class Idleness(object):
    def __init__(self, prolongued, aftch_idl, est_idl):
        self.__prolongued = prolongued
        self.__estimated = est_idl
        self.__afterchosen_idl = aftch_idl
    
    def get_prolongued(self):
        return self.__prolongued
    
    def get_afterchosen(self):
        return self.__afterchosen_idl

    def get_estimated(self):
        return self.__estimated
    
    def get_estimate_index(self, _type=float):  # TODO: rename
        if _type == float:
            return self.__afterchosen_idl / self.__estimated
        elif _type == str:
            return "%s/%s" % (self.__afterchosen_idl, self.__estimated)


# TODO: needs a refactor
class Destination(object):
    THRESHOLD = 1.5
    
    def __init__(self, name, pose, available=True):
        self.name = name
        self.pose = pose
        self.__ac_idleness = rospy.Time.now()
        self.est_idleness = 0
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
        elif(
                self.__latest_available is True and
                self.__available is False
        ):  # destination has been chosen, set ac_idleness
            self.__ac_idleness = rospy.Time.now()
        
    def get_prolongued_idleness(self):
        """
        :return: seconds elapsed since latest usage (type: float)
        """
        now = rospy.Time.now()
        return (now - self.__latest_usage).to_sec()
        
    def __append_idleness(self):
        prol_idl = self.get_prolongued_idleness()
        
        if prol_idl >= self.THRESHOLD:
            self.__stats.append(
                Idleness(
                    prolongued=prol_idl,
                    est_idl=self.est_idleness,
                    aftch_idl=(rospy.Time.now() - self.__ac_idleness).to_sec()
                )
            )
        
    def get_stats(self):
        return self.__stats
        
    def reset(self):
        self.__ac_idleness = rospy.Time.now()
        self.est_idleness = 0
        self.__latest_usage = rospy.Time.now()
        
    def force_shutdown(self):
        self.__append_idleness()
        self.reset()
        
    def __str__(self):
        return "Name: %s, status: %s, idleness: %s" %(self.name, self.available, self.get_prolongued_idleness())
    
    def __repr__(self):
        return "Dest %s" % self.name
    
    def __eq__(self, other):
        if isinstance(other, Destination):
            return self.name == other.name
        else:
            raise TypeError('%s is not of type <Destination>' % other)
    

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
        for d in self.dest_list:
            d.force_shutdown()
            
            idlenesses = d.get_stats()
            
            idlenesses_str = [i.get_estimate_index(_type=str) for i in idlenesses]
            line = d.name + ': ' + ', '.join(idlenesses_str) + '\n'
            lines.append(line)

            prolongued_idl = [i.get_prolongued() for i in idlenesses]
            statistics[d.name] = {
                'mean': np.mean(prolongued_idl),
                'max': np.max(prolongued_idl),
                'min': np.min(prolongued_idl)
            }
            
        means = [d['mean'] for d in statistics.values()]
        average_idl = np.mean(means)
        variance_average_idl = np.var(means)
        
        lines = sorted(lines)
        separator = "-----------\n"
        lines.append(separator + json.dumps(statistics, indent=2) + '\n')
        lines.append(separator + "Average idleness: %s\n" % average_idl)
        lines.append(separator + "Variance idleness: %s\n" % variance_average_idl)
        
        file = open(self.path+filename, 'w')
        file.writelines(lines)
