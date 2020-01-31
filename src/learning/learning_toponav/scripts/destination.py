#!/usr/bin/env python

import rospy
import numpy as np
import tkMessageBox
import time
import json
import subprocess

import tkFont
from tkinter import *


class Idleness(object):
    def __init__(self, true, remaining, estim):
        self.__true_idl = true
        self.__estim_idl = estim
        self.__remaining_idl = remaining
    
    def get_true(self):
        return self.__true_idl
    
    def get_remaining(self):
        return self.__remaining_idl

    def get_estimated(self):
        return self.__estim_idl
    
    def get_estimate_index(self, _type=float):
        """
        :param _type: type of return value
        :return: (str or float) ratio between remaining idleness and estimated idleness
        """
        if _type == float:
            return self.__remaining_idl / self.__estim_idl
        elif _type == str:
            return "%s/%s" % (self.__remaining_idl, self.__estim_idl)


class Destination(object):
    def __init__(self, name, pose, available=True):
        self.name = name
        self.pose = pose
        self.estim_idl = 0
        self.__latest_usage = rospy.Time.now()
        self.__remaining_idl = rospy.Time.now()
        self.__stats = []  # stores every idleness registered
        self.__available = available
        
    @property
    def available(self):
        return self.__available
    
    @available.setter
    def available(self, value):
        latest_available = self.__available
        self.__available = value
        if(
            latest_available is False and
            self.__available is True
        ):  # destination has been freed, reset idleness
            self.__append_idleness()
            self.reset()
        elif(
            latest_available is True and
            self.__available is False
        ):  # destination has been chosen, set ac_idleness
            self.__remaining_idl = rospy.Time.now()
        
    def get_true_idleness(self):
        """
        :return: seconds elapsed since latest usage (type: float)
        """
        now = rospy.Time.now()
        return (now - self.__latest_usage).to_sec()
        
    def __append_idleness(self):
        true_idl = self.get_true_idleness()
        
        if true_idl >= 0:
            self.__stats.append(
                Idleness(
                    true=true_idl,
                    estim=self.estim_idl,
                    remaining=(rospy.Time.now() - self.__remaining_idl).to_sec()
                )
            )
        
    def get_stats(self):
        return self.__stats
        
    def reset(self):
        self.estim_idl = 0
        self.__latest_usage = rospy.Time.now()
        self.__remaining_idl = rospy.Time.now()
        
    def force_shutdown(self):
        self.__append_idleness()
        self.reset()
        
    def __str__(self):
        return "Name: %s, status: %s, idleness: %s" %(self.name, self.available, self.get_true_idleness())
    
    def __repr__(self):
        return self.name
    
    def __eq__(self, other):
        if isinstance(other, Destination):
            return self.name == other.name
        else:
            raise TypeError('%s is not of type <Destination>' % other)


class IdlenessLogger(object):
    DEFAULT_PATH = '/home/davide/ros_ws/src/learning/learning_toponav/idleness/'
    
    def __init__(self, dest_list, environment, robots_num, path=DEFAULT_PATH):
        if all(isinstance(d, Destination) for d in dest_list):
            self.dest_list = dest_list
        else:
            raise ValueError("Items of dest_list aren't of type <Destination>")
        self.path = path
        self.environment = environment  # office, house ...
        self.robots_num = robots_num
        self.tk_root = Tk()
        
    def show_confirm_gui(self):
        self.tk_root.title("Dump destinations")
        self.tk_root.geometry("320x200")
        self.tk_root.eval('tk::PlaceWindow %s center' % self.tk_root.winfo_toplevel())

        pop = subprocess.Popen(["wmctrl", "-r", "Dump destinations", "-b", "add,above"])
        pop.communicate()

        msg = "Do you want to save\n the observed idlenesses\n of the destinations?"
        font = tkFont.Font(family="Helvetica", size=14)
        
        label = Label(self.tk_root, text=msg, font=font)
        label.pack(side="top", fill="both", expand=True, padx=20, pady=20)

        frame = Frame(self.tk_root).pack(side="bottom", expand=True)

        button = Button(frame, text="OK", command=lambda: self.write_statfile())
        button.pack(side="left", fill="none", expand=True, padx=5, pady=5)

        button = Button(frame, text="Cancel", command=lambda: self.tk_root.destroy())
        button.pack(side="right", fill="none", expand=True, padx=5, pady=5)

        self.tk_root.mainloop()
        
    def write_statfile(self):
        datetime = time.strftime("%d-%m@%H:%M", time.localtime())
        filename = "%s-%s-%sbots.txt" % (datetime, self.environment, self.robots_num)
        
        lines = []
        statistics = {}
        total_visits = 0
        for d in self.dest_list:
            d.force_shutdown()
            
            idlenesses = d.get_stats()
            total_visits += len(idlenesses)
            
            idlenesses_str = [i.get_estimate_index(_type=str) for i in idlenesses]
            line = d.name + ': ' + ', '.join(idlenesses_str) + '\n'
            lines.append(line)

            prolongued_idl = [i.get_true() for i in idlenesses]
            statistics[d.name] = {
                'mean': round(np.mean(prolongued_idl), 3),
                'max': np.max(prolongued_idl),
                'min': np.min(prolongued_idl)
            }
            
        means = [d['mean'] for d in statistics.values()]
        average_idl = round(np.mean(means), 3)
        variance_average_idl = round(np.var(means), 3)
        
        lines = sorted(lines)
        separator = "-----------\n"
        lines.append(separator + json.dumps(statistics, indent=2) + '\n')
        lines.append(separator + "Average idleness: %s\n" % average_idl)
        lines.append(separator + "Variance idleness: %s\n" % variance_average_idl)
        lines.append(separator + "Total visits: %s\n" % total_visits)
        
        file = open(self.path+filename, 'w')
        file.writelines(lines)

        rospy.loginfo('Destination idlenesses have been wrote to %s' % self.path)
        self.tk_root.destroy()
