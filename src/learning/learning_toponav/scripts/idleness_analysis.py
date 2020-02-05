#!/usr/bin/env python

import rospy
import numpy as np
import tkinter as tk
import tkMessageBox
import time
import subprocess
import pickle
import tkFont

from destination import Destination
from prettytable import PrettyTable


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
        self.tk_root = tk.Tk()
    
    def show_confirm_gui(self):
        self.tk_root.title("Dump destinations")
        self.tk_root.geometry("320x200")
        self.tk_root.eval('tk::PlaceWindow %s center' % self.tk_root.winfo_toplevel())
        
        pop = subprocess.Popen(["wmctrl", "-r", "Dump destinations", "-b", "add,above"])
        pop.communicate()
        
        msg = "Do you want to save\n the observed idlenesses\n of the destinations?"
        font = tkFont.Font(family="Helvetica", size=14)
        
        label = tk.Label(self.tk_root, text=msg, font=font)
        label.pack(side="top", fill="both", expand=True, padx=20, pady=20)
        
        frame = tk.Frame(self.tk_root).pack(side="bottom", expand=True)
        
        button = tk.Button(frame, text="OK", command=lambda: self.write_statfile())
        button.pack(side="left", fill="none", expand=True, padx=5, pady=5)
        
        button = tk.Button(frame, text="Cancel", command=lambda: self.tk_root.destroy())
        button.pack(side="right", fill="none", expand=True, padx=5, pady=5)
        
        self.tk_root.mainloop()
    
    def write_statfile(self):
        # datetime = time.strftime("%d-%m@%H:%M", time.localtime())
        # filename = "%s-%s-%sbots.txt" % (datetime, self.environment, self.robots_num)
        #
        # lines = []
        # statistics = {}
        # total_visits = 0
        # for d in self.dest_list:
        #     d.force_shutdown()
        #
        #     idlenesses = d.get_stats()
        #     total_visits += len(idlenesses)
        #
        #     idlenesses_str = [i.get_estimate_index(_type=str) for i in idlenesses]
        #     line = d.name + ': ' + ', '.join(idlenesses_str) + '\n'
        #     lines.append(line)
        #
        #     prolongued_idl = [i.get_true() for i in idlenesses]
        #     statistics[d.name] = {
        #         'mean': round(np.mean(prolongued_idl), 3),
        #         'max': np.max(prolongued_idl),
        #         'min': np.min(prolongued_idl)
        #     }
        #
        # means = [d['mean'] for d in statistics.values()]
        # average_idl = round(np.mean(means), 3)
        # variance_average_idl = round(np.var(means), 3)
        #
        # separator = "-----------\n"
        # lines = sorted(lines)
        # lines.append(separator + json.dumps(statistics, indent=2) + '\n')
        # lines.append(separator + "Average idleness: %s\n" % average_idl)
        # lines.append(separator + "Variance idleness: %s\n" % variance_average_idl)
        # lines.append(separator + "Total visits: %s\n" % total_visits)
        #
        # file = open(self.path+filename, 'w')
        # file.writelines(lines)
        datetime = time.strftime("%d-%m@%H:%M", time.localtime())
        filename = "%s-%s-%sbots.txt" % (datetime, self.environment, self.robots_num)
        
        self.write_dumpfile(filename)
        
        lines = []
        pt = PrettyTable()
        pt.field_names = ['Dest name', 'true', 'remaining', 'estimated', 'mean', 'max', 'min']
        
        total_visits = 0
        means = []
        for d in sorted(self.dest_list, key=lambda dest: dest.name):
            # if len(d.get_stats()) == 0:
            #     d.force_shutdown()
            #     idlenesses = d.get_stats()
            # else:
            d.force_shutdown()
            idlenesses = d.get_stats()
            idlenesses = d.get_stats()
            true_idls = [i.get_true() for i in idlenesses]
            
            # try:
            # if len(idlenesses) > 0:
            mean = round(np.mean(true_idls), 3)
            min = round(np.min(true_idls), 3)
            max = round(np.max(true_idls), 3)
        
            means.append(mean)
            total_visits += d.get_visits_num()
            
            for i in range(len(idlenesses)):
                if i == 0:
                    pt.add_row([
                        d.name, idlenesses[i].get_true(),
                        idlenesses[i].get_remaining(), idlenesses[i].get_estimated(), mean, max, min
                    ])
                else:
                    pt.add_row([
                        '', idlenesses[i].get_true(),
                        idlenesses[i].get_remaining(), idlenesses[i].get_estimated(), '', '', ''
                    ])
            # else:
            #     rospy.logerr("%s idlenesses empty" % d.name)
            # except ValueError:
            #     rospy.logwarn("Something wrong while calculating idlenesses stats")
            #     print true_idls
        
        separator = "-----------\n"
        lines.append(pt.__str__())
        lines.append("\nAverage idleness: %s\n" % round(np.mean(means), 3))
        lines.append(separator + "Variance idleness: %s\n" % round(np.var(means), 3))
        lines.append(separator + "Total visits: %s\n" % total_visits)
        
        file = open(self.path + filename, 'w')
        file.writelines(lines)
        
        rospy.loginfo('Destination idlenesses have been wrote to %s' % self.path)
        self.tk_root.destroy()
    
    def write_dumpfile(self, filename):
        name = filename.split('.')
        name.insert(1, '_DUMP.')
        _filename = ''.join(name)
        
        dests = []
        for d in sorted(self.dest_list, key=lambda dest: dest.name):
            dests.append(d)
        
        f = open(self.path + "dumps/" + _filename, 'w')
        pickle.dump(dests, f)
        f.close()


class IdlenessAnalizer(object):
    DEFAULT_PATH = IdlenessLogger.DEFAULT_PATH+"dumps/"
    
    def __init__(self, filename, path_to_pickled=DEFAULT_PATH):
        self.destinations = pickle.load(open(path_to_pickled+filename, 'rb'))

