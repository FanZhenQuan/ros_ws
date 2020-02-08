#!/usr/bin/env python

import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
import tkMessageBox
import time
import subprocess
import pickle
import tkFont
from pprint import pprint

from destination import Destination
from prettytable import PrettyTable


DEFAULT_PATH = '/home/davide/ros_ws/src/learning/learning_toponav/idleness/'


class IdlenessLogger(object):
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
        datetime = time.strftime("%d-%m@%H:%M", time.localtime())
        subdir = "%s/%s/" % (self.environment, self.robots_num)
        filename = "%s-%s-%sbots.txt" % (datetime, self.environment, self.robots_num)
        
        self.write_dumpfile(filename=filename, env=self.environment, robots=str(self.robots_num))
        
        lines = []
        pt = PrettyTable()
        pt.field_names = ['Dest name', 'true', 'remaining', 'estimated', 'path_len', 'mean', 'max', 'min']
        
        total_visits = 0
        means = []
        for d in sorted(self.dest_list, key=lambda dest: dest.name):
            d.force_shutdown()
            
            observations = d.get_stats()
            true_idls = [o.idleness.get_true() for o in observations]
            
            # try:
            # if len(idlenesses) > 0:
            mean = round(np.mean(true_idls), 3)
            min = round(np.min(true_idls), 3)
            max = round(np.max(true_idls), 3)
        
            means.append(mean)
            total_visits += len(d.get_visits())
            
            for i in range(len(observations)):
                idl = observations[i].idleness
                if i == 0:
                    pt.add_row([
                        d.name, idl.get_true(),
                        idl.get_remaining(), idl.get_estimated(), observations[i].path_len, mean, max, min
                    ])
                else:
                    pt.add_row([
                        '', idl.get_true(),
                        idl.get_remaining(), idl.get_estimated(),observations[i].path_len, '', '', ''
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
        
        file = open(self.path + subdir + filename, 'w')
        file.writelines(lines)
        
        rospy.loginfo('Destination idlenesses have been wrote to %s' % self.path)
        self.tk_root.destroy()
    
    def write_dumpfile(self, filename, env, robots):
        name = filename.split('.')
        name.insert(1, '_DUMP.')
        _filename = ''.join(name)
        
        dests = []
        for d in sorted(self.dest_list, key=lambda dest: dest.name):
            dests.append(d)
        
        f = open(self.path+env+"/dumps/"+robots+'/'+_filename, 'w')
        pickle.dump(dests, f)
        f.close()


class IdlenessAnalizer(object):
    def __init__(self, maindir=DEFAULT_PATH):
        self.maindir = maindir
    
    @staticmethod
    def load(filename):
        """
        loads the serialized destinations from the path+filename provided
        :param filename: (str) name of the file
        :return: list of the deserialized destinations
        """
        if not filename.endswith("_DUMP.txt"):
            name = filename.split(".")
            return pickle.load(open(name[0]+"_DUMP.txt", 'rb'))
        
        return pickle.load(open(filename, 'rb'))
    
    def interferences(self):
        environments = os.listdir(self.maindir)
        environment_averages = []
    
        for env in sorted(environments):
            robots_num = os.listdir(self.maindir + env)
            robots_num.remove('dumps')
            robots_averages = []
        
            for robot in sorted(robots_num):
                runs = os.listdir(self.maindir + env + '/dumps/' + robot)
                runs_averages = []
                observs_num = 0
            
                for run in runs:
                    dests = self.load(self.maindir + env + '/dumps/' + robot + '/' + run)
                    observs = []
                
                    for d in dests:
                        if d.get_visits():  # != []
                            observs.extend(d.get_visits())
                
                    if observs:  # != []
                        observs_num += len(observs)
                        runs_averages.append(np.mean([o.get_interference() for o in observs]))
            
                if runs_averages:  # != []
                    runs_mean = round(np.mean(runs_averages), 4)
                    robots_averages.append({'robot_num': int(robot), 'value': runs_mean, 'observs': observs_num})
        
            if robots_averages:  # != []
                environment_averages.append({'environment': env, 'interferences': robots_averages})
    
        return environment_averages

    # def plot_average_interference(self):
        # plt.plot(x, y, 'ro')
        # plt.axis([0, 60, 0, 60])
        # plt.xlabel("Remaining idleness")
        # plt.ylabel("Estimated idleness")
        #
        # plt.show()

 
def debug():
    ia = IdlenessAnalizer()
    path = DEFAULT_PATH+"house/dumps/4/08-02@16:55-house-4bots_DUMP.txt"
    dests = ia.load(path)
    pprint([d.get_visits() for d in dests])


if __name__ == '__main__':
    ia = IdlenessAnalizer()
    environmental_interferences = ia.interferences()
    pprint(environmental_interferences)
    # debug()
