#!/usr/bin/env python

import rospy
import os
import argparse
import yaml

from learning_toponav.msg import *
from Tkinter import *
from ttk import Separator

CONFIG_DIR = '/home/davide/.dest_debugger/'
CONFIG_FILE = 'config.txt'
FRAME_UI = {
    'padx': 10,
    'pady': 10,
    'borderwidth': 2,
    'relief': 'raised',
    'width': 10,
    'height': 10,
    'bg': 'green'
}
SMALLER = {
    'padx': 10,
    'pady': 10,
    'borderwidth': 2,
    'relief': 'raised',
    'width': 7,
    'height': 7
}


class DestDebugger(object):
    def __init__(self, yaml, dest_count):
        self.yaml = yaml
        
        self.root = Tk()
        self.root.title("Destinations debugger")
        try:
            file = open(CONFIG_DIR + CONFIG_FILE, 'r')
            coords = file.readline()
        
            self.root.geometry(coords)
        except IOError:
            self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())
    
        row = 0
        col = 0
        base = 2
        for i in range(1, dest_count+1):
            frame = Frame(self.root, name='waypoint'+str(i), **FRAME_UI)
            frame.grid(row=row, column=col)
        
            col += 1
            if col == base:
                row += 1
                col = 0
        
            l = Label(frame, text='WayPoint'+str(i))
            l.grid(row=0, column=0)

        # -------- separator
        row += 1
        separator = Separator(self.root, orient="horizontal")
        separator.grid(row=row, columnspan=base, pady=5, sticky="ew")
        
        # -------- ROBOT NAMES
        row += 1
        robot_1_lab = Label(self.root, text='Robot 1')
        robot_1_lab.grid(row=row, column=0)

        robot_2_lab = Label(self.root, text='Robot 2')
        robot_2_lab.grid(row=row, column=1)
        
        # --------- CURRENT GOAL
        row += 1
        robot_1_cframe = Frame(self.root, bg='green', **SMALLER)
        robot_1_cframe.grid(row=row, column=0)
        self.robot_1_cgoal = Entry(robot_1_cframe)
        self.robot_1_cgoal.grid(row=0, column=0)

        robot_2_cframe = Frame(self.root, bg='green', **SMALLER)
        robot_2_cframe.grid(row=row, column=1)
        self.robot_2_cgoal = Entry(robot_2_cframe)
        self.robot_2_cgoal.grid(row=0, column=0)

        # --------- LATEST GOAL
        row += 1
        robot_1_lframe = Frame(self.root, bg='gray', **SMALLER)
        robot_1_lframe.grid(row=row, column=0)
        self.robot_1_lgoal = Entry(robot_1_lframe)
        self.robot_1_lgoal.grid(row=0, column=0)

        robot_2_lframe = Frame(self.root, bg='gray', **SMALLER)
        robot_2_lframe.grid(row=row, column=1)
        self.robot_2_lgoal = Entry(robot_2_lframe)
        self.robot_2_lgoal.grid(row=0, column=0)
        
        # -------- STATE
        row += 1
        robot_1_sframe = Frame(self.root, bg='gray', **SMALLER)
        robot_1_sframe.grid(row=row, column=0)
        self.robot_1_state = Entry(robot_1_sframe)
        self.robot_1_state.grid(row=0, column=0)

        robot_2_sframe = Frame(self.root, bg='gray', **SMALLER)
        robot_2_sframe.grid(row=row, column=1)
        self.robot_2_state = Entry(robot_2_sframe)
        self.robot_2_state.grid(row=0, column=0)
        
        # -------- AFFERENCE
        row += 1
        robot_1_aframe = Frame(self.root, bg='blue', **SMALLER)
        robot_1_aframe.grid(row=row, column=0)
        self.robot_1_aff = Entry(robot_1_aframe)
        self.robot_1_aff.grid(row=0, column=0)

        robot_2_aframe = Frame(self.root, bg='blue', **SMALLER)
        robot_2_aframe.grid(row=row, column=1)
        self.robot_2_aff = Entry(robot_2_aframe)
        self.robot_2_aff.grid(row=0, column=0)
        
    def on_shutdown(self):
        try:
            x = self.root.winfo_x()
            y = self.root.winfo_y()
            
            if not os.path.exists(CONFIG_DIR):
                os.makedirs(CONFIG_DIR)
            
            with open(CONFIG_DIR + CONFIG_FILE, 'w') as f:
                line = '+%s+%s' % (x, y)
                f.write(line)
                self.root.destroy()
        except TclError:
            pass
    
    def mainloop(self):
        self.root.mainloop()
    
    def listen_destinations(self):
        rospy.Subscriber(self.yaml['destinations_log'], DestinationDebug, self.on_destination)
    
    def on_destination(self, msg):
        name = str(msg.name).lower()
        frame = self.root.nametowidget(name)
        
        if msg.available:
            frame['bg'] = 'green'
        else:
            frame['bg'] = 'red'
            
    def robot_states(self):
        rospy.Subscriber('/robot_1'+self.yaml['robot_state'], RobotState, self.on_state)
        rospy.Subscriber('/robot_2'+self.yaml['robot_state'], RobotState, self.on_state)
        
    def on_state(self, msg):
        if msg.robot_name == '/robot_1':
            self.robot_1_cgoal.delete(0, 'end')
            self.robot_1_lgoal.delete(0, 'end')
            
            self.robot_1_cgoal.insert(0, msg.current_goal)
            self.robot_1_lgoal.insert(0, msg.latest_goal)
            
            self.robot_1_state.delete(0, 'end')
            self.robot_1_state.insert(0, msg.state)
        else:
            self.robot_2_cgoal.delete(0, 'end')
            self.robot_2_lgoal.delete(0, 'end')
            
            self.robot_2_cgoal.insert(0, msg.current_goal)
            self.robot_2_lgoal.insert(0, msg.latest_goal)
            
            self.robot_2_state.delete(0, 'end')
            self.robot_2_state.insert(0, msg.state)
            
    def robot_afference(self):
        rospy.Subscriber('/robot_1' + self.yaml['afference'], RobotAfference, self.on_afference)
        rospy.Subscriber('/robot_2' + self.yaml['afference'], RobotAfference, self.on_afference)
        
    def on_afference(self, msg):
        if msg.robot_name == 'robot_1':
            self.robot_1_aff.delete(0, 'end')
            self.robot_1_aff.insert(0, msg.ipoint_name)
        else:
            self.robot_2_aff.delete(0, 'end')
            self.robot_2_aff.insert(0, msg.ipoint_name)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaml', type=str)
    
    args, unknown = parser.parse_known_args()
    
    f = open(args.yaml, 'r')
    return yaml.load(f)
    

if __name__ == "__main__":
    rospy.init_node('destinations_debugger')
    
    yaml = parse_args()
    
    dest_debugger = DestDebugger(yaml=yaml, dest_count=7)
    rospy.on_shutdown(dest_debugger.on_shutdown)
    
    dest_debugger.listen_destinations()
    dest_debugger.robot_states()
    dest_debugger.robot_afference()
    dest_debugger.mainloop()
