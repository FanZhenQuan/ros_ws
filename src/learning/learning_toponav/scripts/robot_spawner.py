#!/usr/bin/env python

import rospy
import tkFont
import subprocess
import tkMessageBox as messagebox
import argparse
import yaml
import webcolors

from geometry_msgs.msg import PointStamped
from robot import Robot
from ttk import Separator
from tkinter import *


PADDING = {
    'padx': 5,
    'pady': 5
}
ENTRY_W = {
    'width': 15,
}


class RobotSpawner(object):
    def __init__(self, yaml_dir):
        self.robot_list = []
        self.some_colors = ['blue', 'yellow', 'green', 'purple', 'orange', 'cyan', 'gray']
        self.yaml_dir = yaml_dir
        self.yaml = yaml.load(open(yaml_dir))
        
        self.root = Tk()
        self.root.title("Robot spawner")
        self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())
        
        self.set_clicked_point_listener()
        self.place_above()
        
        row = 0
        title_font = tkFont.Font(family="Ubuntu", size=13, weight="bold")
        title_lab = Label(self.root, text="Robot details", font=title_font).grid(row=row, column=0, columnspan=4)
        
        # ----------- prima riga
        row += 1
        x_lab = Label(self.root, text="x").grid(row=row, column=0, **PADDING)
        self.x_ent = Entry(self.root, **ENTRY_W)
        self.x_ent.grid(row=row, column=1, **PADDING)
        
        ns_lab = Label(self.root, text="ns").grid(row=row, column=2, **PADDING)
        self.ns_ent = Entry(self.root, **ENTRY_W)
        self.ns_ent.insert(0, 'robot_1')
        self.ns_ent.grid(row=row, column=3, **PADDING)

        # ----------- seconda riga
        row += 1
        y_lab = Label(self.root, text="y").grid(row=row, column=0, **PADDING)
        self.y_ent = Entry(self.root, **ENTRY_W)
        self.y_ent.grid(row=row, column=1, **PADDING)
        
        color_lab = Label(self.root, text="color").grid(row=row, column=2, **PADDING)
        self.color_ent = Entry(self.root, **ENTRY_W)
        self.color_ent.insert(0, 'red')
        self.color_ent.grid(row=row, column=3, **PADDING)

        # ----------- terza riga
        # row += 1
        # z_lab = Label(self.root, text="z").grid(row=row, column=0, **PADDING)
        # self.z_ent = Entry(self.root, **ENTRY_W)
        # self.z_ent.grid(row=row, column=1, **PADDING)

        # ----------- quinta riga (una riga vuota)
        row += 2
        self.add_btn = Button(self.root, text="Add", bg="blue", fg="white", command=lambda: self.add_robot())
        self.add_btn.grid(row=row, column=3)
        
        # ----------- separator
        row += 1
        separator = Separator(self.root, orient="horizontal")
        separator.grid(row=row, columnspan=4, sticky="ew")
        
        # ----------- sesta riga
        row += 1
        robot_list_lab = Label(self.root, text="robots").grid(row=row, column=0, **PADDING)
        self.robot_list_text = Text(self.root, width=30, height=4)
        self.robot_list_text.grid(row=row, column=1)

        # ----------- settima riga
        row += 1
        self.add_btn = Button(self.root, text="Start", bg="green", fg="white", command=lambda: self.launch_robots())
        self.add_btn.grid(row=row, column=3)
        
    def mainloop(self):
        self.root.mainloop()
        
    def place_above(self):
        pop = subprocess.Popen(["wmctrl", "-r", "Robot spawner", "-b", "add,above"])
        pop.communicate()
        
    def add_robot(self):
        if(
            self.x_ent.get() != '' and
            self.y_ent.get() != '' and
            self.ns_ent.get() != ''
        ):
            try:
                color = webcolors.name_to_rgb(self.color_ent.get())
                color_str = "'%s %s %s 1'" % (float(color[0])/255, float(color[1])/255, float(color[2])/255)
                params = {
                    'x': round(float(self.x_ent.get()), 3),
                    'y': round(float(self.y_ent.get()), 3),
                    'z': 0,
                    'ns': self.ns_ent.get(),
                    'color': color_str,
                    'yaml': self.yaml_dir
                }
                
                self.robot_list.append(params)
                self.update_robot_list()
                self.clear_entries()
                self.insert_default()
            except ValueError as e:
                if e.message == "'' is not defined as a named color in css3":
                    messagebox.showerror('Error', "You didn't provide a color")
        else:
            messagebox.showerror('Error', 'Position or namespace invalid')
        
    def update_robot_list(self):
        namespaces = [r['ns'] for r in self.robot_list]
        
        self.robot_list_text.delete(1.0, 'end')
        text = '[%s]' % ', '.join(namespaces)
        self.robot_list_text.insert('end', text)
        
    def clear_entries(self):
        self.x_ent.delete(0, 'end')
        self.y_ent.delete(0, 'end')
        self.color_ent.delete(0, 'end')
        self.ns_ent.delete(0, 'end')
        
    def insert_default(self):
        self.ns_ent.insert(0, "robot_%s" % (len(self.robot_list) + 1))
        if len(self.some_colors) > 0:
            self.color_ent.insert(0, self.some_colors.pop(0))
        
    def set_clicked_point_listener(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_received)

    def clicked_point_received(self, msg):
        # clear the input values
        self.x_ent.delete(0, 'end')
        self.y_ent.delete(0, 'end')
    
        # insert the ones coming off the topic
        self.x_ent.insert(0, round(msg.point.x, 3))
        self.y_ent.insert(0, round(msg.point.y, 3))
        
    def launch_robots(self):
        calls = []
        call = ['roslaunch', 'learning_toponav', 'one_robot_manual.launch']
        
        colors_param = {}
        for r in self.robot_list:
            call.append("robot_name:=%s" % r['ns'])
            call.append("color:=%s" % r['color'])
            call.append("x_pos:=%s" % r['x'])
            call.append("y_pos:=%s" % r['y'])
            call.append("z_pos:=%s" % r['z'])
            call.append("config_yaml:='%s'" % r['yaml'])
            
            calls.append(call)
            
            call = ['roslaunch', 'learning_toponav', 'one_robot_manual.launch']
            colors_param[r['ns']] = r['color']
            
        for ns, color in colors_param.items():
            rospy.set_param(self.yaml['namespaces_topic']+ns, color)
        
        for c in calls:
            pop = subprocess.Popen(c)
            
        self.close_window()
        
    def close_window(self):
        try:
            self.root.destroy()
        except TclError:
            # window already close, ignore exception
            pass
    

if __name__ == '__main__':
    rospy.init_node('robot_spawner')
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaml', type=str)
    args, unknown = parser.parse_known_args()
    
    rs = RobotSpawner(yaml_dir=args.yaml)
    rospy.on_shutdown(rs.close_window)
    
    try:
        rs.mainloop()
    except rospy.ROSInterruptException:
        rs.close_window()