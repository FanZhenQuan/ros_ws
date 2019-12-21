#!/usr/bin/env python

import rospy
import os
import yaml
import argparse
import random
import subprocess
import tkMessageBox as messagebox

from Tkinter import *
from ttk import Separator
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from learning_toponav.msg import RobotNavRequest, RobotAfference


CONFIG_DIR = '/home/davide/.simplegoalpublisher/'
CONFIG_FILE = 'config.txt'


PADDING = {
    'padx': 15,
    'pady': 5
}
ENTRY_WIDTH = {
    'width': 10
}


class Gui(object):
    publ = None
    robots = []
    
    def __init__(self, yaml):
        # finestra root
        self.yaml = yaml
        
        self.root = Tk()
        self.root.title("Goal dispatcher")
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        try:
            file = open(CONFIG_DIR + CONFIG_FILE, 'r')
            coords = file.readline()

            self.root.geometry(coords)
        except IOError:
            self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())
        
        self.place_above()
        
        img = Image("photo", file='/home/davide/ros_ws/icon.png')
        self.root.call('wm', 'iconphoto', self.root._w, img)
        
        self.clicked_point_listener()
        
        self.robots = get_robots()
        
        # ---------- prima riga
        position_label = Label(self.root, text='Position')
        position_label.grid(row=0, column=1, **PADDING)
        
        orient_label = Label(self.root, text='Orientation')
        orient_label.grid(row=0, column=3, **PADDING)
        
        # ---------- seconda riga
        x_pos_label = Label(self.root, text='x')
        x_pos_label.grid(row=1, column=0, **PADDING)
        
        self.x_pos = Entry(self.root, **ENTRY_WIDTH)
        self.x_pos.grid(row=1, column=1)
        
        x_orient_label = Label(self.root, text='x')
        x_orient_label.grid(row=1, column=2, **PADDING)
        
        self.x_orient = Entry(self.root, **ENTRY_WIDTH)
        self.x_orient.grid(row=1, column=3)
        
        # --------- terza riga
        y_pos_label = Label(self.root, text='y')
        y_pos_label.grid(row=2, column=0, **PADDING)
        
        self.y_pos = Entry(self.root, **ENTRY_WIDTH)
        self.y_pos.grid(row=2, column=1)
        
        y_orient_label = Label(self.root, text='y')
        y_orient_label.grid(row=2, column=2, **PADDING)
        
        self.y_orient = Entry(self.root, **ENTRY_WIDTH)
        self.y_orient.grid(row=2, column=3)
        
        # --------- quarta riga
        z_pos_label = Label(self.root, text='z')
        z_pos_label.grid(row=3, column=0, **PADDING)
        
        self.z_pos = Entry(self.root, **ENTRY_WIDTH)
        self.z_pos.grid(row=3, column=1)
        
        z_orient_label = Label(self.root, text='z')
        z_orient_label.grid(row=3, column=2, **PADDING)
        
        self.z_orient = Entry(self.root, **ENTRY_WIDTH)
        self.z_orient.grid(row=3, column=3)
        
        # --------- quinta riga
        w_orient_label = Label(self.root, text='w')
        w_orient_label.grid(row=4, column=2, **PADDING)
        
        self.w_orient = Entry(self.root, **ENTRY_WIDTH)
        self.w_orient.grid(row=4, column=3)
        
        # --------- sesta riga
        target_robot_label = Label(self.root, text='Send METRIC to')
        target_robot_label.grid(row=5, column=0, padx=15, pady=25)

        self.target_robot = Entry(self.root, **ENTRY_WIDTH)
        self.target_robot.insert(0, 'red')
        self.target_robot.grid(row=5, column=1)

        self.send_goal_btn = Button(self.root, text="Send", bg="blue", fg="white", command=lambda: self.check_metric_input())
        self.send_goal_btn.grid(row=5, column=3)
        
        # -------- separator
        separator = Separator(self.root, orient="horizontal")
        separator.grid(row=6, columnspan=4, sticky="ew")
        
        # -------- settima riga # TOPONAVIGATION
        target_topo_robot_label = Label(self.root, text='Send TOPO to')
        target_topo_robot_label.grid(row=7, column=0, padx=15, pady=25)

        self.target_topo_robot_ent = Entry(self.root, width=8)
        self.target_topo_robot_ent.insert(0, 'red')
        self.target_topo_robot_ent.grid(row=7, column=1)

        self.goal_topo = Entry(self.root, **ENTRY_WIDTH)
        self.goal_topo.grid(row=7, column=2)

        self.send_topogoal_btn = Button(self.root, text="Send", bg="green", fg="white", command=lambda: self.check_topo_input())
        self.send_topogoal_btn.grid(row=7, column=3)
    
    def mainloop(self):
        self.root.mainloop()

    @staticmethod
    def place_above():
        pop = subprocess.Popen(["wmctrl", "-r", "'Goal dispatcher'", "-b", "add,above"])
        # pop = subprocess.Popen(['wmctrl', '-r', '0x03c00027', '-b', 'toggle,above'])
        pop.communicate()
    
    def check_metric_input(self):
        try:
            x_pos = self.to_float(self.x_pos.get())
            y_pos = self.to_float(self.y_pos.get())
            z_pos = self.to_float(self.z_pos.get())
            x_orient = self.to_float(self.x_orient.get())
            y_orient = self.to_float(self.y_orient.get())
            z_orient = self.to_float(self.z_orient.get())
            w_orient = self.to_float(self.w_orient.get())
            
            target_robot_ent = str(self.target_robot.get())
            
            target_robot = None
            
            # se non inizia con robot_, vuol dire che e'
            # stato passato un colore -> cerca tra i robot
            # salvati il ns del robot a partire dal suo colore
            if not target_robot_ent.startswith('robot_'):
                for r in self.robots:
                    if r[0] == target_robot_ent:
                        target_robot = r[1]
            else:
                target_robot = str(target_robot_ent).strip('/ ')
            
            goal = {
                'x_pos': x_pos,
                'y_pos': y_pos,
                'z_pos': z_pos,
                'x_orient': x_orient,
                'y_orient': y_orient,
                'z_orient': z_orient,
                'w_orient': w_orient,
                'target_robot': target_robot
            }
            
            self.send_goal(goal)
        except Exception as e:
            messagebox.showerror('Check input error', e)
            rospy.logerr('Error type: ' + str(e.__class__))
            
    def check_topo_input(self):
        target_robot_ent = self.target_topo_robot_ent.get()
        # se non inizia con robot_, vuol dire che e'
        # stato passato un colore -> cerca tra i robot
        # salvati il ns del robot a partire dal suo colore
        target_robot = None
        if not target_robot_ent.startswith('robot_'):
            for r in self.robots:
                if r[0] == target_robot_ent:
                    target_robot = r[1]
        else:
            target_robot = str(target_robot_ent).strip('/ ')

        # check if destination exists
        destination = self.goal_topo.get()
        if not destination.startswith('WayPoint'):
            destination = 'WayPoint'+destination

        ipoints = rospy.get_param(self.yaml['interest_points'])
        ipoint_is_valid = False
        for i in ipoints:
            if i['name'] == destination:
                ipoint_is_valid = True
                
        source = rospy.wait_for_message('/'+target_robot+self.yaml['afference'], RobotAfference).ipoint_name

        if ipoint_is_valid:
            self.send_topogoal(target_robot, source=source, dest=destination)
        else:
            messagebox.showerror('Interest point not found')
    
    @staticmethod
    def to_float(entry_input):
        try:
            if entry_input == '':
                return 0.0
            else:
                return float(entry_input)
        except ValueError:
            messagebox.showerror('Floatify error', entry_input + " e' una stringa malformata")
    
    @staticmethod
    def send_goal(data):
        goal = PoseStamped()
        
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = data['x_pos']
        goal.pose.position.y = data['y_pos']
        goal.pose.position.z = data['z_pos']
        
        if (
                data['x_orient'] == 0 and
                data['y_orient'] == 0 and
                data['z_orient'] == 0 and
                data['w_orient'] == 0
        ):
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = random.uniform(0, 6)
            goal.pose.orientation.w = 1.0
        
        else:
            goal.pose.orientation.x = data['x_orient']
            goal.pose.orientation.y = data['y_orient']
            goal.pose.orientation.z = data['z_orient']
            goal.pose.orientation.w = data['w_orient']
        
        publ = rospy.Publisher(data['target_robot'] + '/move_base_simple/goal', PoseStamped, queue_size=10)
        while publ.get_num_connections() < 1:
            rospy.sleep(0.1)
        publ.publish(goal)

    def send_topogoal(self, robot, source, dest):
        msg = RobotNavRequest()
        msg.robot_name = robot
        msg.source = source
        msg.dest = dest
        
        pub = rospy.Publisher(self.yaml['planner_requests'], RobotNavRequest, queue_size=10)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.3)
        pub.publish(msg)
    
    def clicked_point_listener(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_received)
    
    def clicked_point_received(self, msg):
        # clear the input values
        self.x_pos.delete(0, 'end')
        self.y_pos.delete(0, 'end')
        self.z_pos.delete(0, 'end')
        
        # insert the ones coming off the topic
        self.x_pos.insert(0, round(msg.point.x, 1))
        self.y_pos.insert(0, round(msg.point.y, 1))
        self.z_pos.insert(0, round(msg.point.z, 1))
    
    def close_window(self):
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
            # window already close, ignore exception
            pass


def get_robots():
    """
    ottiene dal parameter service tutti i parametri, filtra secondo
    il ns /colors/ e aggiunge alla lista robots una tupla contenente
        [0]: colore del robot (param name)
        [1]: ns del robot (param value)
    
    :return: robots, la lista delle tuple
    """
    parameters = rospy.get_param_names()
    
    robots = []
    
    for p in parameters:
        if str(p).startswith('/colors'):
            color = str(p).replace('/colors/', '')
            robot_name = str(rospy.get_param(p))
            
            robots.append((color, robot_name))
            
    return robots


def load_yaml():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--yaml', type=str, required=False)
        
        args, unknown = parser.parse_known_args()
        f = open(args.yaml, 'r')
        return yaml.load(f)
    except:
        return None


if __name__ == '__main__':
    yaml = load_yaml()
    gui = Gui(yaml=yaml)

    rospy.init_node('goal_publisher')
    rospy.on_shutdown(gui.close_window)
    
    try:
        gui.mainloop()
    except rospy.ROSInterruptException:
        gui.close_window()
