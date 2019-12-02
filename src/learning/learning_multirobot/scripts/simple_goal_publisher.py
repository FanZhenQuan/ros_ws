#!/usr/bin/env python

import rospy
import random
import subprocess
from Tkinter import *
import tkMessageBox as messagebox
from geometry_msgs.msg import PoseStamped, PointStamped, Twist


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
    
    def __init__(self):
        # finestra root
        self.root = Tk()
        self.root.title("Goal dispatcher")
        self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())
        
        self.place_above()
        
        img = Image("photo", file='/home/davide/ros_ws/icon.png')
        self.root.call('wm','iconphoto', self.root._w, img)
        
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
        target_robot_label = Label(self.root, text='Send to')
        target_robot_label.grid(row=5, column=0, padx=15, pady=25)
        
        self.send_goal_btn = Button(self.root, text="Send", bg="blue", fg="white", command=lambda: self.check_input())
        self.send_goal_btn.grid(row=5, column=3)
        
        self.target_robot = Entry(self.root, **ENTRY_WIDTH)
        self.target_robot.insert(0, 'red')
        self.target_robot.grid(row=5, column=1)
    
    def mainloop(self):
        self.root.mainloop()

    @staticmethod
    def place_above():
        pop = subprocess.Popen(["wmctrl", "-r", "Goal dispatcher", "-b", "add,above"])
        
        pop.communicate()
    
    def check_input(self):
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
        publ.publish(goal)
        
        cmd_vel_monitor = CmdVelMonitor(data['target_robot'] + '/cmd_vel')
        
        rospy.sleep(0.5)
        
        attempts = 3
        
        while attempts != 0:
            if cmd_vel_monitor.get_msgs_count() == 0:
                publ.publish(goal)
                rospy.logwarn('Goal resent')
                
                break
            attempts -= 1
            
        if attempts == 0:
            rospy.logerr('Topic ' + data['target_robot'] + '/cmd_vel' + " doesn't seem to be published.")
        
        del cmd_vel_monitor
    
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


class CmdVelMonitor(object):
    messages_cnt = 0
    
    def __init__(self, topic):
        rospy.Subscriber(topic, Twist, self.count)
        
    def count(self, msg):
        self.messages_cnt += 1
        
    def get_msgs_count(self):
        return self.messages_cnt


if __name__ == '__main__':
    gui = Gui()

    rospy.init_node('goal_publisher')
    rospy.on_shutdown(gui.close_window)
    
    try:
        gui.mainloop()
    except rospy.ROSInterruptException:
        gui.close_window()
