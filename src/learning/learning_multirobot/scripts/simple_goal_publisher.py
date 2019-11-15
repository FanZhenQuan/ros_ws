#!/usr/bin/env python

import rospy
import sys
from Tkinter import *
import tkMessageBox as messagebox
from geometry_msgs.msg import PoseStamped


PADDING = {
    'padx': 15,
    'pady': 5
}
ENTRY_WIDTH = {
    'width': 10
}


class Gui(object):
    def __init__(self):
        # finestra root
        self.root = Tk()
        self.root.title("Goal dispatcher")
        self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())

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

        self.target_robot = Entry(self.root, **ENTRY_WIDTH)
        self.target_robot.insert(0, 'robot_1')
        self.target_robot.grid(row=5, column=1)

        self.send_goal_btn = Button(self.root, text="Send", bg="blue", fg="white", command=lambda: self.check_input())
        self.send_goal_btn.grid(row=5, column=3)

        # mainloop
        self.root.mainloop()
        
    def check_input(self):
        try:
            x_pos = self.floatify(self.x_pos.get())
            y_pos = self.floatify(self.y_pos.get())
            z_pos = self.floatify(self.z_pos.get())
            x_orient = self.floatify(self.x_orient.get())
            y_orient = self.floatify(self.y_orient.get())
            z_orient = self.floatify(self.z_orient.get())
            w_orient = self.floatify(self.w_orient.get())
            
            target_robot = str(self.target_robot.get()).strip('/ ')

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

            send_goal(goal)
        except Exception as e:
            messagebox.showerror('Check input error', e)
            print e.__class__

    def floatify(self, input):
        try:
            if input == '':
                return 0.0
            else:
                return float(input)
        except ValueError:
            messagebox.showerror('Floatify error', input + " e' una stringa malformata")
        

def send_goal(data):
    goal = PoseStamped()

    goal.header.seq = 0
    goal.header.frame_id = 'map'

    goal.pose.position.x = data['x_pos']
    goal.pose.position.y = data['y_pos']
    goal.pose.position.z = data['z_pos']

    goal.pose.orientation.x = data['x_orient']
    goal.pose.orientation.y = data['y_orient']
    goal.pose.orientation.z = data['z_orient']
    goal.pose.orientation.w = data['w_orient']

    pub = rospy.Publisher(data['target_robot'] + '/move_base_simple/goal', PoseStamped, queue_size=10)
    pub.publish(goal)
    
    # for key, val in data.items():
    #     print key + ': ' + str(val)


if __name__ == '__main__':
    rospy.init_node('dispatcher')
    
    try:
        g = Gui()
        rospy.spin()
    except KeyboardInterrupt:
        sys.exit(1)
