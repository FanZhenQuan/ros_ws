#!/usr/bin/env python

import rospy

from learning_toponav.msg import AfferenceDebug
from Tkinter import *
from ttk import Separator


class AfferenceDebugger(object):
    def __init__(self):
        self.root = Tk()
        self.root.title('Afference debug')
        self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())

        robot_1_lab = Label(self.root, text='Robot 1')
        robot_1_lab.grid(row=0, column=0)

        robot_2_lab = Label(self.root, text='Robot 2')
        robot_2_lab.grid(row=0, column=1)
        
        # ---------------
        


if __name__ == '__main__':
    rospy.init_node('afference_debugger')
    
    rospy.Subscriber('/afference_debug', AfferenceDebug, on_afference)