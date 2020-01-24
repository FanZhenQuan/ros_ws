#!/usr/bin/env python

import rospy
import subprocess
from tkinter import *

if __name__ == '__main__':
    root = Tk()
    root.title("Dump destinations")
    root.eval('tk::PlaceWindow %s center' % root.winfo_toplevel())

    pop = subprocess.Popen(["wmctrl", "-r", "'Dump destinations'", "-b", "add,above"])
    pop.communicate()
    
    lab = Label(root, text="")