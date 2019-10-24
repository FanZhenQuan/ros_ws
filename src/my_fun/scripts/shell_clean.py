#!/usr/bin/env python

import subprocess
import psutil
import os
from pynput import keyboard


'''
TOO BAD THAT IT DOESN'T WORK, PROBABLY BECAUSE
KEY STROKES AREN'T RECORED PERFECTLY WHEN RUNNING
ROSCORE OR WHATEVER...
'''


def get_focused_window():

    w_id = subprocess.check_output(["xdotool", "getactivewindow"])
    name = subprocess.check_output(["xprop", "-id", w_id, "WM_CLASS"])

    return name.strip().split(' ')[3]


def on_press(key):
    try:
        # print (get_focused_window())
        if (
                key.char == 'c'
                and get_focused_window().startswith('"Gnome-terminal"')
        ):
            subprocess.call(['clear'])
    except AttributeError:
        pass


if __name__ == '__main__':
    try:
        with keyboard.Listener(on_press=on_press) as kb:
            kb.join()
    except KeyboardInterrupt:
        subprocess.call(['clear'])
        os._exit(1)
