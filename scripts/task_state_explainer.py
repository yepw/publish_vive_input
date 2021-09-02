#!/usr/bin/env python
'''
Print the current tasj status on a pop-up message box

author: Yeping Wang
email: yepingw@outlook.com
'''

import yaml
import os
import rospy
from std_msgs.msg import Int16, String
import Tkinter as tk
import tkFont 
import threading

# size of the pop up message box
SCALE = 20

class TaskStateExplainer():
    window = tk.Tk()
    window.title("Study Progress")    
    var = tk.StringVar()
    font = tkFont.Font(family='Helvetica', size=24)
    
    tk.Label(window, textvariable=var, font = font).pack()
    window.geometry('+%d+%d' % (30*SCALE, 10*SCALE))
    window.minsize(width=30*SCALE, height=10*SCALE)
    window.maxsize(width=30*SCALE, height=10*SCALE)

    rospy.init_node('joystick_explainer')
    
    lock = threading.Lock()

    def __init__(self):    

        self.msg = ""

        rospy.Subscriber("/study_state", String, self.study_state_callback)
        
        # update every 0.1 s
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        
    def timer_callback(self, timer):
        """
        """
        s = self.msg

        self.var.set(s)
        # window.update_idletasks()

 
    def study_state_callback(self, msg):
        """
        """
        with self.lock:
            self.msg = msg.data

    def start(self):
        self.window.mainloop()


if __name__ == '__main__':
    task_state_explainer = TaskStateExplainer()
    task_state_explainer.start()
    # rospy.spin()