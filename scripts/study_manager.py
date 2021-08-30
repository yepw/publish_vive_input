#!/usr/bin/env python

import _osx_support
import numpy as np
import rospy
from std_msgs.msg import Bool, String, Int16
import math
import roslaunch
import rospkg
import rosnode
import subprocess
import random
import string
import yaml
import json
from datetime import datetime

def curr_time():
    now = rospy.get_rostime()
    time = { 'time' : datetime.fromtimestamp(now.secs), 
            'secs' : now.secs, 
            'nsecs' : now.nsecs }
    return time


def gen_user_id ():
    # https://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits?rq=1
    size = 4
    chars=string.ascii_uppercase + string.digits
    return ''.join(random.choice(chars) for _ in range(size))

class StudyManager():
    def __init__(self):
        self.user_id = gen_user_id()
        self.entire_study_start_time = curr_time()
        self.robot_state = "stopped"
        self.task_state = "finished"

        self.round_names = ["freeExplore",
                            "practice1",
                            "practice2",
                            "trial1",
                            "trial2",
                            "trial3"]
        self.totalRound = 5

        path_to_src = rospkg.RosPack().get_path('publish_vive_input')
        condition_order_file_path = path_to_src + '/configs/condition_orders.yaml'
        condition_order_file = open(condition_order_file_path, 'r')
        y = yaml.load(condition_order_file, Loader=yaml.FullLoader)

        condition_map = y["condition_map"]
        self.condition_order = y["condition_order"]
        
        self.round =  y["starting_round"]
        self.times = y["starting_times"]
        self.cdt_index = y["starting_cdt_index"]

        self.conditions = []
        for i, c in enumerate(self.condition_order):
            cdt = condition_map[c]
            self.conditions.append(cdt)


        rospy.Subscriber("/reset", Bool, self.reset_cb)
        rospy.Subscriber("/start", Bool, self.start_cb)
        rospy.Subscriber("/success", Bool, self.success_cb)
        rospy.Subscriber("/robot_state/clutching", Bool, self.clutching_cb)

        rospy.Subscriber("/failure", Bool, self.failure_cb)

        subprocess.Popen(["roslaunch", "mimicry_openvr", 
                        "mimicry_openvr.launch", "print_to_screen:=false"])

        self.spawn_robot_control(self.conditions[self.cdt_index])


    def saveTofile(self):
        filename = self.user_id
        filename += "_condition_" + str(self.cdt_index)
        filename += "_" + self.conditions[self.cdt_index]
        filename += "_round_" + self.round_names[self.round]
        filename += "_times_" + str(self.times)

        data = {}
        data['userId'] = self.user_id
        data['entire_study_start_time'] = self.entire_study_start_time
        data['task start time (user release cluch)'] = self.task_start_time
        data['task end time (drop into bin)'] = self.task_end_time
        data['task completion time'] = str( self.task_end_time['secs'] + self.task_end_time['nsecs'] * 1e-9 
                                            - self.task_start_time['secs'] - self.task_start_time['nsecs'] * 1e-9 ) + " s"

        data['cdt_index'] = self.cdt_index
        data['condition_order'] = self.condition_order
        data['current condition'] = self.conditions[self.cdt_index]
        data['current round'] = self.round
        data['current try times'] = self.times

        data['task_state'] = self.task_state

        with open( "/home/uwgraphics/inperson_study_logs/" + filename + '.yaml', 'w') as file:
            yaml.dump(data, file)

    def clutching_cb(self, msg):
        if msg.data == False and self.task_state == "prepare":
            self.task_start_time =  curr_time()
            self.task_state = "taking"
            print("User released cluch; task start!")

    def reset_cb(self, msg):
        if (self.round == 0):
            self.stop_robot_control()
            self.spawn_robot_control(self.conditions[self.cdt_index])

    def start_cb(self, msg):
        if self.robot_state == "running":
            self.stop_robot_control()

        if  self.task_state == "finished":
            self.round += 1
            self.times = 0
            if (self.round > self.totalRound ):
                self.cdt_index += 1
                self.round = 0
                if self.cdt_index > len(self.conditions):
                    print("finished everything")

        elif self.task_state == "failure":
            self.times += 1
        else: 
            print("error: unknow task state: " + self.task_state)
            return

        self.spawn_robot_control(self.conditions[self.cdt_index])
        self.task_state = "prepare"

        print("user ID: "  + str(self.user_id))
        print("current condition index: "  + str(self.cdt_index))
        print("current condition: "  + self.conditions[self.cdt_index])
        print("current round: "  + str(self.round))
        print("current times: "  + str(self.times))

    def success_cb(self, msg):
        self.task_end_time = curr_time()
        self.task_state = "finished"
        self.saveTofile()
        self.stop_robot_control()

    def failure_cb(self, msg):
        self.task_end_time = curr_time()
        self.task_state = "failure"
        self.saveTofile()
        self.stop_robot_control()
      
    def spawn_robot_control(self, control_mapping ):
        print("spawn_robot_control", control_mapping)
        subprocess.Popen(["roslaunch", "ros_server", 
                        "vive_sawyer.launch", "user_study_manager:=true", "control_mapping:="+control_mapping])
        rospy.sleep(2)
        self.robot_state = "running"
      
    def stop_robot_control(self):

        node_names = [
            "vive_input",
            "sawyer_control",
            "relaxed_ik",
        ]

        cmd = ["rosnode", "kill"]
        for name in node_names:
            cmd.append(name)

        subprocess.Popen(cmd)
        rospy.sleep(5)
        self.robot_state = "stopped"
  

if __name__ == '__main__':
   rospy.init_node('study_manager')
   serverManager = StudyManager()
   rospy.spin()
