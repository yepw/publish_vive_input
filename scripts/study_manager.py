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
import os
import json
from datetime import datetime
import Tkinter as tk
import tkFont 
import time
import intera_interface

def null_time():
    time = { 'time' : datetime.fromtimestamp(0), 
            'secs' : 0, 
            'nsecs' : 0 }
    return time

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

# size of the pop up message box
SCALE = 20

class StudyManager():

    def __init__(self):

        self.time_limit = 0
        self.display_msg = ""

        self.user_id = gen_user_id()
        self.directory = "/home/uwgraphics/inperson_study_logs"
        self.entire_study_start_time = curr_time()

        ## robot_state: running or stopped
        self.robot_state = "stopped"

        ## task_state: prepare, taking, successm, or failure
        self.task_state = "prepare"

        # gripper
        rp = intera_interface.RobotParams()
        valid_limbs = rp.get_limb_names()
        if not valid_limbs:
            rp.log_message(("Cannot detect any limb parameters on this robot. "
                            "Exiting."), "ERROR")
            return
        self.original_deadzone = None
        try:
            self.gripper = intera_interface.Gripper(valid_limbs[0] + '_gripper')
        except (ValueError, OSError) as e:
            rospy.logerr("Could not detect an electric gripper attached to the robot.")
            self.clean_shutdown()
            return
        self.original_deadzone = self.gripper.get_dead_zone()
        self.pre_grasping_msg = False
        self.gripper_status = "open"

        self.round_names = ["training",
                            "freeExplore",
                            "practice1",
                            "practice2",
                            "trial1",
                            "trial2",
                            "trial3"]

        self.round_names_vis = ["Training",
                            "Explore",
                            "Practice Round 1",
                            "Practice Round 2",
                            "Trial 1",
                            "Trial 2",
                            "Trial 3"]
        self.totalRound = 6

        path_to_src = rospkg.RosPack().get_path('publish_vive_input')
        condition_order_file_path = path_to_src + '/configs/condition_orders.yaml'
        condition_order_file = open(condition_order_file_path, 'r')
        y = yaml.load(condition_order_file, Loader=yaml.FullLoader)

        condition_map = y["condition_map"]
        self.condition_order = y["condition_order"]
        
        self.round =  y["starting_round"]
        self.cdt_index = y["starting_cdt_index"]

        self.conditions = []
        for i, c in enumerate(self.condition_order):
            cdt = condition_map[c]
            self.conditions.append(cdt)
        
        self.study_state_pub = rospy.Publisher('study_state', String, queue_size=5)

        rospy.Subscriber("/reset", Bool, self.reset_cb)
        rospy.Subscriber("/start", Bool, self.start_cb)
        rospy.Subscriber("/success", Bool, self.success_cb)
        rospy.Subscriber("/failure", Bool, self.failure_cb)
        rospy.Subscriber("/robot_state/clutching", Bool, self.clutching_cb)
        rospy.Subscriber('/robot_state/grasping', Bool, self.gripper_cb)

        subprocess.Popen(["roslaunch", "mimicry_openvr", 
                        "mimicry_openvr.launch", "print_to_screen:=false"])

        self.start()

        self.task_start_time = null_time()
        self.start_recording_time = null_time()
        self.stop_recording_time = null_time()
        self.last_gripper_open_time = null_time()

        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)


    def timer_cb(self, event):
        if self.time_limit > 0 and self.task_state == "taking":
            self.pub_study_state(self.display_msg + "\n\n Time Remaining: " + str(self.time_limit) + " s")
            self.time_limit -= 1
        else:
            self.pub_study_state(self.display_msg)

    def pub_study_state(self, s):
            msg = String()
            msg.data = s
            self.study_state_pub.publish(msg)

    def gen_filename(self):
        filename = self.user_id
        filename += "_condition_" + str(self.cdt_index+1)
        filename += "_" + self.conditions[self.cdt_index]
        filename += "_" + self.round_names[self.round]
        filename += "_" 
        filename += ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(3))
        return filename

    def saveTofile(self):
        if (self.task_state == "prepare"):
            return

        # stop rosbag recording
        s = '/record'
        node_names = rosnode.get_node_names()
        for name in node_names:
            if (s in name):
                os.system("rosnode kill " + name)

        filename = self.gen_filename() + '.yaml'

        data = {}
        data['userId'] = self.user_id
        data['entire_study_start_time'] = self.entire_study_start_time

        data['task start time (user release cluch)'] = self.task_start_time
        data['task end time (drop into bin)'] = self.last_gripper_open_time
        data['task completion time'] = str( self.last_gripper_open_time['secs'] + self.last_gripper_open_time['nsecs'] * 1e-9 
                                            - self.task_start_time['secs'] - self.task_start_time['nsecs'] * 1e-9 ) + " s"

        data['start reocording time'] = self.start_recording_time
        data['stop reocording time'] = self.stop_recording_time

        data['cdt_index'] = self.cdt_index
        data['condition_order'] = self.condition_order
        data['current condition'] = self.conditions[self.cdt_index]
        data['current round'] = self.round

        data['task_state'] = self.task_state

        with open( self.directory + '/' + filename, 'w') as file:
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

    def start(self):
        if self.task_state == "taking":
            self.success()
        elif  self.robot_state == "running":
            self.stop_robot_control()

        self.spawn_robot_control(self.conditions[self.cdt_index])
        self.task_state = "prepare"

        print("user ID: "  + str(self.user_id))
        print("current condition index: "  + str(self.cdt_index))
        print("current condition: "  + self.conditions[self.cdt_index])
        print("current round: "  + str(self.round))

        if self.round == 0:
            self.display_msg = "Training"
            self.time_limit = 0
        else:
            self.display_msg = "Condition " + str(self.cdt_index + 1) + " out of 3" \
                    + "\n\n" + self.round_names_vis[self.round]

            self.time_limit = 90
            if (self.round == 1): 
                self.time_limit = 60

        self.start_recording_time = curr_time()

        bag_file_name = self.gen_filename() + ".bag"
        record_topics = '/tf /usb_cam/image_raw/compressed /usb_cam_2/image_raw/compressed /relaxed_ik/ee_pose_goals /relaxed_ik/joint_angle_solutions /vive_input/raw_data'

        command = "rosbag record -O " + self.directory + '/' + bag_file_name  + ' ' + record_topics
        print(command)
        self.bag = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)

    def start_cb(self, msg):
        self.start()

    def success(self):
        self.task_state = "finished"
        self.stop_robot_control()
        self.round += 1
        if (self.round > self.totalRound ):
            self.cdt_index += 1
            self.round = 1
            if self.cdt_index > len(self.conditions):
                print("finished everything")

    def success_cb(self, msg):
        self.success()

    def failure_cb(self, msg):
        self.task_state = "failure"
        self.stop_robot_control()
        self.round += 1
        if (self.round > self.totalRound ):
            self.cdt_index += 1
            self.round = 1
            if self.cdt_index > len(self.conditions):
                print("finished everything")
      
    def spawn_robot_control(self, control_mapping ):
        print("spawn_robot_control", control_mapping)
        subprocess.Popen(["roslaunch", "ros_server", 
                        "vive_sawyer.launch", "user_study_manager:=true", "control_mapping:="+control_mapping])
        rospy.sleep(2)
        self.robot_state = "running"
      
    def stop_robot_control(self):

        self.stop_recording_time = curr_time()

        self.time_limit = 0

        self.saveTofile()

        node_names = [
            "vive_input",
            "sawyer_control",
            "relaxed_ik",
            "sawyer_protector",
        ]

        cmd = ["rosnode", "kill"]
        for name in node_names:
            cmd.append(name)

        subprocess.Popen(cmd)
        rospy.sleep(5)
        self.robot_state = "stopped"

    def gripper_cb(self, msg):
        if (msg.data == True and self.pre_grasping_msg != True):
            if (self.gripper_status == "open"):
                self.gripper.close()
                self.gripper_status = "closed"
            else:
                self.gripper.open()
                self.last_gripper_open_time = curr_time()
                self.gripper_status = "open"
        self.pre_grasping_msg = msg.data
  

if __name__ == '__main__':
   rospy.init_node('study_manager')
   serverManager = StudyManager()
   rospy.spin()
