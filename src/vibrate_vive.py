#!/usr/bin/python3

import rospy
import socket
from std_msgs.msg import String, Bool, Int32

class VibrateManager:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = rospy.get_param("~port", 8082)
        try:
            self.socket.connect(('', self.port))
        except:
            print("Could not connect to socket.")
            exit()

        # self.command_sub = rospy.Subscriber("/vibrate_vive/command", String, self.publishCommand)
        self.vibrate_sub = rospy.Subscriber("/vibrate_vive/vibrate", Bool, self.handleVibration)
        self.pulse_time_sub = rospy.Subscriber("/vibrate_vive/pulse_time", Int32, self.changePulseTime)
        

    # def publishCommand(self, msg: String):
    #     self.socket.send(msg.data.encode('utf-8'))

    def handleVibration(self, msg: Bool):
        if msg.data:
            self.socket.send(b"vibrate")

    def changePulseTime(self, msg: Int32):
        self.socket.send(b"pulse_time:" + str(msg.data).encode('utf-8'))

if __name__ == '__main__':
    rospy.init_node("vibrate_vive", anonymous=True)
    mgr = VibrateManager()

    rospy.spin()

