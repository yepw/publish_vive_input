#!/usr/bin/python3

import pygame

import rospy
from std_msgs.msg import Bool

class SoundPlayer:

    def __init__(self):
        pygame.mixer.init()

        self.assets_folder = rospy.get_param("~assets_folder", None)

        if self.assets_folder is None:
            rospy.signal_shutdown("Need assets folder location")
            return

        self.collision_sound = pygame.mixer.Sound(self.assets_folder + "buzzer_fail.wav")
        self.collision_sub = rospy.Subscriber("/play_sound/collision", Bool, self.collisionCb)


    def collisionCb(self, msg: Bool):
        if msg.data:
            self.collision_sound.play()


if __name__ == "__main__":
    rospy.init_node("sound_player", anonymous=True)
    player = SoundPlayer()
    rospy.spin()