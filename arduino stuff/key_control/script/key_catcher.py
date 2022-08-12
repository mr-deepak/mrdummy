#!/usr/bin/env python3
import rospy
from keyboard.msg import Key

def callback(msg):
	rospy.loginfo(str(msg.code))


if __name__ == "__main__":
	rospy.init_node("key_catcher")
	sub = rospy.Subscriber("/keyboard/keydown", Key, callback)
	rospy.spin()