#!/usr/bin/env python3
import rospy
from keyboard.msg import Key
from std_msgs.msg import String

lst = [0,0,0,0,0,0,0,0,0,0] 						#key monitor index as follow q,a,w,s,e,d,r,f,t,g


def publish_data(message):
	encode_data = String()							
	message = str(message)
	message = message.replace(",","")
	message = message.replace(" ","")
	message = message.replace("[","")
	message = message.replace("]","")
	encode_data.data = message
	pub.publish(encode_data)

def key_down(msg):									#function to set detect key press
	global lst
	key_code = int(msg.code)								#msg.code is the ACII value of key char Ex. if a is press msg.code will return 97
	if (key_code == 113):
		lst[0] = 1;
	elif (key_code == 97):
		lst[1] = 1;
	elif (key_code == 119):
		lst[2] = 1;
	elif (key_code == 115):
		lst[3] = 1;
	elif (key_code == 101):
		lst[4] = 1;
	elif (key_code == 100):
		lst[5] = 1;
	elif (key_code == 114):
		lst[6] = 1;
	elif (key_code == 102):
		lst[7] = 1;
	elif (key_code == 116):
		lst[8] = 1;
	elif (key_code == 103):
		lst[9] = 1;
	#rospy.loginfo(lst)
	publish_data(lst)
	#rospy.loginfo(encode_data.data[1:-1])




def key_up(msg):									#Function to detect key release						
	global lst
	key_code = int(msg.code)						#msg.code is the ACII value of key char Ex. if a is press msg.code will return 97
	if (key_code == 113):
		lst[0] = 0;
	elif (key_code == 97):
		lst[1] = 0;
	elif (key_code == 119):
		lst[2] = 0;
	elif (key_code == 115):
		lst[3] = 0;
	elif (key_code == 101):
		lst[4] = 0;
	elif (key_code == 100):
		lst[5] = 0;
	elif (key_code == 114):
		lst[6] = 0;
	elif (key_code == 102):
		lst[7] = 0;
	elif (key_code == 116):
		lst[8] = 0;
	elif (key_code == 103):
		lst[9] = 0;
	rospy.loginfo(lst)
	publish_data(lst)


if __name__ == "__main__":
	rospy.init_node("encoder")
	key_down_sub = rospy.Subscriber("/keyboard/keydown", Key, key_down)
	key_up_sub = rospy.Subscriber("/keyboard/keyup",Key, key_up)
	pub = rospy.Publisher("/control_string", String, queue_size = 10)
	rospy.spin()