#!/usr/bin/env python

from paste_arduino.srv import DepositPaste
from std_msgs.msg import Byte
import rospy

def handle_deposit_paste(srv_msg):
	duration_pub.publish(8)
	now = rospy.Time.now()
	while rospy.Time.now() < now + srv_msg.duration:
		pass
	duration_pub.publish(0) 
	return DepositPasteResponse(True)



if __name__ == '__main__':
	rospy.init_node('paste_deposit_server')
	service = rospy.Service('paste_deposit', DepositPaste, handle_deposit_paste)

	duration_pub = rospy.Publisher('ROS_Valve_Commands', Byte, queue_size=1)

	rospy.spin()