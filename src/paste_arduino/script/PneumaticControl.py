#!/usr/bin/env python

from paste_arduino.srv import DepositPaste
from std_msgs.msg import Byte
import rospy

# Integer equivalants of required bitmasks
hold_tool = 2
drop_tool = 1
paste_dispense = 18

def handle_deposit_paste(srv_msg):

	if srv_msg.whatAction == 0:

		pneumatic_pub.publish(hold_tool)
		return DepositPasteResponse(True)

	elif srv_msg.whatAction == 1:

		pneumatic_pub.publish(drop_tool)
		return DepositPasteResponse(True)

	elif srv_msg.whatAction == 2:

		pneumatic_pub.publish(paste_dispense)

		now = rospy.Time.now()
		while rospy.Time.now() < now + srv_msg.duration:
			pass

		pneumatic_pub.publish(hold_tool) 
		return DepositPasteResponse(True)

	else:
		
		print('Error, action can not be taken')
		return DepositPasteResponse(False)



if __name__ == '__main__':
	rospy.init_node('Pneumatic_Control_Server')
	service = rospy.Service('Pneumatic_Control', DepositPaste, handle_deposit_paste)

	pneumatic_pub = rospy.Publisher('ROS_Valve_Commands', Byte, queue_size=1)

	rospy.spin()

