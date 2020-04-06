#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import rosparam
import yaml
import traceback
def check_shutdown():
	if rospy.is_shutdown():
		exit()
class Manager():
	def __init__(self):
		rospy.init_node('AIESEC_server', anonymous=True)
	def start(self):
		self.husky_s = rospy.Subscriber('husky/finish',Int32,self.husky_get)
		self.husky_p= rospy.Publisher('husky/command',Int32,queue_size=10)
		self.arm_s = rospy.Subscriber('arm/finish',Int32,self.arm_get)
		self.arm_p = rospy.Publisher('arm/command',Int32,queue_size=10)
		time.sleep(1)
		self.husky_reply = -1
		self.arm_reply = -1
		'''
		while not rospy.is_shutdown():
			pass
		'''
		##########################################
		#             step 0        (home)       #
		##########################################
		self.arm_p.publish(Int32(data=1))              # 1 for home place
		time.sleep(9)
		rospy.loginfo("Set up fine")

		##########################################
		#            step 1         (to place 1) #
		##########################################
		self.arm_p.publish(2)                         # 2 for standby place
		while not (self.arm_reply == 2):
			check_shutdown()
			time.sleep(0.5)
		rospy.loginfo("arm go home")
		###########################################
		#            step 2         (catch ball)  #
		###########################################
		self.husky_p.publish(Int32(data = 2))          # 2 for point 1
		while not (self.husky_reply == 2 ):
			check_shutdown()
			time.sleep(0.5)
		rospy.loginfo("arived point 1")
		###########################################
		#            step 3         (catch ball)  #
		###########################################
		self.arm_p.publish(Int32(data = 4))           # 4 for catch ball  
		while not self.arm_reply == 4:
			check_shutdown()
			time.sleep(0.5)
		rospy.loginfo("gocha!")
		###########################################
		#            step 4         (to place 2)  #
		###########################################
		self.husky_p.publish(Int32(data = 4))           # 4 for point 2  
		while not self.husky_reply == 4:
			check_shutdown()
			time.sleep(0.5)
		rospy.loginfo("arrived point 2")
		###########################################
		#            step 5         (to put down the ball)  #
		###########################################
		self.arm_p.publish(Int32(data = 5))           # 5 for put ball down
		while not self.arm_reply == 5:
			check_shutdown()
			time.sleep(0.5)
		rospy.loginfo("put down the ball OK")

	def husky_get(self,data):
		self.husky_reply = data.data
	def arm_get(self, data):
		self.arm_reply = data.data
if __name__ == "__main__":
	try:
		mang=Manager()
		mang.start()
	except Exception as e:
		exstr = traceback.format_exc()
		print(exstr)