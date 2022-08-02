#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

class Movement:

	def __init__(self):
		self.pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=1)

		# Only linear x and angular z are relevant
		self.command = Twist()
		self.command.linear.x = 0.0
		self.command.angular.z = 0.0
		self.command.linear.y = 0.0			#do not change these values
		self.command.linear.z = 0.0			#do not change these values
		self.command.angular.x = 0.0		#do not change these values
		self.command.angular.y = 0.0		#do not change these values

	def move_forward(self):
		# Set command
		self.command.linear.x = 0.1
		self.command.angular.z = 0.0
		# Publish
		self.pub.publish(self.command)

	def move_backwards(self):
		# Set command
		self.command.linear.x = -0.1
		self.command.angular.z = 0.0
		# Publish
		self.pub.publish(self.command)

	def stop(self):
		# Set command
		self.command.linear.x = 0.0
		self.command.angular.z = 0.0
		# Publish
		self.pub.publish(self.command)

	def left_turn(self):
		# Set command
		self.command.angular.z = 0.2
		# Publish
		self.pub.publish(self.command)

	def right_turn(self):
		# Set command
		self.command.angular.z = -0.2
		# Publish
		self.pub.publish(self.command)

if __name__ == '__main__':
	rospy.init_node('move')
	base_motion = Movement()

	rate = rospy.Rate(1)
	
	##########FIX THIS SO IT DOESNT MOVE FORWARD FOREVER!!!! WE WANT ACTUAL MOTION CONTROLS##############
	while not rospy.is_shutdown():
		# Get input
		Base_motion = Movement()
		Base_motion.move_forward()
		rate.sleep()
		Base_motion.move_backwards()
		rate.sleep()
		Base_motion.left_turn()
		rate.sleep()
		Base_motion.right_turn()
		rate.sleep()









