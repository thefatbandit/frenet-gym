import rospy
import carla
from agents.navigation.controller import VehiclePIDController
import math
from nav_msgs.msg import Path 

def callback_cmd_vel(data):
	"""
	Assigns the value of velocity from topic cmd_vel to tar_vel

	:param tar_vel: (float)
	:param data: (twist) 
	"""
	global tar_vel
	tar_vel = data.linear.x 


def start():
	rospy.init_node('controls', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)



if __name__ == '__main__':	
	start()