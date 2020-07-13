import gym
import carla
import time         #Need to subscribe odometry and lidar data
import random		#Publish CarlaEgoVehicleContol 
import numpy as np
import os
import sys
import glob
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
from prius_msgs.msg import Control
from gym.envs.registration import register
from carla_msgs.msg import CarlaEgoVehicleControl
from gym import spaces
from gym.envs.registration import register
from tf.transformations import quaternion_from_euler
import rospy
import roslib
import carla
from sensor_msgs.msg import PointCloud2   
from nav_msgs.msg import Odometry

SECONDS_PER_EPISODE = 20



class CarlaEnv(gym.Env):

	def __init__(self):
		self.client = carla.Client("localhost", 2000)
		self.client.set_timeout(5.0) #Network operations timeout/ maximum allowed delay 
		self.world = self.client.get_world()
		self.blueprint_library = self.world.get_blueprint_library()
		self.action_space = spaces.Box(low=np.array([-1, 0, 0]), high=np.array([1, 1, 1]), dtype = np.float32)
		#self.observation_space = spaces.Box()
		#self.template = blueprint_library.filter("model3")[0] #  Just a particular blueprint or type of vehicle/template

	def reset(self):
		rospy.init_node('CarlaEnv')
		init_pub = rospy.Publisher('/carla/ego_vehicle/initialpose', PoseWithCovarianceStamped, queue_size = 10)
		#self.coll_hist = []
		#self.actor_list = []
		#print("Inside Reset")
		valid_transforms = self.world.get_map().get_spawn_points()
		self.init_transform = valid_transforms[0]
		#print("Inside Reset2")
		#self.actor_list.append(ego_vehicle)
		
		p = PoseWithCovarianceStamped()   #apply control to get immediate response
		#print("Inside Reset3")
		p.pose.pose.position.x = self.init_transform.location.x
		p.pose.pose.position.y = self.init_transform.location.y
		p.pose.pose.position.z = self.init_transform.location.z
		#print("Inside Reset4")
		quat = quaternion_from_euler(self.init_transform.rotation.roll, self.init_transform.rotation.pitch, self.init_transform.rotation.yaw)
		p.pose.pose.orientation.x = quat[0]
		p.pose.pose.orientation.y = quat[1]
		p.pose.pose.orientation.z = quat[2]
		p.pose.pose.orientation.w = quat[3]
		#print("Inside Reset5")
		p.header.stamp = rospy.Time.now()

		self.episode_start = time.time() 
		#print("Inside Reset6")
		print(p.pose.pose.position.x)
		print(p.pose.pose.position.y)
		print(p.pose.pose.position.z)
		print(p.pose.pose.orientation.x)
		print(p.pose.pose.orientation.y)
		print(p.pose.pose.orientation.z)
		print(p.pose.pose.orientation.w)
		init_pub.publish(p)
		#print("Inside Reset7")
		rospy.sleep(5)

		return None

	def callback(self, data):
		OBS.add_odom(data) 

	def callback1(self, data):
		OBS.add_lidar(data)

	def listener(self):
		rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, callback)
		rospy.Subscriber('/carla/ego_vehicle/lidar/<sensor>/point_cloud', PointCloud2, callback1)
		rospy.spin()

	def step(self, action):
		#print("step")
		# self.ego_vehicle.apply_control(carla.VehicleControl(throttle = action[0], steer = action[1], brake = action[2]))
		p = CarlaEgoVehicleControl()
		p.header.stamp = rospy.Time.now()
		p.throttle = action[0]
		p.steer = action[1] 
		p.brake = 0
		print(action[0])
		print(action[1])
		print(action[2])
		act_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size = 100)
		act_pub.publish(p)
		rospy.sleep(1)
		is_done = False

		if self.episode_start + SECONDS_PER_EPISODE < time.time():
			is_done = True
		reward = None

		return OBS.get_observation(), reward, is_done, None 

	# def close(self):
		# pass
# try:
# 	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
# 		sys.version_info.major,
# 		sys.version_info.minor,
# 		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError
# 	pass

# register(
# 	id='CarlaEnv-v0',
# 	entry_point=':custom_environments.envs:CarlaEnv'
# )