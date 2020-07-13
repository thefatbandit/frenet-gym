import sys
print("XXXXXXXXXXXXXXXx")
for p in sys.path:
	print(p)
print("XXXXXXXXXXXXXXXx")
import gym
import carla
import time         #Need to subscribe odometry and lidar data
import random		#Publish CarlaEgoVehicleContol 
import numpy as np
import os
import glob
import cv2
import roslib								   #change name of published
import rospy                                   #change action space
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
# import tf
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy                                   #change test script
import carla
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from prius_msgs.msg import Control
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import PointCloud2		   #use Control_to_EgoVehicleControl.py  
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gym import spaces
from gym.envs.registration import register
# from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
import copy
from std_msgs.msg import String
from std_msgs.msg import Float64
from carla_msgs.msg import CarlaEgoVehicleInfo
from controller import VehiclePIDController
from carla import ActorList
from ackermann_msgs.msg import AckermannDrive

SECONDS_PER_EPISODE = 20
# information = CarlaEgoVehicleInfo()
#get all params from the launch file

class observation():
	def __init__(self):
		self.odom = Odometry()
		self.lidar = LaserScan()
		self.path = Path()

	def get_observation(self):
		return self.odom, self.lidar, self.path

	def add_odom(self, odom):
		self.odom = odom
		
	def add_lidar(self, lidar):
		self.lidar = lidar

	def add_path(self, path):
		self.path = path

	def ret_path(self):
		return self.path

	def ret_lidar(self):
		return self.lidar

	def ret_odom(self):
		return self.odom

class CarlaEnv(gym.Env):
	def __init__(self):
		print("INSIDE INIT")

		MAX_SPEED = rospy.get_param('/frenet_planner/path/max_speed')
		MAX_ACCEL = rospy.get_param('/frenet_planner/path/max_accel')
		MAX_CURVATURE = rospy.get_param('/frenet_planner/path/max_curvature')
		MAX_ROAD_WIDTH = rospy.get_param('/frenet_planner/path/max_road_width')
		D_ROAD_W = rospy.get_param('/frenet_planner/path/d_road_w')
		DT = rospy.get_param('/frenet_planner/path/dt')
		MAXT = rospy.get_param('/frenet_planner/path/maxt')
		MINT = rospy.get_param('/frenet_planner/path/mint')
		TARGET_SPEED = rospy.get_param('/frenet_planner/path/target_speed')
		D_T_S = rospy.get_param('/frenet_planner/path/d_t_s')
		N_S_SAMPLE = rospy.get_param('/frenet_planner/path/n_s_sample')
		ROBOT_RADIUS = rospy.get_param('/frenet_planner/path/robot_radius')
		MAX_LAT_VEL = rospy.get_param('/frenet_planner/path/max_lat_vel')
		MIN_LAT_VEL = rospy.get_param('/frenet_planner/path/min_lat_vel')
		D_D_NS = rospy.get_param('/frenet_planner/path/d_d_ns')
		MAX_SHIFT_D = rospy.get_param('/frenet_planner/path/max_shift_d')


		self.client = carla.Client("localhost", 2000)
		self.client.set_timeout(5.0) #Network operations timeout/maximum allowed delay 
		self.world = self.client.load_world('Town02')
		self.settings = self.world.get_settings()
		self.settings.no_rendering_mode = True
		self.world.apply_settings(self.settings)
		self.blueprint_library = self.world.get_blueprint_library()
		self.action_space = spaces.Box(low=np.array([TARGET_SPEED - D_T_S*N_S_SAMPLE, MINT, -MAX_ROAD_WIDTH, -MAX_LAT_VEL]), high=np.array([TARGET_SPEED + D_T_S*N_S_SAMPLE, MAXT, MAX_ROAD_WIDTH, MAX_LAT_VEL]), dtype = np.float32)
		
		model_3 = self.blueprint_library.filter("model3")[0]
		transform = carla.Transform(carla.Location(x=168.4, y=191.6, z=2))
		actor = self.world.spawn_actor(model_3, transform, attach_to=None)
		transform_1 = carla.Transform(carla.Location(x=151.7, y=188.3, z=2))
		actor_1 = self.world.spawn_actor(model_3, transform_1, attach_to=None)
		transform_2 = carla.Transform(carla.Location(x=156.5, y=187.9, z=4))
		actor_2 = self.world.spawn_actor(model_3, transform_2, attach_to=None)
		transform_3 = carla.Transform(carla.Location(x=138.0, y=187.5, z=2))
		actor_3 = self.world.spawn_actor(model_3, transform_3, attach_to=None)
		transform_4 = carla.Transform(carla.Location(x=131.6, y=204.8, z=2))
		actor_4 = self.world.spawn_actor(model_3, transform_4, attach_to=None)
		transform_5 = carla.Transform(carla.Location(x=136.0, y=216.9, z=4))
		actor_5 = self.world.spawn_actor(model_3, transform_5, attach_to=None)
		transform_6 = carla.Transform(carla.Location(x=151.1, y=240.3, z=2))
		actor_6 = self.world.spawn_actor(model_3, transform_6, attach_to=None)
		transform_7 = carla.Transform(carla.Location(x=178.7, y=236.9, z=2))
		actor_7 = self.world.spawn_actor(model_3, transform_7, attach_to=None)
		transform_8 = carla.Transform(carla.Location(x=196.9, y=210.0, z=10))
		actor_8 = self.world.spawn_actor(model_3, transform_8, attach_to=None)
		# tv = action.linear.x
		# Ti = action.linear.y
		# di = action.linear.z
		# di_d = action.angular.x
		# s0 = action.angular.y (CURRENTY NOT SAMPLED)
		print("LAUNCH ego_vehicle.")
		print("After launching ego vechicle launch ")
		print("roslaunch pointcloud_to_laserscan sample_node.launch")
		print("pure_pursuit_rl")
		print("roslaunch carla_ackermann_control carla_ackermann_control.launch")

		input("Press Enter to continue...")
		#self.template = blueprint_library.filter("model3")[0] #  Just a particular blueprint or type of vehicle/template

	def reset(self):
		print("INSIDE RESET")
		self.episode_start = time.time()

		#Generating global path.
		idd = 2
		path_id = String()
		path_id.data = str(idd)
		print("Path id to be published : " + path_id.data)
		path_id_pub = rospy.Publisher('path_id', String, queue_size = 10000)
		rospy.init_node('CarlaEnv')

		while not path_id_pub.get_num_connections() > 0:
			print("Waiting for connection with path_id. Run frenet_planner.")
		print(path_id_pub.get_num_connections())
		path_id_pub.publish(path_id)#publishing the id for frenet planner to use the same path.

		W_X = rospy.get_param("/frenet_planner/waypoints/W_X_" + path_id.data)
		W_Y = rospy.get_param("/frenet_planner/waypoints/W_Y_" + path_id.data)
		initial_orien = rospy.get_param("/frenet_planner/waypoints/initial_orien_" + path_id.data)
		print("W_X")
		print(W_X)
		print("W_Y")
		print(W_Y)
		print("Got params")
		self.frenet_path = Path()
		self.target_velocity = Twist()
		self.cost_fp = Float64()
		self.OBS = observation()#global path is stored in OBS
		global_path = Path()
		for i in range(1,len(W_X)+1):
			pose = Pose()
			pose.orientation.x = initial_orien[0]
			pose.orientation.y = initial_orien[1]
			pose.orientation.z = initial_orien[2]
			pose.orientation.w = initial_orien[3]
			pose.position.x = W_X[i-1]
			pose.position.y = W_Y[i-1]
			pose.position.z = 0.1
			global_path.poses.append(pose)
		print("Path : ")
		print(global_path)
		self.OBS.add_path(global_path)

		init_pub = rospy.Publisher('/carla/ego_vehicle/initialpose', PoseWithCovarianceStamped, queue_size = 100)
		#self.coll_hist = []
		#self.actor_list = []
		p = PoseWithCovarianceStamped()
		p.pose.pose.position.x = self.OBS.path.poses[0].position.x
		p.pose.pose.position.y = self.OBS.path.poses[0].position.y
		p.pose.pose.position.z = self.OBS.path.poses[0].position.z
		p.pose.pose.orientation.x = self.OBS.path.poses[0].orientation.x
		p.pose.pose.orientation.y = self.OBS.path.poses[0].orientation.y
		p.pose.pose.orientation.z = self.OBS.path.poses[0].orientation.z
		p.pose.pose.orientation.w = self.OBS.path.poses[0].orientation.w
		print("Initial position : ")
		print(p)
		time.sleep(1)
		init_pub.publish(p)
		# time.sleep(1)
	
	def callback(self, data):
		self.OBS.add_odom(data)

	def callback_lidar(self, data):
		self.OBS.add_lidar(data)

	def callback_fp(self, data):
		self.frenet_path = data

	def callback_cost_fp(self, data):
		self.cost_fp = data

	def listener(self):
		rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.callback)
		rospy.Subscriber('/scan_laser', LaserScan, self.callback_lidar)
		rospy.Subscriber('/frenet_path', Path, self.callback_fp)
		rospy.Subscriber('/cost_fp', Float64, self.callback_cost_fp)

	def step(self, action):
		# p = CarlaEgoVehicleControl()
		# p.header.stamp = rospy.Time.now()
		# p.throttle = action[0]
		# p.steer = action[1] 
		# p.brake = 0
		# print("Inside step.")
		p = Twist()#Random data type of ros to communicate action instead defining our own message.
		p.linear.x = action[0]
		p.linear.y = action[1]
		p.linear.z = action[2]
		p.angular.x = action[3]
		p.angular.y = 0
		p.angular.z = 0
		print("Action inside step : ")
		print(p)
		rospy.init_node('CarlaEnv')
		act_pub = rospy.Publisher('/action', Twist, queue_size = 100)
		while not act_pub.get_num_connections() > 1:
			print("Waiting for connection with act_pub. Run frenet_planner.")
		act_pub.publish(p)
		self.listener()
		self.frenet_path = rospy.wait_for_message("/frenet_path", Path)
		self.cost_fp = rospy.wait_for_message("/cost_fp", Float64)

		while math.isnan(self.frenet_path.poses[0].pose.position.x) :
			print("Waiting for proper values of frenet path") 
			act_pub.publish(p)
			self.listener()

		is_done = False
		if self.episode_start + SECONDS_PER_EPISODE < time.time():
			is_done = True
		odom_curr = self.OBS.ret_odom()
		path_curr = self.OBS.ret_path()
		lidar_curr = self.OBS.ret_lidar()
		observation = np.array((odom_curr.twist.twist.linear.x,odom_curr.twist.twist.linear.y,odom_curr.twist.twist.linear.z,odom_curr.twist.twist.angular.z))
		observation = np.hstack((observation, lidar_curr.ranges))
		for i in range(1,len(path_curr.poses)+1):
			observation = np.hstack((observation, path_curr.poses[i-1].position.x - odom_curr.pose.pose.position.x))
			observation = np.hstack((observation, path_curr.poses[i-1].position.y - odom_curr.pose.pose.position.y))
		lidar_reward = 0
		if not len(lidar_curr.ranges):
			lidar_reward = min(lidar_curr.ranges)
		print("LIDAR REWARD")
		print(lidar_reward)
		frenet_reward = self.cost_fp.data
		velocity_reward = (odom_curr.twist.twist.linear.x)**2
		reward = lidar_reward + frenet_reward + velocity_reward
		return observation, reward, is_done, None