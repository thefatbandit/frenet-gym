#!/usr/bin/env python

# Copyright (c) 2019: Jianyu Chen (jianyuchen@berkeley.edu)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import division

import copy
import numpy as np
import pygame
import random
import time
from skimage.transform import resize
import sys
import os
import math

import gym
from gym import spaces
from gym.utils import seeding
import carla

from gym_carla.envs.render import BirdeyeRender
from gym_carla.envs.route_planner import RoutePlanner
from gym_carla.envs.misc import *

from gym_carla.envs.controller import VehiclePIDController

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../python_robotics/FrenetOptimalTrajectory/")


try:
    import frenet_optimal_trajectory
except ImportError:
    raise


def scale(X, x_min, x_max):
    nom = (X-X.min(axis=0))*(x_max-x_min)
    denom = X.max(axis=0) - X.min(axis=0)
    denom[denom == 0] = 1
    return x_min + nom/denom


class CarlaEnv(gym.Env):
    """An OpenAI gym wrapper for CARLA simulator."""

    def __init__(self, params):
        # parameters
        print('init mai hai ham')
        action_params = params['action_params']
        self.path_params = params['path_params']

        self.display_size = params['display_size']  # rendering screen size
        self.max_past_step = params['max_past_step']
        self.number_of_vehicles = params['number_of_vehicles']
        self.number_of_walkers = params['number_of_walkers']
        self.task_mode = params['task_mode']
        self.max_time_episode = params['max_time_episode']
        self.max_waypt = params['max_waypt']
        self.obs_range = params['obs_range']
        self.lidar_bin = params['lidar_bin']
        self.d_behind = params['d_behind']
        self.obs_size = int(self.obs_range/self.lidar_bin)
        self.out_lane_thres = params['out_lane_thres']
        self.desired_speed = params['desired_speed']
        self.max_ego_spawn_times = params['max_ego_spawn_times']
        self.display_route = params['display_route']
        control_params = params['control_params']
        self.args_lateral_dict = control_params['args_lateral_dict']
        self.args_longitudinal_dict = control_params['args_longitudinal_dict']

        D_T_S = action_params['d_t_s']
        N_S_SAMPLE = action_params['n_s_sample']
        MINT = action_params['mint']
        MAXT = action_params['maxt']
        MAX_ROAD_WIDTH = action_params['max_road_width']
        MAX_LAT_VEL = action_params['max_lat_vel']
        TARGET_SPEED = self.path_params['TARGET_SPEED']

        self.dt = self.path_params['DT']  # time interval between 2 frames

        if 'pixor' in params.keys():
            self.pixor = params['pixor']
            self.pixor_size = params['pixor_size']
        else:
            self.pixor = False

        # Destination
        if params['task_mode'] == 'roundabout':
            self.dests = [[4.46, -61.46, 0], [-49.53, -2.89, 0],
                          [-6.48, 55.47, 0], [35.96, 3.33, 0]]
        else:
            self.dests = None

        # action and observation spaces
        self.discrete = params['discrete']
        self.discrete_act = [params['discrete_acc'],
                             params['discrete_steer']]  # acc, steer
        self.n_acc = len(self.discrete_act[0])
        self.n_steer = len(self.discrete_act[1])
        if self.discrete:
            self.action_space = spaces.Discrete(self.n_acc*self.n_steer)
        else:
            self.action_space = spaces.Box(low=np.array([TARGET_SPEED - D_T_S*N_S_SAMPLE, MINT, -MAX_ROAD_WIDTH, -MAX_LAT_VEL]),
                                           high=np.array(
                [TARGET_SPEED + D_T_S*N_S_SAMPLE, MAXT, MAX_ROAD_WIDTH, MAX_LAT_VEL]),
                dtype=np.float32)

        observation_space_dict = {
            'camera': spaces.Box(low=0, high=255, shape=(self.obs_size, self.obs_size, 3), dtype=np.uint8),
            'lidar': spaces.Box(low=0, high=255, shape=(self.obs_size, self.obs_size, 3), dtype=np.uint8),
            'birdeye': spaces.Box(low=0, high=255, shape=(self.obs_size, self.obs_size, 3), dtype=np.uint8),
            'state': spaces.Box(np.array([-2, -1, -5, 0]), np.array([2, 1, 30, 1]), dtype=np.float32)
        }
        if self.pixor:  # dont change
            observation_space_dict.update({
                'roadmap': spaces.Box(low=0, high=255, shape=(self.obs_size, self.obs_size, 3), dtype=np.uint8),
                'vh_clas': spaces.Box(low=0, high=1, shape=(self.pixor_size, self.pixor_size, 1), dtype=np.float32),
                'vh_regr': spaces.Box(low=-5, high=5, shape=(self.pixor_size, self.pixor_size, 6), dtype=np.float32),
                'pixor_state': spaces.Box(np.array([-1000, -1000, -1, -1, -5]), np.array([1000, 1000, 1, 1, 20]), dtype=np.float32)
            })
        self.observation_space = spaces.Dict(observation_space_dict)
        self.observation_space = spaces.Box(
            low=-1, high=1, shape=(self.obs_size*self.obs_size*3+4,1), dtype=np.float32)
        # Connect to carla server and get world object
        print('connecting to Carla server...')
        client = carla.Client('localhost', params['port'])
        client.set_timeout(10.0)
        self.world = client.load_world(params['town'])
        print('Carla server connected!')

        # Set weather
        self.world.set_weather(carla.WeatherParameters.ClearNoon)

        # Get spawn points
        self.vehicle_spawn_points = list(
            self.world.get_map().get_spawn_points())
        self.walker_spawn_points = []
        for i in range(self.number_of_walkers):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                self.walker_spawn_points.append(spawn_point)

        # Create the ego vehicle blueprint
        self.ego_bp = self._create_vehicle_bluepprint(
            params['ego_vehicle_filter'], color='49,8,8')

        # Collision sensor
        self.collision_hist = []  # The collision history
        self.collision_hist_l = 1  # collision history length
        self.collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')

        # Lidar sensor
        self.lidar_data = None
        self.lidar_height = 2.1  # Note : HEIGHT OF LIDAR
        self.lidar_trans = carla.Transform(
            carla.Location(x=0.0, z=self.lidar_height))
        self.lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        self.lidar_bp.set_attribute('channels', '32')
        self.lidar_bp.set_attribute('range', '5000')

        # Camera sensor
        self.camera_img = np.zeros(
            (self.obs_size, self.obs_size, 3), dtype=np.uint8)
        self.camera_trans = carla.Transform(carla.Location(x=0.8, z=1.7))
        self.camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        self.camera_bp.set_attribute('image_size_x', str(self.obs_size))
        self.camera_bp.set_attribute('image_size_y', str(self.obs_size))
        self.camera_bp.set_attribute('fov', '110')
        # Set the time in seconds between sensor captures
        self.camera_bp.set_attribute('sensor_tick', '0.02')

        # Set fixed simulation step for synchronous mode
        self.settings = self.world.get_settings()
        self.settings.fixed_delta_seconds = self.dt

        # Record the time of total steps and resetting steps
        self.reset_step = 0
        self.total_step = 0

        # Initialize the renderer
        self._init_renderer()

        # Get pixel grid points
        if self.pixor:
            x, y = np.meshgrid(np.arange(self.pixor_size), np.arange(
                self.pixor_size))  # make a canvas with coordinates
            x, y = x.flatten(), y.flatten()
            self.pixel_grid = np.vstack((x, y)).T

    def reset(self):
        # Clear sensor objects
        print('reset')
        self.collision_sensor = None
        self.lidar_sensor = None
        self.camera_sensor = None

        # Delete sensors, vehicles and walkers
        self._clear_all_actors(['sensor.other.collision', 'sensor.lidar.ray_cast',
                                'sensor.camera.rgb', 'vehicle.*', 'controller.ai.walker', 'walker.*'])

        # Disable sync mode
        self._set_synchronous_mode(False)

        # Spawn surrounding vehicles
        random.shuffle(self.vehicle_spawn_points)
        count = self.number_of_vehicles
        if count > 0:
            for spawn_point in self.vehicle_spawn_points:
                if self._try_spawn_random_vehicle_at(spawn_point, number_of_wheels=[4]):
                    count -= 1
                if count <= 0:
                    break
        while count > 0:
            if self._try_spawn_random_vehicle_at(random.choice(self.vehicle_spawn_points), number_of_wheels=[4]):
                count -= 1

        # Spawn pedestrians
        random.shuffle(self.walker_spawn_points)
        count = self.number_of_walkers
        if count > 0:
            for spawn_point in self.walker_spawn_points:
                if self._try_spawn_random_walker_at(spawn_point):
                    count -= 1
                if count <= 0:
                    break
        while count > 0:
            if self._try_spawn_random_walker_at(random.choice(self.walker_spawn_points)):
                count -= 1

        # Get actors polygon list
        self.vehicle_polygons = []
        vehicle_poly_dict = self._get_actor_polygons('vehicle.*')
        self.vehicle_polygons.append(vehicle_poly_dict)
        self.walker_polygons = []
        walker_poly_dict = self._get_actor_polygons('walker.*')
        self.walker_polygons.append(walker_poly_dict)

        # Spawn the ego vehicle
        ego_spawn_times = 0
        while True:
            if ego_spawn_times > self.max_ego_spawn_times:
                self.reset()

            if self.task_mode == 'random':
                transform = random.choice(self.vehicle_spawn_points)
            if self.task_mode == 'roundabout':
                self.start = [
                    52.1+np.random.uniform(-5, 5), -4.2, 178.66]  # random
                # self.start=[52.1,-4.2, 178.66] # static
                transform = set_carla_transform(self.start)
            if self.task_mode == 'debug':
                self.start = [88.71, -237, 4]  # static
                transform = set_carla_transform(self.start)
            if self._try_spawn_ego_vehicle_at(transform):
                break
            else:
                ego_spawn_times += 1
                time.sleep(0.1)

        # Initializing Controller (Requires ego vehicle spawn)
        self._vehicle_controller = VehiclePIDController(self.ego,
                                                        args_lateral=self.args_lateral_dict,
                                                        args_longitudinal=self.args_longitudinal_dict)

        # Add collision sensor
        self.collision_sensor = self.world.spawn_actor(
            self.collision_bp, carla.Transform(), attach_to=self.ego)
        self.collision_sensor.listen(lambda event: get_collision_hist(event))

        def get_collision_hist(event):
            impulse = event.normal_impulse
            intensity = np.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
            self.collision_hist.append(intensity)
            if len(self.collision_hist) > self.collision_hist_l:
                self.collision_hist.pop(0)
        self.collision_hist = []

        # Add lidar sensor
        self.lidar_sensor = self.world.spawn_actor(
            self.lidar_bp, self.lidar_trans, attach_to=self.ego)
        self.lidar_sensor.listen(lambda data: get_lidar_data(data))

        def get_lidar_data(data):
            self.lidar_data = data

        # Add camera sensor
        self.camera_sensor = self.world.spawn_actor(
            self.camera_bp, self.camera_trans, attach_to=self.ego)
        self.camera_sensor.listen(lambda data: get_camera_img(data))

        def get_camera_img(data):
            array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (data.height, data.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.camera_img = array

        # Update timesteps
        self.time_step = 0
        self.reset_step += 1

        # Enable sync mode
        self.settings.synchronous_mode = True
        self.world.apply_settings(self.settings)

        # ===================================================================================
        self.routeplanner = RoutePlanner(self.ego, self.max_waypt)
        self.waypoints, _, self.vehicle_front = self.routeplanner.run_step()
        # ===================================================================================

        W_X_0 = [t[0] for t in self.waypoints]
        W_Y_0 = [t[1] for t in self.waypoints]
        self.tx, self.ty, self.tyaw, self.tc, self.csp = frenet_optimal_trajectory.generate_target_course(
            W_X_0, W_Y_0)

        self.birdeye_render.set_hero(self.ego, self.ego.id)

        return self._get_obs()

    def step(self, action):
        # Get target speed, target_waypoint
        print('step mai hai ham')
        ego_trans = self.ego.get_transform()
        ego_z = ego_trans.location.z
        # act = carla.VehicleControl(throttle=float(throttle), steer=float(-steer), brake=float(brake))
        Ti = action[1]
        di = action[2]
        di_d = action[3]
        tv = action[0]
        # Initial Conditions
        s0, c_speed, c_d, c_d_d, c_d_dd = self.initial_conditions()

        # Generate Frenet Path
        path = frenet_optimal_trajectory.frenet_optimal_planning(
            self.csp, s0, c_speed, c_d, c_d_d, c_d_dd, Ti, di, di_d, tv, self.path_params)

        self.target_waypoint = [path.x[1], path.y[1], ego_z]
        # sqrt(pow(1 - rk[1]*c_d, 2)*pow(c_speed, 2) + pow(c_d_d, 2))
        # To be given in Km/h
        self._target_speed = (
            (((1 - self.tc[1])**2)*((path.s_d[1])**2) + ((path.d_d[1])**2))**0.5) * 3.6
        # Apply control
        control = self._vehicle_controller.run_step(
            self._target_speed, self.target_waypoint)
        self.ego.apply_control(control)

        self.world.tick()

        # Append actors polygon list
        vehicle_poly_dict = self._get_actor_polygons('vehicle.*')
        self.vehicle_polygons.append(vehicle_poly_dict)
        while len(self.vehicle_polygons) > self.max_past_step:
            self.vehicle_polygons.pop(0)
        walker_poly_dict = self._get_actor_polygons('walker.*')
        self.walker_polygons.append(walker_poly_dict)
        while len(self.walker_polygons) > self.max_past_step:
            self.walker_polygons.pop(0)

        # route planner
        self.waypoints, _, self.vehicle_front = self.routeplanner.run_step()

        # state information
        info = {
            'waypoints': self.waypoints,
            'vehicle_front': self.vehicle_front
        }

        # Update timesteps
        self.time_step += 1
        self.total_step += 1

        return (self._get_obs(), self._get_reward(), self._terminal(), copy.deepcopy(info))

    def initial_conditions(self):
        print('initial_conditions mai hai ham')
        vel = self.ego.get_velocity()
        ego_trans = self.ego.get_transform()
        ego_x, ego_y = get_pos(self.ego)

        v = ((vel.x**2) + (vel.y**2))**0.5
        min_id, c_d = self.find_nearest_in_global_path()

        vec1 = [ego_x - self.tx[min_id], ego_y - self.ty[min_id]]
        vec2 = [self.tx[min_id] - self.tx[min_id + 1],
                self.tx[min_id] - self.ty[min_id + 1]]
        curl = vec1[0] * vec2[1] - vec1[1]*vec2[0]
        if (curl < 0):
            c_d = c_d * (-1)

        s0 = self.calc_s(min_id)
        ego_yaw = ego_trans.rotation.yaw/180*np.pi
        global_path_yaw = self.tyaw[min_id]
        delta_theta = ego_yaw - global_path_yaw
        c_d_d = v*math.sin(delta_theta)
        k_r = self.tc[min_id]
        c_speed = v*math.cos(delta_theta) / (1 - k_r*c_d)
        c_d_dd = 0

        return s0, c_speed, c_d, c_d_d, c_d_dd

    def find_nearest_in_global_path(self):
        print('find nearest in global path')
        ego_x, ego_y = get_pos(self.ego)
        tx = np.asarray(self.tx, dtype=np.float32)
        ty = np.asarray(self.ty, dtype=np.float32)
        temp = (tx - ego_x)**2 + (ty - ego_y)**2
        temp = np.sqrt(temp)
        min_id = np.argmin(temp)
        min_dist = np.min(temp)
        return min_id, min_dist

    def calc_s(self, min_id):
        print('calcs mai hai ham')

        tx = np.asarray(self.tx, dtype=np.float32)
        ty = np.asarray(self.ty, dtype=np.float32)
        to_stop = min_id + 1
        tx_cut = tx[:to_stop]
        ty_cut = ty[:to_stop]
        tx_cut_2 = np.hstack((tx_cut[0], tx_cut))
        ty_cut_2 = np.hstack((ty_cut[0], ty_cut))
        tx_cut_2 = tx_cut_2[:to_stop]
        ty_cut_2 = ty_cut_2[:to_stop]
        s = np.sum(np.sqrt((tx_cut - tx_cut_2)**2 + (ty_cut - ty_cut_2)**2))
        return s

    def seed(self, seed=None):
        print('seed')
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode):
        pass

    def _create_vehicle_bluepprint(self, actor_filter, color=None, number_of_wheels=[4]):
        print('create vehicale blyeprint')
        """Create the blueprint for a specific actor type.

		Args:
		  actor_filter: a string indicating the actor type, e.g, 'vehicle.lincoln*'.

		Returns:
		  bp: the blueprint object of carla.
		"""
        blueprints = self.world.get_blueprint_library().filter(actor_filter)
        blueprint_library = []
        for nw in number_of_wheels:
            blueprint_library = blueprint_library + \
                [x for x in blueprints if int(
                    x.get_attribute('number_of_wheels')) == nw]
        bp = random.choice(blueprint_library)
        if bp.has_attribute('color'):
            if not color:
                color = random.choice(
                    bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        return bp

    def _init_renderer(self):
        """Initialize the birdeye view renderer.
        """
        print('init renderer mai hai ham')
        pygame.init()
        self.display = pygame.display.set_mode(
            (self.display_size * 3, self.display_size),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        pixels_per_meter = self.display_size / self.obs_range
        pixels_ahead_vehicle = (
            self.obs_range/2 - self.d_behind) * pixels_per_meter
        birdeye_params = {
            'screen_size': [self.display_size, self.display_size],
            'pixels_per_meter': pixels_per_meter,
            'pixels_ahead_vehicle': pixels_ahead_vehicle
        }
        self.birdeye_render = BirdeyeRender(self.world, birdeye_params)

    def _set_synchronous_mode(self, synchronous=True):
        """Set whether to use the synchronous mode.
        """
        print('set synchrounous mode mai hai ham')
        self.settings.synchronous_mode = synchronous
        self.world.apply_settings(self.settings)

    def _try_spawn_random_vehicle_at(self, transform, number_of_wheels=[4]):
        """Try to spawn a surrounding vehicle at specific transform with random bluprint.

        Args:
          transform: the carla transform object.

        Returns:
          Bool indicating whether the spawn is successful.
        """
        print('try spawn rnadom vehicle mai hai ham')
        blueprint = self._create_vehicle_bluepprint(
            'vehicle.*', number_of_wheels=number_of_wheels)
        blueprint.set_attribute('role_name', 'autopilot')
        vehicle = self.world.try_spawn_actor(blueprint, transform)
        if vehicle is not None:
            vehicle.set_autopilot()
            return True
        return False

    def _try_spawn_random_walker_at(self, transform):
        """Try to spawn a walker at specific transform with random bluprint.

        Args:
          transform: the carla transform object.

        Returns:
          Bool indicating whether the spawn is successful.
        """
        print('try spawn random walker mai hai ham')
        walker_bp = random.choice(
            self.world.get_blueprint_library().filter('walker.*'))
        # set as not invencible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        walker_actor = self.world.try_spawn_actor(walker_bp, transform)

        if walker_actor is not None:
            walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
            walker_controller_actor = self.world.spawn_actor(
                walker_controller_bp, carla.Transform(), walker_actor)
            # start walker
            walker_controller_actor.start()
            # set walk to random point
            walker_controller_actor.go_to_location(
                self.world.get_random_location_from_navigation())
            # random max speed
            # max speed between 1 and 2 (default is 1.4 m/s)
            walker_controller_actor.set_max_speed(1 + random.random())
            return True
        return False

    def _try_spawn_ego_vehicle_at(self, transform):
        """Try to spawn the ego vehicle at specific transform.
        Args:
          transform: the carla transform object.
        Returns:
          Bool indicating whether the spawn is successful.
        """
        print('try spawn ego vehicle at mai hai ham')

        vehicle = None
        # Check if ego position overlaps with surrounding vehicles
        overlap = False
        for idx, poly in self.vehicle_polygons[-1].items():
            poly_center = np.mean(poly, axis=0)
            ego_center = np.array([transform.location.x, transform.location.y])
            dis = np.linalg.norm(poly_center - ego_center)
            if dis > 8:
                continue
            else:
                overlap = True
                break

        if not overlap:
            vehicle = self.world.try_spawn_actor(self.ego_bp, transform)

        if vehicle is not None:
            self.ego = vehicle
            return True

        return False

    def _get_actor_polygons(self, filt):
        """Get the bounding box polygon of actors.

        Args:
          filt: the filter indicating what type of actors we'll look at.

        Returns:
          actor_poly_dict: a dictionary containing the bounding boxes of specific actors.
        """
        print('get actor polygons mai hai ham')

        actor_poly_dict = {}
        for actor in self.world.get_actors().filter(filt):
            # Get x, y and yaw of the actor
            trans = actor.get_transform()
            x = trans.location.x
            y = trans.location.y
            yaw = trans.rotation.yaw/180*np.pi
            # Get length and width
            bb = actor.bounding_box
            l = bb.extent.x
            w = bb.extent.y
            # Get bounding box polygon in the actor's local coordinate
            poly_local = np.array(
                [[l, w], [l, -w], [-l, -w], [-l, w]]).transpose()
            # Get rotation matrix to transform to global coordinate
            R = np.array([[np.cos(yaw), -np.sin(yaw)],
                          [np.sin(yaw), np.cos(yaw)]])
            # Get global bounding box polygon
            poly = np.matmul(R, poly_local).transpose() + \
                np.repeat([[x, y]], 4, axis=0)
            actor_poly_dict[actor.id] = poly
        return actor_poly_dict

    def _get_obs(self):
        """Get the observations."""
        # Birdeye rendering
        print('get obs mai hai ham')

        self.birdeye_render.vehicle_polygons = self.vehicle_polygons
        self.birdeye_render.walker_polygons = self.walker_polygons
        self.birdeye_render.waypoints = self.waypoints

        # birdeye view with roadmap and actors
        birdeye_render_types = ['roadmap', 'actors']
        if self.display_route:
            birdeye_render_types.append('waypoints')
        self.birdeye_render.render(self.display, birdeye_render_types)
        birdeye = pygame.surfarray.array3d(self.display)
        birdeye = birdeye[0:self.display_size, :, :]
        birdeye = display_to_rgb(birdeye, self.obs_size)

        # Roadmap
        if self.pixor:
            roadmap_render_types = ['roadmap']
            if self.display_route:
                roadmap_render_types.append('waypoints')
            self.birdeye_render.render(self.display, roadmap_render_types)
            roadmap = pygame.surfarray.array3d(self.display)
            roadmap = roadmap[0:self.display_size, :, :]
            roadmap = display_to_rgb(roadmap, self.obs_size)
            # Add ego vehicle
            for i in range(self.obs_size):
                for j in range(self.obs_size):
                    if abs(birdeye[i, j, 0] - 255) < 20 and abs(birdeye[i, j, 1] - 0) < 20 and abs(birdeye[i, j, 0] - 255) < 20:
                        roadmap[i, j, :] = birdeye[i, j, :]

        # Display birdeye image
        birdeye_surface = rgb_to_display_surface(birdeye, self.display_size)
        self.display.blit(birdeye_surface, (0, 0))

        # Lidar image generation
        point_cloud = []
        # Get point cloud data
        for location in self.lidar_data:
            point_cloud.append([location.x, location.y, -location.z])
        point_cloud = np.array(point_cloud)
        # Separate the 3D space to bins for point cloud, x and y is set according to self.lidar_bin,
        # and z is set to be two bins.
        y_bins = np.arange(-(self.obs_range - self.d_behind),
                           self.d_behind+self.lidar_bin, self.lidar_bin)
        x_bins = np.arange(-self.obs_range/2, self.obs_range /
                           2+self.lidar_bin, self.lidar_bin)
        z_bins = [-self.lidar_height-1, -self.lidar_height+0.25, 1]
        # Get lidar image according to the bins
        lidar, _ = np.histogramdd(point_cloud, bins=(x_bins, y_bins, z_bins))
        # Only 2 bins in z-direction. Taking positive values.
        lidar[:, :, 0] = np.array(lidar[:, :, 0] > 0, dtype=np.uint8)
        lidar[:, :, 1] = np.array(lidar[:, :, 1] > 0, dtype=np.uint8)
        # Add the waypoints to lidar image
        if self.display_route:
            wayptimg = (birdeye[:, :, 0] <= 10) * \
                (birdeye[:, :, 1] <= 10) * (birdeye[:, :, 2] >= 240)
        else:
            wayptimg = birdeye[:, :, 0] < 0  # Equal to a zero matrix
        wayptimg = np.expand_dims(wayptimg, axis=2)
        wayptimg = np.fliplr(np.rot90(wayptimg, 3))

        # Get the final lidar image
        lidar = np.concatenate((lidar, wayptimg), axis=2)
        lidar = np.flip(lidar, axis=1)
        lidar = np.rot90(lidar, 1)
        lidar = lidar * 255

        # Display lidar image
        lidar_surface = rgb_to_display_surface(lidar, self.display_size)
        self.display.blit(lidar_surface, (self.display_size, 0))

        # Display camera image
        camera = resize(self.camera_img, (self.obs_size, self.obs_size)) * 255
        camera_surface = rgb_to_display_surface(camera, self.display_size)
        self.display.blit(camera_surface, (self.display_size * 2, 0))

        # Display on pygame
        pygame.display.flip()

        # State observation
        ego_trans = self.ego.get_transform()
        ego_x = ego_trans.location.x
        ego_y = ego_trans.location.y
        ego_yaw = ego_trans.rotation.yaw/180*np.pi  # in Radians
        lateral_dis, w = get_preview_lane_dis(self.waypoints, ego_x, ego_y)
        delta_yaw = np.arcsin(np.cross(w,
                                       np.array(np.array([np.cos(ego_yaw), np.sin(ego_yaw)]))))
        v = self.ego.get_velocity()
        ang_v = self.ego.get_angular_velocity()
        speed = np.sqrt(v.x**2 + v.y**2)
        # state = np.array([lateral_dis, - delta_yaw, speed,ang_v.z, self.vehicle_front])
        state = np.array([lateral_dis, - delta_yaw, speed, ang_v.z])
        # state.reshape((4, 1))

        if self.pixor:
            # Vehicle classification and regression maps (requires further normalization)
            vh_clas = np.zeros((self.pixor_size, self.pixor_size))
            vh_regr = np.zeros((self.pixor_size, self.pixor_size, 6))

            # Generate the PIXOR image. Note in CARLA it is using left-hand coordinate
            # Get the 6-dim geom parametrization in PIXOR, here we use pixel coordinate
            for actor in self.world.get_actors().filter('vehicle.*'):
                x, y, yaw, l, w = get_info(actor)
                x_local, y_local, yaw_local = get_local_pose(
                    (x, y, yaw), (ego_x, ego_y, ego_yaw))
                if actor.id != self.ego.id:
                    if abs(y_local) < self.obs_range/2+1 and x_local < self.obs_range-self.d_behind+1 and x_local > -self.d_behind-1:
                        x_pixel, y_pixel, yaw_pixel, l_pixel, w_pixel = get_pixel_info(
                            local_info=(x_local, y_local, yaw_local, l, w),
                            d_behind=self.d_behind, obs_range=self.obs_range, image_size=self.pixor_size)
                        cos_t = np.cos(yaw_pixel)
                        sin_t = np.sin(yaw_pixel)
                        logw = np.log(w_pixel)
                        logl = np.log(l_pixel)
                        pixels = get_pixels_inside_vehicle(
                            pixel_info=(x_pixel, y_pixel,
                                        yaw_pixel, l_pixel, w_pixel),
                            pixel_grid=self.pixel_grid)
                        for pixel in pixels:
                            vh_clas[pixel[0], pixel[1]] = 1
                            dx = x_pixel - pixel[0]
                            dy = y_pixel - pixel[1]
                            vh_regr[pixel[0], pixel[1], :] = np.array(
                                [cos_t, sin_t, dx, dy, logw, logl])

            # Flip the image matrix so that the origin is at the left-bottom
            vh_clas = np.flip(vh_clas, axis=0)
            vh_regr = np.flip(vh_regr, axis=0)

            # Pixor state, [x, y, cos(yaw), sin(yaw), speed]
            pixor_state = [ego_x, ego_y, np.cos(
                ego_yaw), np.sin(ego_yaw), speed]

        obs = {
            'camera': camera.astype(np.uint8),
            'lidar': lidar.astype(np.uint8),
            'birdeye': birdeye.astype(np.uint8),
            'state': state
        }

        if self.pixor:
            obs.update({
                'roadmap': roadmap.astype(np.uint8),
                'vh_clas': np.expand_dims(vh_clas, -1).astype(np.float32),
                'vh_regr': vh_regr.astype(np.float32),
                'pixor_state': pixor_state,
            })
        print(camera.shape)
        print(lidar.shape)
        print(birdeye.shape)
        print(state.shape)
        # float32 mai likhna hai
        temp = lidar.flatten()
        print(temp.shape)
        print("hello")
        temp = np.concatenate((temp, state))
        temp = temp.reshape((temp.shape[0], 1))
        temp = scale(temp,-1,1)
        print(temp.shape)
        return temp

    def _get_reward(self):
        """Calculate the step reward."""
        print('get reward mai hai ham')

        # reward for speed tracking
        v = self.ego.get_velocity()
        speed = np.sqrt(v.x**2 + v.y**2)
        r_speed = -abs(speed - self.desired_speed)

        # reward for collision
        r_collision = 0
        if len(self.collision_hist) > 0:
            r_collision = -1

        # reward for steering:
        r_steer = -self.ego.get_control().steer**2

        # reward for out of lane
        ego_x, ego_y = get_pos(self.ego)
        dis, w = get_lane_dis(self.waypoints, ego_x, ego_y)
        r_out = 0
        if abs(dis) > self.out_lane_thres:
            r_out = -1

        # longitudinal speed
        lspeed = np.array([v.x, v.y])
        lspeed_lon = np.dot(lspeed, w)

        # cost for too fast
        r_fast = 0
        if lspeed_lon > self.desired_speed:
            r_fast = -1

        # cost for lateral acceleration
        r_lat = - abs(self.ego.get_control().steer) * lspeed_lon**2

        r = 200*r_collision + 1*lspeed_lon + 10 * \
            r_fast + 1*r_out + r_steer*5 + 0.2*r_lat - 0.1

        return r

    def _terminal(self):
        """Calculate whether to terminate the current episode."""
        print('terminal mai hai ham')
        # Get ego state
        ego_x, ego_y = get_pos(self.ego)

        # If collides
        if len(self.collision_hist) > 0:
            return True

        # If reach maximum timestep
        if self.time_step > self.max_time_episode:
            return True

        # If at destination
        if self.dests is not None:  # If at destination
            for dest in self.dests:
                if np.sqrt((ego_x-dest[0])**2+(ego_y-dest[1])**2) < 4:
                    return True

        # If out of lane
        dis, _ = get_lane_dis(self.waypoints, ego_x, ego_y)
        if abs(dis) > self.out_lane_thres:
            return True

        return False

    def _clear_all_actors(self, actor_filters):
        """Clear specific actors."""
        print('clear all  mai hai ham')

        for actor_filter in actor_filters:
            for actor in self.world.get_actors().filter(actor_filter):
                if actor.is_alive:
                    if actor.type_id == 'controller.ai.walker':
                        actor.stop()
                    actor.destroy()
