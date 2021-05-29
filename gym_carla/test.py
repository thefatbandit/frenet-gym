#!/usr/bin/env python

# Copyright (c) 2019: Jianyu Chen (jianyuchen@berkeley.edu).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import gym
import gym_carla
import carla
# from stable_baselines.common.policies import FeedForwardPolicy
# from stable_baselines import SAC
from stable_baselines.common.env_checker import check_env

# class CustomPolicy(FeedForwardPolicy):
#         def __init__(self, *args, **kwargs):
#             super(CustomPolicy, self).__init__(*args, **kwargs,
#                                             net_arch=[64, 64,  dict(pi=[64],
#                                                                   vf=[64])],
#                                             feature_extraction="mlp")


def main():
    # parameters for the gym_carla environment
    params = {
        'number_of_vehicles': 10,
        'number_of_walkers': 0,
        'display_size': 256,  # screen size of bird-eye render
        'max_past_step': 1,  # the number of past steps to draw
        'dt': 0.1,  # time interval between two frames
        'discrete': False,  # whether to use discrete control space
        # discrete value of accelerations
        'discrete_acc': [-3.0, 0.0, 3.0],
        # discrete value of steering angles
        'discrete_steer': [-0.2, 0.0, 0.2],
        # continuous acceleration range
        'continuous_accel_range': [-3.0, 3.0],
        # continuous steering angle range
        'continuous_steer_range': [-0.3, 0.3],
        'ego_vehicle_filter': 'vehicle.lincoln*',  # filter for defining ego vehicle
        'port': 2000,  # connection port
        'town': 'Town02',  # which town to simulate
        # mode of the task, [random, roundabout (only for Town03), debug (for fixed starting point)]
        'task_mode': 'random',
        'max_time_episode': 1000,  # maximum timesteps per episode
        'max_waypt': 12,  # maximum number of waypoints
        'obs_range': 32,  # observation range (meter)
        'lidar_bin': 0.125,  # bin size of lidar sensor (meter)
        'd_behind': 12,  # distance behind the ego vehicle (meter)
        'out_lane_thres': 2.0,  # threshold for out of lane
        'desired_speed': 8,  # desired speed (m/s)
        'max_ego_spawn_times': 200,  # maximum times to spawn ego vehicle
        'display_route': True,  # whether to render the desired route
        'pixor_size': 64,  # size of the pixor labels
        'pixor': False,  # whether to output PIXOR observation,
        'action_params': {
                    # 'max_speed': 13.89,
                    # 'max_accel': 2.0,
                    # 'max_curvature': 1.0,
                    'max_road_width': 10.0,
                    # 'd_road_w': 1.0,
                    'maxt': 5.0,
                    'mint': 2.0,
                    'd_t_s': 1.389,
                    'n_s_sample': 1.5,
                    # 'min_lat_vel': -1.0,
                    'max_lat_vel': 1.0,
                    # 'd_d_ns': 0.5,
                    # 'max_shift_d': 3
        },

        'path_params': {
            'K_J': 0.1,
            'K_T': 0.1,
            'K_D': 1.0,
            'K_LAT': 1.0,
            'K_LON': 1.0,
            'DT': 0.2,
            'TARGET_SPEED': 30.0 / 3.6
        },

        # Taken from carla's python examples themselves.
        'control_params': {
            'args_lateral_dict': {
                'K_P': 1.95,
                'K_D': 0.01,
                'K_I': 1.4,
                'dt': 0.2},  # SAME AS LINE 62
            'args_longitudinal_dict': {
                'K_P': 1.0,
                'K_D': 0,
                'K_I': 1,
                'dt': 0.2}
        }
    }

    # Set gym-carla environment
    env = gym.make('carla-v0', params=params)

    # Env-Check for Stable Baselines
    check_env(env)
    print("ENV CHECK DONE")
    # # # Defining Model
    # model = SAC(CustomPolicy, env, verbose=1)

    # print("1st Reset Start")
    # obs = env.reset()
    # print("1st Reset End")

    # total_steps = 1000
    # save_interval = 50
    # iteration_steps = int(save_interval)
    # number_iterations = math.ceil(total_steps/save_interval)

    # for i in range (1000):
    #   if i == num_iterations - 1:
    #     iteration_steps = int(total_steps - i*save_interval)

    #   # obs,r,done,info = env.step(env.action_space.sample())
    #   model.learn(total_timesteps=iteration_steps)

    #   # if done:
    #   #   obs = env.reset()

    #   now = datetime.now()
    #   date_time = now.strftime("%m_%d_%Y_%H_%M_%S")
    #   print(date_time)
    #   model.save("ppo2_frenet" + "%s"%(date_time))


if __name__ == '__main__':
    main()
