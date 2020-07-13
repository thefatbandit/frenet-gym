from gym.envs.registration import register
#print("init inside envs")
register(id='CarlaEnv-v0', entry_point='envs.carla_env_dir:CarlaEnv')
#print("init inside envs2")