import gym
import envs
import rospy

print("Before gym ")
env = gym.make('CarlaEnv-v0')

while 1:
	print("Before Calling reset")
	obs = env.reset()
	print("After Calling reset")
	# print(obs)
	raw_input("Press Enter to continue...")
	for i in range(50):
		print(i)
		obs, rew, done, _ = env.step(env.action_space.sample())
		print("OBSERVATION : ")
		print(obs)
		print("REWARD : ")
		print(rew)
		# rospy.sleep(1)	
		#print(obs)
	pass