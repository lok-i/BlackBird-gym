import gym
import pybullet
from time import sleep

env = gym.make('gym_blackBird:blackBird-v0', renders = True, realWorld = 0)
observation = env.reset()

for i_episode in range(1000):
	print('NEW EPISODE')
	env.reset()
	r = 0
	for t in range(3000):
		sleep(0.001)
		action = env.action_space.sample()
		action = [1,1,1,1,-1,-1,1,1]
		action = [0,0,0,0,0,0,0,0]
		observation, reward, done, info = env.step(action, 80000)
		r+=reward
        # if done:
        #     print("Episode finished after {} timesteps".format(t+1))
        #     print('reward: '+str(r))
        #     break
env.close()

