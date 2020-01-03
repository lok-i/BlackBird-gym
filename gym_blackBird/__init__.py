from gym.envs.registration import register

register(
    id='blackBird-v0',
    entry_point='gym_blackBird.envs:blackBirdEnv',
)

register(
    id='blackBird-v1',
    entry_point='gym_blackBird.envs:blackBirdEnvStaticBalance',
)