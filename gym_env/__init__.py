from gym.envs.registration import register

register(
    id='airsim_env_v0',
    entry_point='gym_env.airsim_env:AirsimEnv'
)

