import gymnasium as gym
import LSTM_Siekmann as lstm
# env = gym.make('Humanoid-v4', render_mode = 'human', xml_file='~/huron/huron/mujoco/huron2.xml')
env = gym.make('Humanoid-v4', render_mode = 'human')
# print(env.action_space)
observation, info = env.reset()
print(observation.size)
# print(type(observation))
# print(observation.shape)

# instantiate lstm
DynMod = 

# fixed joint upper and lower bounds
# config of each body, check humanoid gymnasium env.
# print(env.render_mode)
for _ in range(1000):
    env.render()
    # print('looping')
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)

    # RPPO

    if terminated or truncated:
        observation, info = env.reset()

env.close()