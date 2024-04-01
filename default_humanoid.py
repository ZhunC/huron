import gymnasium as gym
env = gym.make('Walker2d-v4', ctrl_cost_weight=0.1, ....)
observation, info = env.reset()
print(observation)
for _ in range(1000):
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()