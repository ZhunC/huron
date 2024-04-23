import gymnasium as gym
env = gym.make('Walker2d-v4', render_mode = 'human')
observation, info = env.reset()
print(observation)
for _ in range(1000):
    env.render()
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()