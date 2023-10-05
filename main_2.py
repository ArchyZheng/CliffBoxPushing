from omni.isaac.gym.vec_env import VecEnvBase
import numpy as np
from CliffBoxPushingTask import CliffBoxPushingTask

env = VecEnvBase(headless=True, enable_livestream=True)
task = CliffBoxPushingTask(name="CliffBoxPushing", offset=np.array([0, 0, 0]))
env.set_task(task, backend="torch")
print(env.observation_space)
while True:
    env._world.step()
    observation = env._task.get_observations()
    reward = env._task.calculate_metrics()
    print(reward)
    print(observation)
    env.render()

env.close()
