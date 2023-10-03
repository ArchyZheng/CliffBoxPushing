from omni.isaac.gym.vec_env import VecEnvBase

env = VecEnvBase(headless=True, enable_livestream=True)


from CliffBoxPushingTask import CliffBoxPushingTask
import numpy as np

task = CliffBoxPushingTask(name="CliffBoxPushing", offset=np.array([0, 0, 0]))
env.set_task(task, backend="torch")

import time
# time.sleep(10000)
while True:
    env._world.step()
    env.step()
    env.render()

env.close()
