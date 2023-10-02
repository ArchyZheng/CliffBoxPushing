from omni.isaac.gym.vec_env import VecEnvBase

env = VecEnvBase(headless=True)


from environment import CliffBoxPushing

task = CliffBoxPushing(name="CliffBoxPushing")
env.set_task(task, backend="torch")
env.reset()

import time

from omni.isaac.core.utils.extensions import enable_extension

enable_extension("omni.kit.livestream.native")
time.sleep(100000)
