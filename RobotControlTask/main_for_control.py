from omni.isaac.gym.vec_env import VecEnvBase
import numpy as np

env = VecEnvBase(headless=True, enable_livestream=True)
from RobotControlTask import RobotControlTask
task = RobotControlTask(name="RobotControlTask_0", offset=np.array([0, 0, 0]))

env.set_task(task, backend="torch", sim_params={"use_gpu_pipeline": False})
print(env._world._device)
# env.reset()
while True:
    env._world.step()
    print(env._task.get_observations())
    print(env._task.calculate_metrics())
    print({'target_velocity': env._task._target_velocity, 'target_angle': env._task._target_angle})
    env.render()

env.close()
