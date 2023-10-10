from omni.isaac.gym.vec_env import VecEnvBase
import numpy as np
from model import MLPNet

env = VecEnvBase(headless=True, enable_livestream=True)
from RobotControlTask import RobotControlTask
task = RobotControlTask(name="RobotControlTask_0", offset=np.array([0, 0, 0]))

env.set_task(task, backend="torch", sim_params={"use_gpu_pipeline": False})
MLP_net = MLPNet(5, 2).to(device='cpu')
# env.reset()
actions = env._task._initial_wheel_speed

while True:
    observations, rewards, _, _ = env.step(actions=actions)
    actions = MLP_net(observations)
    print(actions)
    env.render()
