from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
from gym import spaces
import torch


class RobotControlTask(BaseTask):
    """
    This task is used to control the robot.

    Given target velocity and angle, the robot need adjust the wheel velocity to make the robot reach the target.

    action space:
        1. the acceleration of each wheels. speed range: [-10, 10]

    state space:
        1. current speed of robot
        2. current angle
        3. current position

    rewards:
        1. the distance between target angle and current angle
        2. the distance between target speed and current speed
        3. the distance between current position and original position
    reward wants the agent can change the angle and speed to reach the goal, but do not left original position too much.
    """
    def __init__(self, name: str, offset=None, device='cpu'):
        """
        Randomly set the wheel speed of each wheel. Veliocity range: [-10, 10]

        @param name: the name of task
        @param offset: the location related
        @param device:
        """
        super().__init__(name, offset)
        self._device = device
        self.num_envs = 10
        self._initial_wheel_speed = torch.tensor(np.random.uniform(-10, 10, size=(2, )),
                                                 dtype=torch.float32, device=self.device)
        self.observation_space = spaces.Dict(
            {
                'curent_angle': spaces.Box(low=0, high=360, shape=(1, 1)),
                'curent_speed': spaces.Box(low=-10, high=10, shape=(2, 1)),
                'current_position': spaces.Box(low=-100, high=100, shape=(2, 1)),
            }
        )

        self.action_space = spaces.Box(low=-10, high=10, shape=(2, 1))  # the velocity of each wheel

    def set_up_scene(self, scene: Scene) -> None:
        """
        Initlize the scene.
        """
        assets_root_path = get_assets_root_path()
        usd_path_jetbot = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        prim_path_jetbot = "/World/" + self._name + "/Jetbot"
        add_reference_to_stage(usd_path=usd_path_jetbot, prim_path=prim_path_jetbot)
        self._jetbot = ArticulationView(prim_path_jetbot, name="Jetbot")
        scene.add(self._jetbot)
        scene.add_default_ground_plane()

    def reset(self, env_index=None) -> None:
        """
        set robot speed as zero.
        """
        if env_index is None:
            env_index = torch.arange(self.num_envs, device=self._device)  # actually, I don't know what is this
        num_resets = len(env_index)

        dof_vel = torch.zeros((num_resets, self._jetbot.num_dof), device=self._device)

        indices = env_index.to(dtype=torch.int32)
        print(dof_vel)
        self._jetbot.set_joint_velocities(dof_vel, indices=indices)

    def post_reset(self) -> None:
        """
        after reset, we will give speed to each wheel.

        reasign the goal velocity and angle.
        """
        action = ArticulationAction(joint_positions=None, joint_efforts=None,
                                    joint_velocities=self._initial_wheel_speed)
        self._target_velocity = torch.tensor(np.random.uniform(-10, 10, size=(2, )),
                                             dtype=torch.float32, device=self.device)
        self._target_angle = torch.tensor(np.random.uniform(-1, 1, size=(1, )),
                                          dtype=torch.float32, device=self.device)
        self._jetbot.apply_action(action)

    def get_observations(self) -> dict:
        """
        angle /in [-1, 1]

        @return: the current angle, speed and position of robot.
        """
        current_speed = self._jetbot.get_linear_velocities()[0][0:2]
        current_angle = self._jetbot.get_local_poses()[1][0][0]
        current_position = self._jetbot.get_local_poses()[0][0][0:2]

        self.observations = {'curent_angle': current_angle, 'curent_speed': current_speed,
                             'current_position': current_position}
        return self.observations

    def calculate_metrics(self) -> dict:
        """
        distance_angle = target_angle - current_angle
        distance_speed = target_speed - current_speed
        distance_position = current_position - original_position
        """
        distance_angle = self._target_angle - self.observations['curent_angle']
        distance_speed = self._target_velocity - self.observations['curent_speed']
        distacne_position = self.observations['current_position'] - torch.tensor([0, 0], dtype=torch.float32, device=self.device)

        self.metrics = {'distance_angle': distance_angle, 'distance_speed': distance_speed,
                        'distacne_position': distacne_position}
        return self.metrics
