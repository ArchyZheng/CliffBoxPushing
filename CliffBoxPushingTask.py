from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from environment import create_box, create_target, create_wall
from gym import spaces
import numpy as np
import torch

narray = np.array


class CliffBoxPushingTask(BaseTask):
    """
    This is the most important class in our experiment.
    The task class in `omni.isaac.core` provides a way to modularize the scene creation, information retrieval,
    calculating metrics and creating more complex scenes with involved logic.
    """

    def __init__(self, name: str, offset=None) -> None:
        """
        Create some elements for RL.

        Such like: buffer, observation space, action space, etc.

        @param name: the name of task
        @param offset: the location related
        """
        super().__init__(name, offset)

        self._name = name
        self._offset = offset
        # location initialize
        self._jetbot_position = np.array([1.1, -2.0, .1]) + self._offset
        self._box_position = np.array([1.5, -4.5, .1]) + self._offset
        self._target_position = np.array([13.5, -4.5, .5]) + self._offset

        # set the action and observation space for RL
        # self.action_space = spaces.Discrete(4)
        self.action_space = spaces.Dict({'box_position': spaces.Box(low=0, high=7, shape=(2, 1)),
                                         'agent_position': spaces.Box(low=0, high=14, shape=(2, 1))})
        self.observation_space = spaces.Box(low=0, high=14, shape=(2, 2))

        # values used for defining RL buffers
        self._num_overvations = 4
        self._num_actions = 4
        self._device = 'cuda'
        self.num_envs = 1
        # a few class buffers to store RL_=-related states
        self.obs = torch.zeros((self.num_envs, self._num_overvations))
        self.resets = torch.zeros((self.num_envs, 1))

    def set_up_scene(self, scene: Scene) -> None:
        """
        Create environment.

        TODO: create a scene with walls, a box and a target.
        """
        print("in set_up_scene")

        # >>>>> agent >>>>>
        assets_root_path = get_assets_root_path()
        usd_path_jetbot = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        prim_path_jetbot = "/World/" + self._name + "/Jetbot"
        create_prim(prim_path=prim_path_jetbot, prim_type="Xform", position=self._jetbot_position)
        add_reference_to_stage(usd_path=usd_path_jetbot, prim_path=prim_path_jetbot)

        self._jetbot = ArticulationView(prim_path_jetbot, name="Jetbot")
        scene.add(self._jetbot)
        # <<<<< agent <<<<<

        # >>>>> box >>>>>
        self._box = create_box(name=self._name, offset=self._offset)
        # <<<<< box <<<<<

        # >>>>> target >>>>>
        self._target = create_target(name=self._name, offset=self._offset)
        # <<<<< target <<<<<

        # >>>>> walls >>>>>
        create_wall(name=self._name, offset=self._offset, scene=scene)
        # <<<<< walls <<<<<

        # >>>>> cliff >>>>>
        # create_cliff(name=self._name, offset=self._offset, scene=scene)
        # <<<<< cliff <<<<<

        scene.add_ground_plane()
        print("out set_up_scene")

    def post_reset(self) -> None:
        print("in post_reset")
        self._jetbot_left_wheel = self._jetbot.get_dof_index("left_wheel_joint")
        self._jetbot_right_wheel = self._jetbot.get_dof_index("right_wheel_joint")
        print(self._jetbot.num_dof)
        from omni.isaac.core.utils.types import ArticulationAction
        action = ArticulationAction(joint_positions=None, joint_efforts=None,
                                    joint_velocities=torch.tensor([5, 5], dtype=torch.float32))
        self._jetbot.apply_action(action)
        # indices = torch.arange(self._jetbot.count, device=self._device)
        print("out post_reset")

    def reset(self, env_index) -> None:
        """
        This method is used to set our environment into an initial state for starting a new training episode.

        We will implement the randomization in this method.

        But in this case, we don't need to randomize the environment.
        """
        print("in reset")
        if env_index is None:
            env_index = torch.arange(self.num_envs, device=self._device)  # actually, I don't know what is this
        num_resets = len(env_index)

        dof_vel = torch.zeros((num_resets, self._jetbot.num_dof), device=self._device)

        indices = env_index.to(dtype=torch.int32)
        self._jetbot.set_joint_velocity(dof_vel, indices=indices)

        self.resets[env_index] = 0
        print("out reset")

    def pre_physic_step(self, actions) -> None:
        print("in pre_physic_step")
        reset_env_index = self.resets.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_env_index) > 0:
            self.reset(reset_env_index)
        actions = torch.tensor(actions)
        forces = torch.zeros((self._jetbot.count, self._jetbot.num_dof), device=self._device)
        forces[:, self._jetbot_left_wheel] = actions[:, 0]
        forces[:, self._jetbot_right_wheel] = actions[:, 1]
        indices = torch.arange(self._jetbot.count, device=self._device)
        self._jetbot.set_joint_efforts(forces, indices=indices)
        print("out pre_physic_step")

    def calculate_metrics(self) -> dict:
        """
        This method is about calculating the reward.
        """
        distance_between_box_and_target = -1 * torch.norm(self._target.get_world_pose()[0]
                                                          - self._box.get_world_pose()[0])
        distance_between_box_and_agent = -1 * torch.norm(self._jetbot.get_world_poses()[0]
                                                         - self._box.get_world_pose()[0])
        distance_betweee_agent_and_target = -1 * torch.norm(self._target.get_world_pose()[0]
                                                            - self._jetbot.get_world_poses()[0])
        self._box.get_local_pose()
        return {'distance_between_agent_and_target': distance_betweee_agent_and_target,
                'distance_between_box_and_target': distance_between_box_and_target,
                'distance_between_box_and_agent': distance_between_box_and_agent}

    def get_observations(self) -> dict:
        observations = {'box_position': self._box.get_world_pose(), 'agent_position': self._jetbot.get_world_poses()}
        return observations

    def is_done(self) -> bool:
        return super().is_done()
