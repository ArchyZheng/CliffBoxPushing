from omni.kit.scripting import BehaviorScript
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core import World
import numpy as np
from omni.isaac.gym.vec_env import VecEnvBase


class NewScript(BehaviorScript):
    def on_init(self):
        world = World()
        world.clear()
        world.scene.add_default_ground_plane()
        name = 'good'
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        prim_path = "/World/" + name + "/Fancy_Robot"

        # this function can create the agent from usd.
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

        location_original = np.array([0, -5, .5])
        location_agent = location_original + np.array([.5, .5, 0])

        self._robot = ArticulationView(prim_paths_expr=prim_path, name=name + "fancy_robot")
        world.scene.add(
            self._robot
        )
        world.reset()
        # get the dof from usd

    
    def on_play(self):
        self._agent_left_joint = self._agent.get_dof_index("chassis/left_wheel_joint")
        self._agent_right_joint = self._agent.get_dof_index("chassis/right_wheel_joint")
        pass
