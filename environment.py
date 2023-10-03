from omni.isaac.core.objects import FixedCuboid, DynamicCuboid
import numpy as np
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
import torch

def create_wheel_agent(self):
    """
    Create a wheel robot.

    1. add agent into scene
    2. state index
    
    reference: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_gym_new_rl_example.html
    """
    assets_root_path = get_assets_root_path()
    asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
    prim_path = "/World/" + self.name + "/Fancy_Robot"

    # this function can create the agent from usd.
    add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

    location_original = np.array([0, -5, .5])
    location_agent = location_original + np.array([.5, -.5, 0]) + self._offset
    robot = ArticulationView(prim_paths_expr=prim_path, name=self._name + "fancy_robot")
    # get the dof from usd
    return robot

def create_agent(name, offset):
    """
    The initial location of agent is at [0, -5, .5].

    @return agent DynamicCuboid
    """
    location_original = np.array([0, -5, .5])
    location_agent = location_original + np.array([.5, -.5, 0]) + offset
    name = name + 'agent'
    agent = DynamicCuboid(
        prim_path='/World/' + name + "/" + 'agent',
        name=name,
        position=location_agent,
        scale=np.array([1, 1, 1]),
        color=np.array([0, 0, 0]),
        mass=True,
    )
    return agent

def create_box(name, offset):
    """
    The initial location of box is fixed at [1, -4, 0.5].

    @return box DynamicCuboid
    """
    location_original = np.array([1, -4, .5])
    location_box = location_original + np.array([.5, -.5, 0]) + offset
    name = name + 'box'
    box = DynamicCuboid(
        prim_path='/World/' + name + "/" + 'box',
        name=name,
        position=location_box,
        scale=np.array([.2, .2, .2]),
        color=np.array([0.5, 0.5, 0]),
        mass=True,
    )
    return box

def create_target(name, offset):
    """
    The position of target is also fixed at [13, -4, .5]. The target can be pass through.
    
    @return target DynamicCuboid
    """
    location_original = np.array([13, -4, .5])
    location_box = location_original + np.array([.5, -.5, 0]) + offset
    name = name + 'target'
    dummy_target = DynamicCuboid(
        prim_path='/World/' + name + "/" + 'target',
        name=name,
        position=location_box,
        scale=np.array([1, 1, 1]),
        color=np.array([0, 1, 0]),
        mass=True,
    )
    dummy_target.disable_rigid_body_physics()
    dummy_target.set_collision_enabled(False)
    return dummy_target

def create_wall(name, offset, scene):
    """
    According to the assignment report. Build 14 * 6 grid wall.

    We use the upper corner as original point of this map, therefore we in the y direction it will be minor.

    The order is `left`, `top`, `right`, `bottom`
    """
    wall_original_location = np.array([
        [-7.5, 0, .5],
        [0, 3.5, .5],
        [7.5, 0, .5],
        [0, -3.5, .5]
    ])
    wall_location = wall_original_location + np.array([7, -3, 0]) + offset  # adjust the position
    wall_name = ['wall_left', 'wall_top', 'wall_right', 'wall_bottom']
    wall_scale = np.array([
        [1, 6, 1],
        [14, 1, 1],
        [1, 6, 1],
        [14, 1, 1]
    ]
    )

    for i in range(4):
        name = name + wall_name[i]
        dummy_wall = FixedCuboid(
            prim_path='/World/' + name + "/" + wall_name[i],
            name=name,
            position=wall_location[i],
            scale=wall_scale[i],
        )
        scene.add(dummy_wall)

def create_cliff(name, offset, scene):
    """
    According to the assignment report, we will build some cliff which don't have rigid body and collision effect.

    Therefor the agent and box can pass through the cliff.

    Maybe we should create new class for cliff for reward function, 
    if agent and box fall into cliff, this experience should be restarted and get -1000 reward.
    """
    cliff_original_location = np.array([
        [6, -0, 0.5],
        [6, -1, 0.5],
        [6, -2, 0.5],
        [6, -3, 0.5],
        [7, -0, 0.5],
        [7, -1, 0.5],
        [7, -2, 0.5],
        [3, -2, 0.5],
        [3, -3, 0.5],
        [3, -4, 0.5],
        [3, -5, 0.5],
        [11, -3, 0.5],
        [11, -4, 0.5],
        [11, -5, 0.5],
        [12, -2, 0.5],
        [12, -3, 0.5],
        [12, -4, 0.5],
        [12, -5, 0.5],
    ])  # according to assignment

    cliff_location = cliff_original_location + np.array([.5, -.5, 0]) + offset  # adjust the postion
    cliff_scale = np.array([1, 1, 1])
    cliff_color = np.array([1.0, 0, 0])  # RGB

    for index in range(len(cliff_location)):
        x, y, _ = cliff_location[index]
        prim_name = 'cliff' + str(index)
        name = name + 'cliff' + str(index)
        cliff_dummy_cuboid = DynamicCuboid(
            prim_path='/World/' + name + "/" + prim_name,
            name=name,
            position=cliff_location[index],
            scale=cliff_scale,
            color=cliff_color,
            mass=True,
        )
        # the cuboid can be pass through.
        cliff_dummy_cuboid.disable_rigid_body_physics()
        cliff_dummy_cuboid.set_collision_enabled(False)

        scene.add(
            cliff_dummy_cuboid
        )
