from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid, DynamicCuboid
import numpy as np
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.types import ArticulationAction


class CliffBoxPushing(BaseTask):
    """
    TODO: We may should reform this class as task, which will crete a environment and agent.

    This class is highly relative to reference shown below.
    ref: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_core_adding_manipulator.html#what-is-a-task
    """

    def __init__(self, name, offset=None):
        """
        @param name: the name of task
        @param offset: the location related
        """
        super().__init__(name=name, offset=offset)
        self._task_achieved = False
        self._name = name
        self.set_up_scene(World().scene)
        self.set_up_post_load()


    def set_up_scene(self, scene):
        """
        Create a grid world:
        1. wall
        2. cliff
        3. box
        4. target

        This world will use meter as standard grid, 14m * 6m world.
        In this class all items is static!

        TODO: initial agent

        ATTENSION!

        THE UPPER LEFT CORNER is [0, 0],
        AND THE Y DIRECTION IS MINER.
        @param scene: the world you create objects on it
        """
        super().set_up_scene(scene)
        self._scene = scene
        scene.add_default_ground_plane()
        self.create_wall()
        self.create_cliff()
        self._target = self.create_target()
        self._box = self.create_box()


        # self._agent = self.create_agent()
        self._agent = self.create_wheel_agent()
        self.set_up_post_load()

        # action = ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=5 * np.random.rand(2,))
        # self._agent.apply_action(action)
        # self._jetbot_articulation_controller.apply_action(action)
        

    def set_up_post_load(self):
        """
        Control the agent.

        This method just for test.

        reference: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_core_hello_robot.html#move-the-robot
        """
        self._jetbot_articulation_controller = self._agent.get_articulation_controller()
        # self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)

    def send_robot_actions(self):
        self._jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                    joint_efforts=None,
                                                                    joint_velocities=5 * np.random.rand(2,)))

    def create_wheel_agent(self):
        """
        Create a wheel robot.
        
        This method just for test.

        reference: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_core_hello_robot.html
        """
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        prim_path = "/World/" + self._name + "/Fancy_Robot"
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

        location_original = np.array([0, -5, .5])
        location_agent = location_original + np.array([.5, -.5, 0]) + self._offset
        scale = np.array([5, 5, 5])
        jetbot_robot = self._scene.add(Robot(prim_path=prim_path, name=self._name + "fancy_robot", position=location_agent, scale=scale))
        return jetbot_robot


    def create_agent(self):
        """
        The initial location of agent is at [0, -5, .5].

        @return agent DynamicCuboid
        """
        location_original = np.array([0, -5, .5])
        location_agent = location_original + np.array([.5, -.5, 0]) + self._offset
        name = self._name + 'agent'
        agent = DynamicCuboid(
                prim_path='/World/' + self._name + "/" + 'agent',
                name=name,
                position=location_agent,
                scale=np.array([1, 1, 1]),
                color=np.array([0, 0, 0]),
                mass=True,
        )
        self._scene.add(
            agent
        )
        return agent


    def create_box(self):
        """
        The initial location of box is fixed at [1, -4, 0.5].

        @return box DynamicCuboid
        """
        location_original = np.array([1, -4, .5])
        location_box = location_original + np.array([.5, -.5, 0]) + self._offset
        name = self._name + 'box'
        box = DynamicCuboid(
                prim_path='/World/' + self._name + "/" + 'box',
                name=name,
                position=location_box,
                scale=np.array([1, 1, 1]),
                color=np.array([0.5, 0.5, 0]),
                mass=True,
            )
        self._scene.add(
            box
        )
        return box

    def create_target(self):
        """
        The position of target is also fixed at [13, -4, .5]. The target can be pass through.
        
        @return target DynamicCuboid
        """
        location_original = np.array([13, -4, .5])
        location_box = location_original + np.array([.5, -.5, 0]) + self._offset
        name = self._name + 'target'
        dummy_target = DynamicCuboid(
                prim_path='/World/' + self._name + "/" + 'target',
                name=name,
                position=location_box,
                scale=np.array([1, 1, 1]),
                color=np.array([0, 1, 0]),
                mass=True,
            )
        dummy_target.disable_rigid_body_physics()
        dummy_target.set_collision_enabled(False)
        self._scene.add(
            dummy_target
        )
        return dummy_target


    def create_wall(self):
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
        wall_location = wall_original_location + np.array([7, -3, 0]) + self._offset # adjust the postion
        wall_name = ['wall_left', 'wall_top', 'wall_right', 'wall_bottom']
        wall_scale = np.array([
            [1, 6, 1],
            [14, 1, 1],
            [1, 6, 1],
            [14, 1, 1]
        ]
        )

        for i in range(4):
            name = self._name + wall_name[i]
            self._scene.add(
                FixedCuboid(
                    prim_path='/World/' + self._name + "/" +wall_name[i],
                    name=name,
                    position=wall_location[i],
                    scale=wall_scale[i],
                )
            )
        
    
    def create_cliff(self):
        """
        According to the assignment report, we will build some cliff which don't have rigid body and collision effect.

        Therefor the agent and box can pass through the cliff.

        Maybe we should create new class for cliff for reward function, 
        if agent and box fall into cliff, this exprience should be restart and get -1000 reward.
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
        ]) # according to assignment

        cliff_location = cliff_original_location + np.array([.5, -.5, 0]) + self._offset # adjust the postion
        cliff_scale = np.array([1, 1, 1])
        cliff_color = np.array([1.0, 0, 0]) # RGB

        for index in range(len(cliff_location)):
            x, y,_ = cliff_location[index]
            prim_name = 'cliff'+ str(index)
            name = self._name + 'cliff'+ str(index)
            cliff_dummy_cuboid = DynamicCuboid(
                prim_path='/World/' + self._name + "/" + prim_name,
                name=name,
                position=cliff_location[index],
                scale=cliff_scale,
                color=cliff_color,
                mass=True,
            )
            # the cuboid can be pass through.
            cliff_dummy_cuboid.disable_rigid_body_physics()
            cliff_dummy_cuboid.set_collision_enabled(False)

            self._scene.add(
                cliff_dummy_cuboid
            )
