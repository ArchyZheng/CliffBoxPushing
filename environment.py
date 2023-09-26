from omni.kit.scripting import BehaviorScript
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid, DynamicCuboid
import numpy as np
from omni.isaac.core.tasks import BaseTask

class Environment(BaseTask):

    def __init__(self, name, offset=None):
        """
        Create a grid world:
        1. ground
        2. wall
        3. cliff
        4. TODO: box
        5. TODO: target

        this world will use meter as standard grid, 14m * 6m world.
        In this class all items is static!

        TODO: initial robot

        ATTENSION!

        THE UPPER LEFT CORNER is [0, 0],
        AND THE Y DIRECTION IS MINER.
        @param name: the name of task
        @param offset: the location related
        """
        super().__init__(name=name, offset=offset)
        self._name = name
        self._world = World()

        # self._world.scene.add_default_ground_plane()

        self.create_wall()
        self.create_cliff()

    def create_wall(self):
        """
        According to the assignment report. Build 14 * 6 grid wall.

        We use the upper corner as original point of this map, therefore we in the y direction it will be minor.
        """
        wall_original_location = np.array([
            [-7.5, 0, .5],
            [0, 3.5, .5],
            [7.5, 0, .5],
            [0, -3.5, .5]
        ])  #order: left, top, right, bottom
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
            self._world.scene.add(
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

        cliff_location = cliff_original_location + np.array([.5, -.5, 0]) + self._offset# adjust the postion
        cliff_scale = np.array([1, 1, 1])
        cliff_color = np.array([1.0, 0, 0]) # RGB

        for index in range(len(cliff_location)):
            x, y,_ = cliff_location[index]
            prim_name = 'cliff'+ str(index)
            name = self._name + 'cliff'+ str(index)
            print(name)
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

            self._world.scene.add(
                cliff_dummy_cuboid
            )
