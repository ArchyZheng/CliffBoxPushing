from omni.kit.scripting import BehaviorScript
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
import numpy as np

class Environment(BehaviorScript):
    def on_init(self):
        """
        Create a grid world:
        1. ground
        2. wall
        3. cliff

        this world will use meter as standard grid, 14m * 6m world.
        In this class all items is static!


        ATTENSION!

        THE UPPER LEFT CORNER is [0, 0] 
        """
        self._world = World()
        self._world.scene.clear()
        self._world.scene.add_default_ground_plane()

        #>>>>> create wall >>>>>#
        wall_original_location = np.array([
            [-7.5, 0, .5],
            [0, 3.5, .5],
            [7.5, 0, .5],
            [0, -3.5, .5]
        ])  #order: left, top, right, bottom
        wall_location = wall_original_location + np.array([7, -3, 0])

        wall_name = ['wall_left', 'wall_top', 'wall_right', 'wall_bottom']

        wall_scale = np.array([
            [1, 6, 1],
            [14, 1, 1],
            [1, 6, 1],
            [14, 1, 1]
        ]
        )

        for i in range(4):
            self._world.scene.add(
                FixedCuboid(
                    prim_path='/World/' + wall_name[i],
                    name=wall_name[i],
                    position=wall_location[i],
                    scale=wall_scale[i]
                )
            )
        #<<<<< Create Wall <<<<<#

        #>>>>> Create Cliff >>>>>#
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
        ])

        cliff_location = cliff_original_location + np.array([.5, -.5, 0])
        cliff_scale = np.array([1, 1, 1])
        cliff_color = np.array([1.0, 0, 0]) # RGB
        print(cliff_location[0])

        for index in range(len(cliff_location)):
            x, y,_ = cliff_location[index]
            prim_name = 'cliff'+str(index)
            self._world.scene.add(
                FixedCuboid(
                    prim_path='/World/' + prim_name,
                    name=prim_name,
                    position=cliff_location[index],
                    scale=cliff_scale,
                    color=cliff_color
                )
            )

        #<<<<< Create Cliff <<<<<#


        

        


    def on_destroy(self):
        print(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        print("hello world!!!!")
        print(f"{__class__.__name__}.on_play()->{self.prim_path}")

    def on_pause(self):
        print(f"{__class__.__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        print(f"{__class__.__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        pass
        # print(f"{__class__.__name__}.on_update(current_time={current_time}, delta_time={delta_time})->{self.prim_path}")