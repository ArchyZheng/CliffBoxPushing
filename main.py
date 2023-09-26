from omni.isaac.core.scenes.scene import Scene
from omni.kit.scripting import BehaviorScript
from .environment import Environment
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
import numpy as np

"""
This class is created for creating multitasks in Isaac Sim.

The main method is on_play.
"""
class Main(BehaviorScript):
    def on_init(self):
        self._world = World()
        self._world.clear()
        self._task = []
        self._number_of_tasks_colomn = 12
        self._number_of_tasks_row = 6


    
    def on_play(self):
        self._world.clear()
        self._world.scene.add_default_ground_plane()
        
        gap_colomn = 7
        gap_row = 15
        for colomn in range(self._number_of_tasks_colomn):
            for row in range(self._number_of_tasks_row):
                self._world.add_task(Environment(name="task" + "colomn" + str(colomn) + "row" + str(row), offset=np.array([-40 + gap_row * row, -40 + gap_colomn * colomn, 0])))
        return 
