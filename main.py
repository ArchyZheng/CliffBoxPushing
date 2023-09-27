from omni.isaac.core.scenes.scene import Scene
from omni.kit.scripting import BehaviorScript
from .environment import CliffBoxPushing
from omni.isaac.core import World
import numpy as np


class Main(BehaviorScript):
    """
    This class is created for creating multitasks in Isaac Sim.

    The main method is on_play.
    """
    def on_init(self):
        self._world = World()
        self._world.clear()
        self._task = []
        self._number_of_tasks_colomn = 2 # default 12
        self._number_of_tasks_row = 2 # default 6
    
    def on_play(self):
        self._world.clear()
        self._world.scene.add_default_ground_plane()
        
        gap_colomn = 7
        gap_row = 15
        
        for colomn in range(self._number_of_tasks_colomn):
            for row in range(self._number_of_tasks_row):
                self._world.add_task(CliffBoxPushing(name="task" + "colomn" + str(colomn) + "row" + str(row),\
                                                  offset=np.array([-40 + gap_row * row, -40 + gap_colomn * colomn, 0])))
        return 
