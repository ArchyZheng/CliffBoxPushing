from omni.isaac.core.scenes.scene import Scene
from omni.kit.scripting import BehaviorScript
from .environment import Environment
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
import numpy as np


class Main(BehaviorScript):
    def on_init(self):
        print("hello world!")
        self._world = World()
        self._world.clear()
        self._task = []
        self._number_of_tasks = 12


    
    def on_play(self):
        self._world.clear()
        self._world.scene.add_default_ground_plane()
        for i in range(self._number_of_tasks):
            self._world.add_task(Environment(name="task" + str(i), offset=np.array([0, -40 + 7 * i, 0])))
        return 
