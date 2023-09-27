from omni.isaac.core.controllers import BaseController
import numpy as np


class CoolController(BaseController):
    """
    This class is a trial.

    reference: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_core_adding_controller.html#isaac-sim-app-tutorial-core-adding-controller
    """
    
    def __init__(self):
        """
        This method should assign two crucial properties of the robot.

        Which are: wheel radius and wheel base.

        Those two properties are related with velocity.

        
        """
        super().__init__(name="my_cool_controller")
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return 
    
    def forward(self, command):
        pass