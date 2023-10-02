import numpy as np
# from omni.isaac.core import World

def SimulationStarter():
    """
    This method will create framework.

    Generate the framework of Isaac Sim.

    1. SimulationApp:
        reference: standalone_examples/api/omni.isaac.kit/livestream.py

    2. Reinforcement learning related:
    """
    # >>>>> Simulation App >>>>>
    # this part is mainly referencing "standalone_examples/api/omni.isaac.kit/livestream.py"
    from omni.isaac.kit import SimulationApp
    CONFIG = {
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
        "headless": True,
        "renderer": "RayTracedLighting",
        "display_options": 3286,  # Set display options to show default grid
    }

    kit = SimulationApp(launch_config=CONFIG)

    from omni.isaac.core.utils.extensions import enable_extension

    # defule Livestream settings
    kit.set_setting("/app/window/drawMouse", True)
    kit.set_setting("/app/livestream/proto", "ws")
    kit.set_setting("/app/livestream/websocket/framerate_limit", 120)
    kit.set_setting("/ngx/enabled", False)

    enable_extension("omni.kit.livestream.native")

    # Enable WebSocket Livestream extension
    # Default URL: http://localhost:8211/streaming/client/
    # enable_extension("omni.services.streamclient.websocket")

    # Enable WebRTC Livestream extension
    # Default URL: http://localhost:8211/streaming/webrtc-client/
    # enable_extension("omni.services.streamclient.webrtc")

    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.articulations import ArticulationView
    name = 'good'
    assets_root_path = get_assets_root_path()
    asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
    prim_path = "/World/" + name + "/Fancy_Robot"

    # this function can create the agent from usd.
    add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

    location_original = np.array([0, -5, .5])
    location_agent = location_original + np.array([.5, -.5, 0])

    _robot = ArticulationView(prim_paths_expr=prim_path, name=name + "fancy_robot")
    from omni.isaac.core import World
    world = World()
    world.scene.add_default_ground_plane()

    world.scene.add(
        _robot
    )
    world.reset()
    while kit._app.is_running() and not kit.is_exiting():
        # _agent_left_joint = _robot.get_dof_index("chassis/left_wheel_joint")
        # _agent_right_joint = _robot.get_dof_index("chassis/right_wheel_joint")
        print(_robot.dof_names)
        kit.update()

    kit.close()
    # <<<<< Simulation App <<<<<


if __name__ == "__main__":
    """
    This code is created for test.

    I will create the RL system directly using standalong python.
    """
    SimulationStarter()

