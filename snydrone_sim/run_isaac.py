#!/usr/bin/env python3
"""
Snydrone Isaac Sim Cinema Environment
Spawns a Pegasus drone + PX4/ROS2 backends and a person to film.
"""

import carb
from isaacsim import SimulationApp

# Start Simulation Ext
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World
from isaacsim.core.utils.extensions import enable_extension

# Enable ROS2 Bridge
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.usd
omni.usd.get_context().new_stage()

import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

def create_ros2_target_publisher():
    """Create an OmniGraph to publish the Target Car pose over ROS 2 securely (zero rclpy dependency)"""
    import omni.graph.core as og
    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": "/ROS2PublishPoseGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishPose", "omni.isaac.ros2_bridge.ROS2PublishPose"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishPose.inputs:execIn"),
                ],
                keys.SET_VALUES: [
                    ("PublishPose.inputs:targetPrims", ["/World/target_car"]),
                    ("PublishPose.inputs:topicName", "/snydrone/target/pose"),
                ]
            }
        )
        carb.log_warn("Created native ROS2 Publish Pose ActionGraph for Target Car.")
    except Exception as e:
        carb.log_error(f"Failed to create OmniGraph: {e}")

def create_ros2_camera_publisher(camera_prim_path: str, topic_name: str = "/drone1/camera/color/image_raw"):
    """Create an OmniGraph to publish Camera images over ROS 2 securely"""
    import omni.graph.core as og
    import omni.replicator.core as rep

    try:
        # Create render product externally
        render_product = rep.create.render_product(camera_prim_path, [1280, 720])
        
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/ROS2CameraGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishImage.inputs:execIn"),
                ],
                keys.SET_VALUES: [
                    ("PublishImage.inputs:renderProductPath", render_product.path),
                    ("PublishImage.inputs:topicName", topic_name),
                    ("PublishImage.inputs:type", "rgb"),
                ]
            }
        )
        carb.log_warn(f"Created native ROS2 Publish Image Writer for {camera_prim_path} on {topic_name}.")
    except Exception as e:
        carb.log_error(f"Failed to create Camera OmniGraph: {e}")

class CircleVehicleController:
    """A simple controller to make an Isaac Sim Prim drive in a circle"""
    def __init__(self, prim_path, radius=8.0, speed=0.15):
        self.prim_path = prim_path
        self._radius = radius
        self.gamma = 0.0
        self.gamma_dot = speed
        
    def update(self, dt: float):
        self.gamma += self.gamma_dot * dt
        x = self._radius * np.cos(self.gamma)
        y = self._radius * np.sin(self.gamma)
        # Update prim position directly
        import omni.usd
        from pxr import UsdGeom, Gf
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.prim_path)
        if prim:
            xform = UsdGeom.Xformable(prim)
            xform_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    xform_op = op
                    break
            if not xform_op:
                xform_op = xform.AddTranslateOp()
            xform_op.Set(Gf.Vec3d(x, y, 0.4))
            
            # Optionally update rotation to face direction of travel
            rot_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rot_op = op
                    break
            if not rot_op:
                rot_op = xform.AddRotateXYZOp()
            rot_op.Set(Gf.Vec3d(0, 0, np.degrees(self.gamma + np.pi/2)))
        
        return [x, y, 0.4], [0,0,0,1]


class SnydroneApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load Urban environment
        import isaacsim.storage.native as nucleus
        assets_root_path = nucleus.get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
        else:
            city_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd" 
            print(f"Loading environment from: {city_path}")
            self.pg.load_asset(city_path, "/World/layout")

        # Spawn Target (Carton Box)
        from omni.isaac.core.objects import DynamicCuboid
        import numpy as np
        self.target_box = DynamicCuboid(
            prim_path="/World/target_car",
            name="target_carton",
            position=np.array([8.0, 0.0, 0.5]),
            scale=np.array([0.6, 0.6, 0.6]),
            color=np.array([0.5, 0.35, 0.2]),
            mass=5.0
        )
        self.car_controller = CircleVehicleController("/World/target_car", radius=8.0, speed=0.4)

        # Spawn Drone (Iris)
        config_multirotor = MultirotorConfig()
        
        px4_dir = "/home/flux/snydrone_ws/drone_stack/PX4-Autopilot"
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": px4_dir,
            "px4_vehicle_model": self.pg.px4_default_airframe
        })

        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
        ]
        
        # Add Camera
        config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"update_rate": 60.0})]
        
        self.drone = Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        self.pg.set_viewport_camera([5.0, 9.0, 6.5], [0.0, 0.0, 0.0])
        self.world.reset()
        
        # Initialize OmniGraph Target Publisher
        create_ros2_target_publisher()

        self.stop_sim = False
        self.camera_graph_created = False

    def run(self):
        self.timeline.play()

        try:
            import omni.replicator.core as rep
            while simulation_app.is_running() and not self.stop_sim:
                self.world.step(render=True)
                rep.orchestrator.step()
                
                if not self.camera_graph_created:
                    create_ros2_camera_publisher("/World/quadrotor/body/camera")
                    self.camera_graph_created = True
                
                # Update target vehicle pos manually (OmniGraph publishes it automatically)
                dt = self.world.get_physics_dt()
                pos, quat = self.car_controller.update(dt)

                # Dynamic Cinematics: Viewport follows the drone/car
                # Let's frame the car from a distance to see the drone panning
                import numpy as np
                from isaacsim.core.utils.viewports import set_camera_view
                cam_pos = np.array([pos[0] + 10.0, pos[1] + 10.0, 6.0])
                target_pos = np.array(pos)
                set_camera_view(eye=cam_pos, target=target_pos)
        except Exception as e:
            import traceback
            traceback.print_exc()
            carb.log_error(f"Error in SnydroneApp run loop: {e}")

        carb.log_warn("SnydroneApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():
    pg_app = SnydroneApp()
    pg_app.run()

if __name__ == "__main__":
    main()
