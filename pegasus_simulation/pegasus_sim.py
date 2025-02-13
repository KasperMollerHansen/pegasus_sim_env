#!/usr/bin/env python


import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})



from pxr import Gf, UsdLux, Sdf

import omni.timeline
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, is_stage_loading
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.extensions import enable_extension


# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
import numpy as np
import os.path
from scipy.spatial.transform import Rotation


class PegasusApp:
    def __init__(self):

        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.world.scene.add_default_ground_plane()
        self.spawn_ground_plane()
        self.spawn_light()
        self.spawn_windturbine(position=[0, 0, -0.25])
        self.spawn_quadrotor(position=[5, 0, 0])

        self.world.reset()
        self.stop_sim = False

    def spawn_ground_plane(self):
        XFormPrim(
        prim_path = "/World/defaultGroundPlane",
        scale = np.array([1000., 1000., 1000.])
        )

    def spawn_light(self):
        light = UsdLux.SphereLight.Define(self.world.stage, Sdf.Path("/World/Light"))
        light.CreateRadiusAttr(50.)
        light.CreateIntensityAttr(1000.0)
        light.AddTranslateOp().Set(Gf.Vec3f(1000., 1000., 1000.))

    def spawn_quadrotor(
        self, position=[0.0, 0.0, 0.07], camera: bool = True, lidar: bool = True
    ):
        prim_path = "/World/quadrotor"
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
                "px4_vehicle_model": self.pg.px4_default_airframe,  # CHANGE this line to 'iris' if using PX4 version bellow v1.14
            }
        )
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        Multirotor(
            prim_path,
            ROBOTS["Iris"],
            0,
            position,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        body_frame = XFormPrim(
            prim_path=prim_path + "/body",
            position=position,
        )
        if camera:
            camera_frame = XFormPrim(
                prim_path=body_frame.prim_path + "/camera_frame",
                position=body_frame.get_world_pose()[0]
                + np.array(
                    [0.0, 0.0, 0.5]
                ),  # Offset camera frame relative to body frame
            )
            camera = Camera(
                prim_path=camera_frame.prim_path + "/Camera",
                resolution=(640, 480),
                orientation=rot_utils.euler_angles_to_quats(
                    np.array([0.0, 0.0, 0.0]), degrees=True
                ),
            )
            camera.initialize()
            self._publish_rgb_camera(camera)

        if lidar:
            lidar_frame = XFormPrim(
                prim_path=body_frame.prim_path + "/lidar_frame",
                position=body_frame.get_world_pose()[0] + np.array([0.0, 0.0, 0.5]),
            )

            _, lidar = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path=lidar_frame.prim_path + "/Lidar",
                config="Example_Rotary",
                translation=(0, 0, 1.0),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
            )

    def spawn_windturbine(self, position=[0.0, 0.0, -0.25]):
        # Get current path
        windturbine_path = "pegasus_simulation/data/windturbine.usdc"
        add_reference_to_stage(
            usd_path=windturbine_path, prim_path="/World/Windturbine"
        )
        XFormPrim(
            prim_path="/World/Windturbine",
            position=position,
            scale=np.array([0.1, 0.1, 0.1]),  # Default scale is 100
            orientation=rot_utils.euler_angles_to_quats(
                np.array([90.0, 0.0, 180.0]), degrees=True
            ),
        )

    @staticmethod
    def _publish_rgb_camera(camera, freq: int=30):
        render_product = camera._render_product_path
        step_size = int(60/freq)
        topic_name = camera.name+"_rgb"
        queue_size = 1
        node_namespace = ""
        frame_id = camera.prim_path

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId = frame_id,
            nodeNamespace = node_namespace,
            queueSize = queue_size,
            topicName = topic_name
        )
        writer.attach([render_product])
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)



    def run(self):
        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()

    pg_app.run()


if __name__ == "__main__":
    main()
