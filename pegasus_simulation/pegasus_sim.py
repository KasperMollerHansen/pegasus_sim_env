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
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
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

import numpy as np
import os.path
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan

enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

class PegasusApp:
    def __init__(self):
        self.topic_prefix = "/isaac"
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.world.scene.add_default_ground_plane()
        self.spawn_ground_plane(scale=[500, 500, 500])
        self.spawn_light()
        self.spawn_windturbine(position=[-5.0, 0, -0.25])
        # MicroXRCEAgent provides a unique topic name for vehicle_id=0
        self.spawn_quadrotor(position=[0, 0, 0], rotation=[0, 0, 180], vehicle_id=0)
        # self.spawn_quadrotor(position=[0, 5, 0], rotation=[0,0,-90], vehicle_id=2)
        self.world.reset()
        self.stop_sim = False

    @staticmethod
    def spawn_ground_plane(scale=[1000, 1000, 1000]):
        XFormPrim(prim_path="/World/defaultGroundPlane", scale=scale)

    def spawn_light(self):
        light = UsdLux.SphereLight.Define(self.world.stage, Sdf.Path("/World/Light"))
        light.CreateRadiusAttr(50.0)
        light.CreateIntensityAttr(1000.0)
        light.AddTranslateOp().Set(Gf.Vec3f(1000.0, 1000.0, 1000.0))

    def spawn_quadrotor(
        self,
        position=[0.0, 0.0, 0.07],
        rotation=[0.0, 0.0, 0.0],
        vehicle_id: int = 0,
        camera: bool = True,
        lidar: bool = True,
    ):
        prim_path = f"/World/quadrotor_{vehicle_id}"
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig(
            {
                "vehicle_id": vehicle_id,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
                "px4_vehicle_model": self.pg.px4_default_airframe,  # CHANGE this line to 'iris' if using PX4 version bellow v1.14
            }
        )
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        Multirotor(
            prim_path,
            ROBOTS["Iris"],
            vehicle_id,
            position,
            Rotation.from_euler("XYZ", rotation, degrees=True).as_quat(),
            config=config_multirotor,
        )

        body_frame = XFormPrim(
            prim_path=prim_path + "/body",
            position=position,
        )

        self._publish_clock()
        frame_prims = []
        frame_prims.append(body_frame.prim_path)
        if camera:
            camera = self._initialize_camera(body_frame, resolution=(640, 480))
            frame_prims.append("/".join(camera.prim_path.split("/")[:-1]))
            camera.initialize()
            self._publish_rgb_camera(camera, vehicle_id)

        if lidar:
            lidar = self._initialize_lidar(body_frame)
            frame_prims.append(
                "/".join(prims_utils.get_prim_path(lidar).split("/")[:-1])
            )
            try:
                self._publish_lidar(lidar, vehicle_id)
            except Exception as e:
                carb.log_error(f"Error publishing lidar: {e}")

        if len(frame_prims) >= 1:
            self._publish_tf(frame_prims)

    def spawn_windturbine(self, position=[0.0, 0.0, -0.25]):
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
    def _initialize_camera(body_frame, resolution=(640, 480)):
        camera_frame = XFormPrim(
            prim_path=body_frame.prim_path + "/camera_frame",
            position=body_frame.get_world_pose()[0]
            + np.array([0.0, 0.0, 0.5]),  # Offset camera frame relative to body frame
        )
        camera = Camera(
            prim_path=camera_frame.prim_path + "/Camera",
            resolution=resolution,
            orientation=rot_utils.euler_angles_to_quats(
                np.array([0.0, 0.0, 0.0]), degrees=True
            ),
        )
        return camera

    def _publish_rgb_camera(self, camera: Camera, vehicle_id, freq: int = 30):
        render_product = camera._render_product_path
        step_size = int(60 / freq)
        topic_name = self.topic_prefix + "/" + camera.name + f"_{vehicle_id}_rgb"
        queue_size = 1
        node_namespace = ""
        frame_id = camera.prim_path

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
            sd.SensorType.Rgb.name
        )
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
        )
        writer.attach([render_product])
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    @staticmethod
    def _initialize_lidar(body_frame):
        lidar_frame = XFormPrim(
            prim_path=body_frame.prim_path + "/lidar_frame",
            position=body_frame.get_world_pose()[0] + np.array([0.0, 0.0, 0.5]),
        )

        lidar_config = "OS1_REV7_128ch10hz1024res"

        _, lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_frame.prim_path + "/Lidar",
            config=lidar_config,
            translation=(0, 0, 1.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
        )
        return lidar

    def _publish_lidar(self, lidar, vehicle_id):
        topic_name = self.topic_prefix + f"/point_cloud_{vehicle_id}"
        og.Controller.edit(
            {"graph_path": "/Graphs/ROS_Lidar", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("RosContext", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "RunSimFrame",
                        "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame",
                    ),
                    (
                        "CreateRenderProduct",
                        "omni.isaac.core_nodes.IsaacCreateRenderProduct",
                    ),
                    ("RTXLidar", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RunSimFrame.inputs:execIn"),
                    ("RunSimFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "RTXLidar.inputs:execIn"),
                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        "RTXLidar.inputs:renderProductPath",
                    ),
                    ("RosContext.outputs:context", "RTXLidar.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RTXLidar.inputs:topicName", f"{topic_name}"),
                    ("RTXLidar.inputs:type", "point_cloud"),
                    ("RTXLidar.inputs:frameId", "lidar_frame"),
                    ("RTXLidar.inputs:fullScan", True),
                    (
                        "CreateRenderProduct.inputs:cameraPrim",
                        f"{prims_utils.get_prim_path(lidar)}",
                    ),
                ],
            },
        )

    def _publish_tf(self, frames):
        topic_name = "/tf"
        og.Controller.edit(
            {"graph_path": "/Graphs/ROS_TF", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("RosContext", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("RosContext.outputs:context", "PublishTF.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishTF.inputs:timeStamp",
                    ),
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF.inputs:topicName", f"{topic_name}"),
                    ("PublishTF.inputs:targetPrims", [f"{frame}" for frame in frames]),
                ],
            },
        )

    def _publish_clock(self):
        topic_name = self.topic_prefix + "/clock"
        og.Controller.edit(
            {"graph_path": "/Graphs/Clock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("RosContext", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("RosContext.outputs:context", "PublishClock.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishClock.inputs:timeStamp",
                    ),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishClock.inputs:topicName", f"{topic_name}"),
                ],
            },
        )

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
