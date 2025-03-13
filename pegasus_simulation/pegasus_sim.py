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
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from omni_graphs import OmniGraphs

import numpy as np
from scipy.spatial.transform import Rotation

enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()


class PegasusApp:
    def __init__(self):
        self.ig = OmniGraphs()

        self.topic_prefix = "/isaac"
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.setup_scene()
        self.world.reset()
        self.stop_sim = False

    def setup_scene(self):
        self.world.scene.add_default_ground_plane()
        self._publish_clock()
        self._spawn_ground_plane(scale=[500, 500, 500])
        self._spawn_light()
        self._spawn_windturbine(position=[-5, 0, -0.25])
        self._spawn_quadrotor(position=[0, 0, 0], rotation=[0, 0, 0], vehicle_id=0)

    @staticmethod
    def _spawn_ground_plane(scale=[1000, 1000, 1000]):
        XFormPrim(prim_path="/World/defaultGroundPlane", scale=scale)
        return

    def _spawn_light(self):
        light = UsdLux.SphereLight.Define(self.world.stage, Sdf.Path("/World/Light"))
        light.CreateRadiusAttr(50.0)
        light.CreateIntensityAttr(1000.0)
        light.AddTranslateOp().Set(Gf.Vec3f(1000.0, 1000.0, 1000.0))
        return

    def _spawn_windturbine(self, position=[0.0, 0.0, -0.25]):
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
        return

    def _spawn_quadrotor(
        self,
        position=[0.0, 0.0, 0.07],
        rotation=[0.0, 0.0, 0.0],
        vehicle_id: int = 0,
        camera: bool = True,
        lidar: bool = True,
    ):
        if vehicle_id == 0:
            quad_frame = XFormPrim(
            prim_path="/quadrotor",
            position=position,
            )
            prim_path = quad_frame.prim_path + f"/quadrotor"
        else:
            quad_frame = XFormPrim(
            prim_path=f"/quadrotor_{vehicle_id}",
            position=position,
            )
            prim_path = quad_frame.prim_path + f"/quadrotor_{vehicle_id}"
        
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

        odom_frame = XFormPrim(
            prim_path=prim_path + "/odom",
            position=position,
        )

        body_frame = XFormPrim(
            prim_path=prim_path + "/body",
            position=position,
        )

        frame_prims = []
        base_link_frame = self._initialize_base_link_frame(body_frame)

        # Initialize Camera if enabled
        if camera:
            camera = self._initialize_camera(body_frame, resolution=(640, 480))
            camera_frame_path = "/".join(
                camera.prim_path.split("/")[:-1]
            )  # Get the parent path
            frame_prims.append(camera_frame_path)
            camera.initialize()
            self._publish_camera(camera, vehicle_id)

        if lidar:
            lidar = self._initialize_lidar(body_frame)
            lidar_frame_path = "/".join(
                prims_utils.get_prim_path(lidar).split("/")[:-1]
            )
            frame_prims.append(lidar_frame_path)
            try:
                self._publish_lidar(lidar, vehicle_id)
            except Exception as e:
                carb.log_error(f"Error publishing lidar: {e}")

        # Publish the TF tree, ensuring the correct hierarchy
        if len(frame_prims) >= 1:
            self._publish_tf(
                odom_frame.prim_path, base_link_frame.prim_path, frame_prims
            )
        return

    @staticmethod
    def _initialize_base_link_frame(body_frame):
        base_link_frame = XFormPrim(
            prim_path=body_frame.prim_path + "/base_link",
            position=body_frame.get_world_pose()[0],
        )
        return base_link_frame

    @staticmethod
    def _initialize_camera(body_frame, resolution=(640, 480)):
        camera_frame = XFormPrim(
            prim_path=body_frame.prim_path + "/camera_frame",
            position=body_frame.get_world_pose()[0]
            + np.array([0.0, 0.0, 0.25]),  # Offset camera frame relative to body frame
        )
        camera_prim = camera_frame.prim_path + "/Camera"
        camera = Camera(
            prim_path=camera_prim,
            resolution=resolution,
            orientation=rot_utils.euler_angles_to_quats(
                np.array([0.0, 0.0, 0.0]), degrees=True
            ),
        )
        return camera
        from omni.isaac.ros2_bridge import read_camera_info
        # The following code will link the camera's render product and publish the data to the specified topic name.
        render_product = camera._render_product_path
        step_size = int(60/freq)
        if vehicle_id == 0:
            topic_name = self.topic_prefix +"/" + camera.name+"_camera_info"
        else:
            topic_name = self.topic_prefix +"/" + camera.name+f"_camera_info_{vehicle_id}"
        queue_size = 1
        node_namespace = ""
        frame_id = camera.prim_path

        writer = rep.writers.get("ROS2PublishCameraInfo")
        camera_info = read_camera_info(render_product_path=render_product)
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
            width=camera_info["width"],
            height=camera_info["height"],
            projectionType=camera_info["projectionType"],
            k=camera_info["k"].reshape([1, 9]),
            r=camera_info["r"].reshape([1, 9]),
            p=camera_info["p"].reshape([1, 12]),
            physicalDistortionModel=camera_info["physicalDistortionModel"],
            physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", render_product
        )

        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

        return

    @staticmethod
    def _initialize_lidar(body_frame):
        lidar_frame = XFormPrim(
            prim_path=body_frame.prim_path + "/lidar_frame",
            position=body_frame.get_world_pose()[0] + np.array([0.0, 0.0, 0.25]),
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
    
    def _publish_camera(self, camera, vehicle_id):
        frame_id = "camera_frame"
        if vehicle_id == 0:
            topic_name = self.topic_prefix + "/camera"
        else:
            topic_name = self.topic_prefix + f"/camera_{vehicle_id}"
        self.ig.camera_graph(camera, topic_name, frame_id)
        return
   
    def _publish_lidar(self, lidar, vehicle_id):
        if vehicle_id == 0:
            topic_name = self.topic_prefix + "/point_cloud"
        else:
            topic_name = self.topic_prefix + f"/point_cloud_{vehicle_id}"
        self.ig.lidar_graph(lidar, topic_name)
    
    @staticmethod
    def _publish_tf(odom, base_link, frames):
        topic_name = "/tf"
        og.Controller.edit(
            {"graph_path": "/Graphs/ROS_TF", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("RosContext", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishODOM", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("RosContext.outputs:context", "PublishODOM.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishODOM.inputs:timeStamp",
                    ),
                    ("OnPlaybackTick.outputs:tick", "PublishODOM.inputs:execIn"),
                    ("RosContext.outputs:context", "PublishTF.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishTF.inputs:timeStamp",
                    ),
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishODOM.inputs:topicName", f"{topic_name}"),
                    ("PublishODOM.inputs:parentPrim", f"{odom}"),
                    ("PublishODOM.inputs:targetPrims", f"{base_link}"),
                    ("PublishTF.inputs:topicName", f"{topic_name}"),
                    ("PublishTF.inputs:parentPrim", f"{base_link}"),
                    ("PublishTF.inputs:targetPrims", [f"{frame}" for frame in frames]),
                ],
            },
        )

    def _publish_clock(self):
        topic_name = "/clock"
        self.ig.clock_graph(topic_name)

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
