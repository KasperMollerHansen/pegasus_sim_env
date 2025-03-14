import omni
from pxr import Gf
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np

from omni_graphs import OmniGraphs

class StereoCamera():

    def __init__(self, topic_prefix, drone_prim_path, vehicle_id:int=0, translation:tuple=(0.0,0.0,0.0),orientation:tuple=(1.0, 0.0, 0.0, 0.0),distance_between_cameras:float=0.1, resolution:tuple=(640, 480)):
        self.topic_prefix = topic_prefix
        self.drone_prim_path = drone_prim_path
        self.body_prim_path = drone_prim_path + "/body"
        self.vehicle_id = vehicle_id
        self.translation = translation
        self.orientation = orientation
        self.delta_cam = distance_between_cameras/2
        self.resolution = resolution

        self.omni_graphs = OmniGraphs()

        self._initialize_camera()
        self._publish_camera()
        return
    
    def _initialize_camera(self):
        stereo_prim = XFormPrim(
            prim_path = self.body_prim_path + "/stereo_camera",
            translation = self.translation,
            orientation = self.orientation,
        )
        left_prim = XFormPrim(
            prim_path = stereo_prim.prim_path + "/left",
            translation = (0.0, self.delta_cam, 0.0),
        )
        right_prim = XFormPrim(
            prim_path = stereo_prim.prim_path + "/right",
            translation = (0.0, -self.delta_cam, 0.0),
        )
        left_camera = Camera(
            prim_path=left_prim.prim_path + "/camera_left",
            resolution=self.resolution,
        )
        right_camera = Camera(
            prim_path=right_prim.prim_path + "/camera_right",
            resolution=self.resolution,
        )
        left_camera.initialize()
        right_camera.initialize()

        self.camera_prims = (left_camera, right_camera)
        self.camera_frame_ids = ("camera_left", "camera_right")
        return
    
    def _publish_camera(self):
        if self.vehicle_id == 0:
            namespace = self.topic_prefix + "/stereo_camera"
        else:
            namespace = self.topic_prefix + f"/stereo_camera_{self.vehicle_id}"

        prim_path = self.drone_prim_path
        self.omni_graphs.stereo_camera_graph(prim_path, namespace, self.camera_prims, self.camera_frame_ids, self.resolution)
        return
        
class RTXLidar():

    def __init__(self, topic_prefix, drone_prim_path, vehicle_id:int=0, translation:tuple=(0.0,0.0,0.0),orientation:tuple=(1.0, 0.0, 0.0, 0.0)):
        self.topic_prefix = topic_prefix
        self.drone_prim_path = drone_prim_path
        self.body_prim_path = drone_prim_path + "/body"
        self.vehicle_id = vehicle_id
        self.translation = translation
        self.orientation = orientation

        self.omni_graphs = OmniGraphs()

        self._initialize_lidar()
        self._publish_lidar()
        return
        
    def _initialize_lidar(self):
        lidar_frame = XFormPrim(
            prim_path=self.body_prim_path + "/lidar",
            translation = self.translation,
            orientation = self.orientation,
        )

        lidar_config = "OS1_REV7_128ch10hz1024res"

        _, self.lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_frame.prim_path + "/rtx_lidar",
            config=lidar_config,
            orientation=Gf.Quatd(*self.orientation),
        )
        self.lidar_id = "rtx_lidar"
        return
    
    def _publish_lidar(self):
        if self.vehicle_id == 0:
            namespace = self.topic_prefix + "/lidar"
        else:
            namespace = self.topic_prefix + f"/lidar_{self.vehicle_id}"
        prim_path = self.drone_prim_path
        lidar_prim_path = str(self.lidar.GetPath())

        self.omni_graphs.lidar_graph(prim_path, namespace, lidar_prim_path, self.lidar_id)
        return


        