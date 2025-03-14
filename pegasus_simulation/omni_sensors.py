from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera

from omni_graphs import OmniGraphs

class StereoCamera():

    def __init__(self, topic_prefix, drone_prim_path, vehicle_id:int=0, resolution:tuple=(640, 480)):
        self.topic_prefix = topic_prefix
        self.drone_prim_path = drone_prim_path
        self.body_prim_path = drone_prim_path + "/body"
        self.vehicle_id = vehicle_id
        self.resolution = resolution

        self.omni_graphs = OmniGraphs()

        self._initialize_camera()
        self._publish_camera()
        return
    
    def _initialize_camera(self):
        stereo_prim = XFormPrim(
            prim_path = self.body_prim_path + "/stereo_camera",
            translation = (0.1, 0.0, 0.25),
        )
        left_prim = XFormPrim(
            prim_path = stereo_prim.prim_path + "/left",
            translation = (0.0, 0.05, 0.0),
        )
        right_prim = XFormPrim(
            prim_path = stereo_prim.prim_path + "/right",
            translation = (0.0, -0.05, 0.0),
        )
        left_camera = Camera(
            prim_path=left_prim.prim_path + "/camera_left",
            resolution=self.resolution,
            translation=(0, 0, 0.0),
        )
        right_camera = Camera(
            prim_path=right_prim.prim_path + "/camera_right",
            resolution=self.resolution,
            translation=(0, 0, 0.0),
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
        

