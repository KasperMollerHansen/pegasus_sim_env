import omni.graph.core as og
import omni.isaac.core.utils.prims as prims_utils 

class IssacGraphs:

    @staticmethod
    def camera_graph(camera, topic_name, frame_id, resolution: tuple = (640, 480)):
        og.Controller.edit(
        {"graph_path": "/Graphs/ROS_Camera", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Ros2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                
                ("RunSimFrame","omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                ("CreateRenderProduct","omni.isaac.core_nodes.IsaacCreateRenderProduct"),

                ("CameraRGB","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("CameraDepth","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("CameraInfo","omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),

                ("ROS2QoS", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunSimFrame.inputs:execIn"),
                ("RunSimFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),
                
                ("CreateRenderProduct.outputs:execOut", "CameraRGB.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraDepth.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),

                ("CreateRenderProduct.outputs:renderProductPath","CameraRGB.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath","CameraDepth.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath","CameraInfo.inputs:renderProductPath"),
                
                ("Ros2Context.outputs:context", "CameraRGB.inputs:context"),
                ("Ros2Context.outputs:context", "CameraDepth.inputs:context"),
                ("Ros2Context.outputs:context", "CameraInfo.inputs:context"),

                ("ROS2QoS.outputs:qosProfile", "CameraRGB.inputs:qosProfile"),
                ("ROS2QoS.outputs:qosProfile", "CameraDepth.inputs:qosProfile"),
                ("ROS2QoS.outputs:qosProfile", "CameraInfo.inputs:qosProfile"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CameraRGB.inputs:topicName", f"{topic_name}/rgb"),
                ("CameraRGB.inputs:type", "rgb"),
                ("CameraRGB.inputs:frameId", f"{frame_id}"),

                ("CameraDepth.inputs:topicName", f"{topic_name}/depth"),
                ("CameraDepth.inputs:type", "depth"),
                ("CameraDepth.inputs:frameId", f"{frame_id}"),

                ("CameraInfo.inputs:topicName", f"{topic_name}/info"),
                ("CameraInfo.inputs:frameId", f"{frame_id}"),

                (
                    "CreateRenderProduct.inputs:cameraPrim",
                    f"{camera.prim_path}",
                ),
                ("CreateRenderProduct.inputs:height", resolution[1]),
                ("CreateRenderProduct.inputs:width", resolution[0]),
            ],
        },
    )
        
    @staticmethod
    def lidar_graph(lidar, topic_name):
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
        
    @staticmethod
    def clock_graph(topic_name):
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
