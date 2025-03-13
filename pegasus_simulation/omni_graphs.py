import omni.graph.core as og
import omni.isaac.core.utils.prims as prims_utils 

class OmniGraphs:

    @staticmethod
    def camera_graph(camera, topic_name, frame_id, resolution: tuple = (640, 480)):
        og.Controller.edit(
        {"graph_path": "/Graphs/ROS_Camera", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_run_one_simulation_frame","omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                ("isaac_create_render_product","omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("camera_publish_image","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("camera_publish_depth","omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("ros2_camera_info_helper","omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                ("ros2_qos_profile", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
            ],
            og.Controller.Keys.CONNECT: [
                # isaac_run_one_simulation_frame inputs
                ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
                # isaac_create_render_product inputs
                ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
                # camera_publish_image inputs
                ("ros2_context.outputs:context", "camera_publish_image.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "camera_publish_image.inputs:qosProfile"),
                ("isaac_create_render_product.outputs:execOut", "camera_publish_image.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath","camera_publish_image.inputs:renderProductPath"),
                # camera_publish_depth inputs
                ("ros2_context.outputs:context", "camera_publish_depth.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "camera_publish_depth.inputs:qosProfile"),
                ("isaac_create_render_product.outputs:execOut", "camera_publish_depth.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath","camera_publish_depth.inputs:renderProductPath"),
                # ros2_camera_info_helper inputs
                ("ros2_context.outputs:context", "ros2_camera_info_helper.inputs:context"),
                ("ros2_qos_profile.outputs:qosProfile", "ros2_camera_info_helper.inputs:qosProfile"),
                ("isaac_create_render_product.outputs:execOut", "ros2_camera_info_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath","ros2_camera_info_helper.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # camera_publish_image inputs
                ("camera_publish_image.inputs:topicName", f"{topic_name}/rgb"),
                ("camera_publish_image.inputs:type", "rgb"),
                ("camera_publish_image.inputs:frameId", f"{frame_id}"),
                # camera_publish_depth inputs
                ("camera_publish_depth.inputs:topicName", f"{topic_name}/depth"),
                ("camera_publish_depth.inputs:type", "depth"),
                ("camera_publish_depth.inputs:frameId", f"{frame_id}"),
                # ros2_camera_info_helper inputs
                ("ros2_camera_info_helper.inputs:topicName", f"{topic_name}/info"),
                ("ros2_camera_info_helper.inputs:frameId", f"{frame_id}"),
                # isaac_create_render_product inputs
                ("isaac_create_render_product.inputs:cameraPrim",f"{camera.prim_path}",),
                ("isaac_create_render_product.inputs:height", resolution[1]),
                ("isaac_create_render_product.inputs:width", resolution[0]),
            ],
        },
    )
        
    @staticmethod
    def lidar_graph(lidar, topic_name):
        og.Controller.edit(
            {"graph_path": "/Graphs/ROS_Lidar", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("isaac_run_one_simulation_frame","omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                    ("isaac_create_render_product","omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("rtx_lidar", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    # isaac_run_one_simulation_frame inputs
                    ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
                    # isaac_create_render_product inputs
                    ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
                    # rtx_lidar inputs
                    ("isaac_create_render_product.outputs:execOut", "rtx_lidar.inputs:execIn"),
                    ("isaac_create_render_product.outputs:renderProductPath","rtx_lidar.inputs:renderProductPath"),
                    ("ros2_context.outputs:context", "rtx_lidar.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # rtx_lidar inputs
                    ("rtx_lidar.inputs:topicName", f"{topic_name}"),
                    ("rtx_lidar.inputs:type", "point_cloud"),
                    ("rtx_lidar.inputs:frameId", "lidar_frame"),
                    ("rtx_lidar.inputs:fullScan", True),
                    # isaac_create_render_product inputs
                    ("isaac_create_render_product.inputs:cameraPrim",f"{prims_utils.get_prim_path(lidar)}"),
                ],
            },
        )
        
    @staticmethod
    def tf_graph(base_link_prim, sensor_prims, body_prim, rotor_prims, topic_prefix):
        og.Controller.edit(
            {"graph_path": "/Graphs/ROS_TF", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_qos_profile", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
                    ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("tf_tree_base_link_to_sensors", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("tf_tree_base_link_to_body", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("isaac_compute_odometry_node", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("ros2_publish_odometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ("ros2_publish_raw_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    # tf_tree_base_link_to_sensors inputs
                    ("on_playback_tick.outputs:tick", "tf_tree_base_link_to_sensors.inputs:execIn"),
                    ("ros2_context.outputs:context", "tf_tree_base_link_to_sensors.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "tf_tree_base_link_to_sensors.inputs:timeStamp"),
                    ("ros2_qos_profile.outputs:qosProfile", "tf_tree_base_link_to_sensors.inputs:qosProfile"),
                    # tf_tree_base_link_to_body inputs
                    ("on_playback_tick.outputs:tick", "tf_tree_base_link_to_body.inputs:execIn"),
                    ("ros2_context.outputs:context", "tf_tree_base_link_to_body.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "tf_tree_base_link_to_body.inputs:timeStamp"),
                    ("ros2_qos_profile.outputs:qosProfile", "tf_tree_base_link_to_body.inputs:qosProfile"),
                    # isaac_compute_odometry_node inputs
                    ("on_playback_tick.outputs:tick", "isaac_compute_odometry_node.inputs:execIn"),
                    # ros2_publish_odometry inputs
                    ("on_playback_tick.outputs:tick", "ros2_publish_odometry.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_publish_odometry.inputs:context"),
                    ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_odometry.inputs:qosProfile"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_odometry.inputs:timeStamp"),
                    ("isaac_compute_odometry_node.outputs:angularVelocity", "ros2_publish_odometry.inputs:angularVelocity"),
                    ("isaac_compute_odometry_node.outputs:linearVelocity", "ros2_publish_odometry.inputs:linearVelocity"),
                    ("isaac_compute_odometry_node.outputs:orientation", "ros2_publish_odometry.inputs:orientation"),
                    ("isaac_compute_odometry_node.outputs:position", "ros2_publish_odometry.inputs:position"),
                    # ros2_publish_raw_transform_tree inputs
                    ("on_playback_tick.outputs:tick", "ros2_publish_raw_transform_tree.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_publish_raw_transform_tree.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_raw_transform_tree.inputs:timeStamp"),
                    ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_raw_transform_tree.inputs:qosProfile"),
                    ("isaac_compute_odometry_node.outputs:position", "ros2_publish_raw_transform_tree.inputs:translation"),
                    ("isaac_compute_odometry_node.outputs:orientation", "ros2_publish_raw_transform_tree.inputs:rotation"),
                                     
                ],
                og.Controller.Keys.SET_VALUES: [
                    # tf_tree_base_link_to_sensors inputs
                    ("tf_tree_base_link_to_sensors.inputs:parentPrim", f"{base_link_prim}"),
                    ("tf_tree_base_link_to_sensors.inputs:targetPrims", [f"{prims}" for prims in sensor_prims]),
                    # tf_tree_base_link_to_body inputs
                    ("tf_tree_base_link_to_body.inputs:parentPrim", f"{base_link_prim}"),
                    ("tf_tree_base_link_to_body.inputs:targetPrims", f"{body_prim}"),
                    # # isaac_compute_odometry_node inputs
                    ("isaac_compute_odometry_node.inputs:chassisPrim", f"{body_prim}"),
                    # ros2_publish_odometry inputs
                    ("ros2_publish_odometry.inputs:topicName", f"{topic_prefix}/odom"),
                ],

            },
        )

    @staticmethod
    def clock_graph(topic_name):
        og.Controller.edit(
            {"graph_path": "/Graphs/ros_clock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_publish_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    # ros2_publish_clock inputs
                    ("ros2_context.outputs:context", "ros2_publish_clock.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime","ros2_publish_clock.inputs:timeStamp"),
                    ("on_playback_tick.outputs:tick", "ros2_publish_clock.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # ros2_publish_clock inputs
                    ("ros2_publish_clock.inputs:topicName", f"{topic_name}"),
                ],
            },
        )
