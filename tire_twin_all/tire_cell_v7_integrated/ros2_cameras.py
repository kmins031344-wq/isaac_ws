"""
손목 카메라 2대(RealSense D455)의 영상을 ROS 2 토픽으로 내보낸다 (Isaac Sim ROS 2 브리지, OmniGraph).

  토픽 (로봇마다)                           내용
  /robot_a/camera/color/image_raw          컬러 (sensor_msgs/Image, rgb8)
  /robot_a/camera/color/camera_info        컬러 카메라 내부 파라미터
  /robot_a/camera/depth/image_raw          깊이 (32FC1, m)
  /robot_a/camera/depth/camera_info
  (/robot_b/... 너트러너 쪽도 같은 구성)
  /tf                                      world -> 카메라 4개 프레임 (robot_a_color, robot_a_depth, ...)
  /clock                                   시뮬레이션 시간 (use_sim_time:=true 로 쓰면 됨)

run_cell_demo.py 가 씬을 연 뒤 setup() 을 호출한다. 해상도는 --ros-res 1280x800 처럼 바꿀 수 있다. D455 렌즈 비율(16:10)과 다르면 fy 가 fx 로 강제돼 camera_info 가 틀어진다.
"""
import os

GRAPH = "/World/ROS2_Cameras"

# (네임스페이스, 카메라 Root 경로)
ROBOTS = (
    ("robot_a", "/World/RobotA/WheelGripper/Camera_RSD455/RSD455"),
    ("robot_b", "/World/RobotB/Nutrunner/Camera_RSD455/RSD455"),
)
STREAMS = (
    # (스트림, 에셋 안의 카메라 프림, 이미지 type)
    ("color", "Camera_OmniVision_OV9782_Color", "rgb"),
    ("depth", "Camera_Pseudo_Depth", "depth"),
)


def enable_bridge():
    """SimulationApp 생성 직후 호출. 브리지 확장을 켠다."""
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")


def setup(stage, width=640, height=400):
    import omni.graph.core as og
    import usdrt.Sdf
    from pxr import Sdf

    if stage.GetPrimAtPath(GRAPH):
        stage.RemovePrim(GRAPH)
    nodes = [
        ("tick", "omni.graph.action.OnPlaybackTick"),
        ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
        ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ]
    conn = [
        ("tick.outputs:tick", "clock.inputs:execIn"),
        ("ctx.outputs:context", "clock.inputs:context"),
        ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
        ("tick.outputs:tick", "tf.inputs:execIn"),
        ("ctx.outputs:context", "tf.inputs:context"),
        ("simtime.outputs:simulationTime", "tf.inputs:timeStamp"),
    ]
    vals = [
        ("ctx.inputs:domain_id", int(os.environ.get("ROS_DOMAIN_ID", "0"))),
        ("ctx.inputs:useDomainIDEnvVar", True),
        ("tf.inputs:parentPrim", [usdrt.Sdf.Path("/World")]),
    ]
    tf_targets, topics = [], []
    for ns, root in ROBOTS:
        if not stage.GetPrimAtPath(root):
            print("[ros2_cameras] camera missing, skipped:", root)
            continue
        for stream, cam, kind in STREAMS:
            cam_path = root + "/" + cam
            frame = "%s_%s" % (ns, stream)                       # TF 프레임 이름 (두 로봇의 프림 이름이 같아서 덮어씀)
            stage.GetPrimAtPath(cam_path).CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(frame)
            tf_targets.append(usdrt.Sdf.Path(cam_path))
            rp, img, info = "rp_%s_%s" % (ns, stream), "img_%s_%s" % (ns, stream), "info_%s_%s" % (ns, stream)
            nodes += [
                (rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                (img, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ]
            conn += [
                ("tick.outputs:tick", rp + ".inputs:execIn"),
                (rp + ".outputs:execOut", img + ".inputs:execIn"),
                (rp + ".outputs:execOut", info + ".inputs:execIn"),
                (rp + ".outputs:renderProductPath", img + ".inputs:renderProductPath"),
                (rp + ".outputs:renderProductPath", info + ".inputs:renderProductPath"),
                ("ctx.outputs:context", img + ".inputs:context"),
                ("ctx.outputs:context", info + ".inputs:context"),
            ]
            vals += [
                (rp + ".inputs:cameraPrim", [usdrt.Sdf.Path(cam_path)]),
                (rp + ".inputs:width", int(width)),
                (rp + ".inputs:height", int(height)),
                (img + ".inputs:type", kind),
                (img + ".inputs:nodeNamespace", ns),
                (img + ".inputs:topicName", "camera/%s/image_raw" % stream),
                (img + ".inputs:frameId", frame),
                (info + ".inputs:nodeNamespace", ns),
                (info + ".inputs:topicName", "camera/%s/camera_info" % stream),
                (info + ".inputs:frameId", frame),
            ]
            topics.append("/%s/camera/%s/{image_raw,camera_info}" % (ns, stream))
    vals.append(("tf.inputs:targetPrims", tf_targets))
    keys = og.Controller.Keys
    og.Controller.edit({"graph_path": GRAPH, "evaluator_name": "execution"},
                       {keys.CREATE_NODES: nodes, keys.CONNECT: conn, keys.SET_VALUES: vals})
    print("[ros2_cameras] publishing %dx%d:" % (width, height))
    for t in topics + ["/tf", "/clock"]:
        print("[ros2_cameras]   " + t)
