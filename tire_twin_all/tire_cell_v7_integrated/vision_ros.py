"""
vision_ros.py - 비전 결과를 ROS 2 토픽으로 내보낸다 (rqt / RViz 에서 보기용). Isaac Sim 내장 rclpy 를 쓴다.

  /robot_b/vision/debug_image   sensor_msgs/Image (bgr8)   검출 결과 영상 (vision_debug.png 와 같음) -> rqt_image_view
  /robot_b/vision/nuts          geometry_msgs/PoseArray    너트 5개 목표점, 화살표 = 소켓이 들어가는 방향 -> RViz
  /robot_b/vision/wheel_pose    geometry_msgs/PoseStamped  휠 중심 + 허브 축(z) -> RViz

  rqt 를 늦게 켜도 보이도록 마지막 결과를 주기적으로 다시 보낸다 (1 s 요청, 헤드리스 실측 약 4~5 s 간격. 촬영은 사이클당 한 번뿐).
  frame_id = "World" (ros2_cameras.py 의 /tf 루트 프레임과 같음).
"""
import os
import sys

import numpy as np

FRAME = "World"
_node = {"node": None, "pubs": None, "last": None}


def _import_rclpy():
    try:
        import rclpy  # noqa: F401
    except ImportError:                                   # 브리지 확장의 내장 rclpy 경로 추가
        distro = os.environ.get("ROS_DISTRO", "jazzy")
        for base in sys.path + [os.path.expanduser("~/isaacsim")]:
            p = os.path.join(base.split("/kit/")[0] if "/kit/" in base else base, "exts", "isaacsim.ros2.bridge", distro, "rclpy")
            if os.path.isdir(p) and p not in sys.path:
                sys.path.append(p)
                break
    import rclpy
    return rclpy


def _ensure():
    if _node["node"] is not None:
        return _node
    rclpy = _import_rclpy()
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import Image

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("nut_vision")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)   # 나중에 붙은 RViz 도 마지막 값을 받음
    _node["node"] = node
    _node["pubs"] = (node.create_publisher(Image, "/robot_b/vision/debug_image", qos),
                     node.create_publisher(PoseArray, "/robot_b/vision/nuts", qos),
                     node.create_publisher(PoseStamped, "/robot_b/vision/wheel_pose", qos))
    return _node


def _pose(p, R):
    from geometry_msgs.msg import Pose

    from cell_kinematics import R_to_quat
    w, x, y, z = R_to_quat(R)
    m = Pose()
    m.position.x, m.position.y, m.position.z = (float(v) for v in p)
    m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = float(w), float(x), float(y), float(z)
    return m


def publish(debug_bgr, T_wheel, nuts_world):
    """결과를 기억하고 바로 한 번 보낸다. T_wheel / nuts_world 는 실패 시 None 이어도 됨 (영상만 보냄)."""
    _ensure()
    _node["last"] = (None if debug_bgr is None else np.ascontiguousarray(debug_bgr, dtype=np.uint8),
                     None if T_wheel is None else np.array(T_wheel), None if nuts_world is None else np.array(nuts_world))
    republish()


def republish():
    if _node["node"] is None or _node["last"] is None:
        return
    from geometry_msgs.msg import PoseArray, PoseStamped
    from sensor_msgs.msg import Image

    img, T, nuts = _node["last"]
    p_img, p_nuts, p_wheel = _node["pubs"]
    stamp = _node["node"].get_clock().now().to_msg()
    if img is not None:
        m = Image()
        m.header.frame_id, m.header.stamp = "robot_b_color", stamp
        m.height, m.width = int(img.shape[0]), int(img.shape[1])
        m.encoding, m.is_bigendian, m.step = "bgr8", 0, int(img.shape[1] * 3)
        m.data = img.tobytes()
        p_img.publish(m)
    if T is not None:
        # PoseArray 화살표는 x 축 방향으로 그려진다 -> x 축이 소켓 진입 방향(-허브 축)이 되도록 돌린다
        n = T[:3, 2]
        x_ax = -n
        y_ax = T[:3, 0] - (T[:3, 0] @ x_ax) * x_ax
        y_ax /= np.linalg.norm(y_ax)
        R_arrow = np.column_stack([x_ax, y_ax, np.cross(x_ax, y_ax)])
        pa = PoseArray()
        pa.header.frame_id, pa.header.stamp = FRAME, stamp
        pa.poses = [_pose(p - x_ax * 0.06, R_arrow) for p in nuts]          # 화살표 끝이 너트에 오도록 6 cm 앞에서 시작
        p_nuts.publish(pa)
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = FRAME, stamp
        ps.pose = _pose(T[:3, 3], T[:3, :3])
        p_wheel.publish(ps)


def shutdown():
    if _node["node"] is not None:
        try:
            _node["node"].destroy_node()
        except Exception:
            pass
        _node["node"] = None
