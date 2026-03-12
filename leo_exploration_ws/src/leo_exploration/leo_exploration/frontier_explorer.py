#!/usr/bin/env python3
"""
Leo Rover Frontier-Based Autonomous Exploration Node
ROS2 Jazzy  |  v3.0  - Continuous Wavefront Frontier Detection (WFD)
No-Spin / Ghost Wall Prevention Edition
"""

import math
import subprocess
from collections import deque
from enum import Enum, auto
from typing import Deque, List, Optional, Tuple

import numpy as np

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from nav2_msgs.action import NavigateToPose

from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

# ============================================================================
#  WFD Constants
# ============================================================================
OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 5
COSTMAP_LETHAL_THRESH = 70

# ============================================================================
#  WFD Support Classes
# ============================================================================
class OccupancyGrid2d:
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map_msg):
        self.map = map_msg

    def getCost(self, mx, my):
        return self.map.data[self._getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        if my >= self.map.info.height or mx >= self.map.info.width:
            raise Exception("Out of bounds")
        return (mx, my)

    def _getIndex(self, mx, my):
        return my * self.map.info.width + mx

class FrontierCache:
    def __init__(self):
        self.cache = {}

    def getPoint(self, x, y):
        idx = self._cantorHash(x, y)
        if idx in self.cache:
            return self.cache[idx]
        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def _cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache.clear()

class FrontierPoint:
    __slots__ = ("classification", "mapX", "mapY")
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8

def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x / length, sum_y / length

def getNeighbors(point, costmap, fCache):
    neighbors = []
    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and
                y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))
    return neighbors

def isFrontierPoint(point, costmap, fCache):
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)
        if cost > OCC_THRESHOLD:
            return False
        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True
    return hasFree

def findFree(mx, my, costmap):
    fCache = FrontierCache()
    bfs = [fCache.getPoint(mx, my)]
    while len(bfs) > 0:
        loc = bfs.pop(0)
        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)
        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)
    return (mx, my)

def getFrontier(pose, costmap, logger):
    fCache = FrontierCache()
    fCache.clear()

    try:
        mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
    except Exception:
        logger.warn("Robot position out of map bounds for WFD")
        return []

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value |
                                       PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (
                            PointClassification.FrontierOpen.value |
                            PointClassification.FrontierClosed.value |
                            PointClassification.MapClosed.value
                        ) == 0:
                            w.classification = w.classification | PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification = q.classification | PointClassification.FrontierClosed.value

            newFrontierCords = []
            for x in newFrontier:
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(centroid(newFrontierCords))

        for v in getNeighbors(p, costmap, fCache):
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers

# ============================================================================
#  State Machine & Frontier Data
# ============================================================================
class State(Enum):
    SELECT_FRONTIER = auto()
    NAVIGATING      = auto()
    AVOIDING        = auto()
    COMPLETE        = auto()

class Frontier:
    __slots__ = ("cx", "cy", "score")
    def __init__(self, cx: float, cy: float):
        self.cx = cx
        self.cy = cy
        self.score = 0.0

# ============================================================================
#  Main node
# ============================================================================
class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("min_frontier_size", 5)
        self.declare_parameter("occ_threshold", 10)
        self.declare_parameter("obstacle_dist", 0.45)
        self.declare_parameter("obstacle_half_angle", 60.0) # 120 degree front sector
        self.declare_parameter("nav_timeout", 35.0)
        self.declare_parameter("backup_speed", -0.18)
        self.declare_parameter("backup_duration", 2.0)
        self.declare_parameter("complete_no_frontier", 8)
        self.declare_parameter("log_interval", 12.0)
        self.declare_parameter("save_map_on_complete", True)
        self.declare_parameter("map_save_path", "/tmp/leo_explored_map")

        self.p_robot_frame = self.get_parameter("robot_frame").value
        self.p_map_frame = self.get_parameter("map_frame").value
        self.p_min_frontier = self.get_parameter("min_frontier_size").value
        self.p_occ_threshold = self.get_parameter("occ_threshold").value
        self.p_obs_dist = self.get_parameter("obstacle_dist").value
        self.p_obs_half_angle = math.radians(self.get_parameter("obstacle_half_angle").value)
        self.p_nav_timeout = self.get_parameter("nav_timeout").value
        self.p_backup_speed = self.get_parameter("backup_speed").value
        self.p_backup_dur = self.get_parameter("backup_duration").value
        self.p_no_front_done = self.get_parameter("complete_no_frontier").value
        self.p_log_interval = self.get_parameter("log_interval").value
        self.p_save_map = self.get_parameter("save_map_on_complete").value
        self.p_map_path = self.get_parameter("map_save_path").value

        # Global definitions update
        global MIN_FRONTIER_SIZE, OCC_THRESHOLD
        MIN_FRONTIER_SIZE = self.p_min_frontier
        OCC_THRESHOLD = self.p_occ_threshold

        self.state = State.SELECT_FRONTIER
        self.map_data = None
        self.costmap_data = None
        self.latest_scan = None
        self._enabled = True

        self.rx = self.ry = self.ryaw = 0.0

        self._goal_handle = None
        self._nav_t0 = None
        self._nav_done = False
        self._nav_ok = False

        self.total_fail = 0
        self.recov_spins = 0

        self._avoid_t0 = None

        self._visited: Deque[Tuple[float, float]] = deque(maxlen=24)
        self._completed = False

        self._wfd_costmap = None

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        tl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, "/map", self._map_cb, tl_qos)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._costmap_cb, tl_qos)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Bool, "/explore/enable", self._enable_cb, 10)

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._viz_pub = self.create_publisher(MarkerArray, "/frontiers", 10)

        self._nav_ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.2, self._ctrl_loop)
        self.create_timer(self.p_log_interval, self._log_progress)

        self.get_logger().info("Frontier Explorer Started (WFD, No-Spin Edition).")

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data = msg
        self._wfd_costmap = OccupancyGrid2d(msg)

    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap_data = msg

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _enable_cb(self, msg: Bool):
        self._enabled = msg.data
        if not self._enabled:
            self._stop()
            self._cancel_nav()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_in_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buf.lookup_transform(self.p_map_frame, self.p_robot_frame, rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return tx, ty, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _obstacle_in_sector(self) -> Tuple[bool, float]:
        """Check for front 120 degree sector (±60 deg)."""
        scan = self.latest_scan
        if scan is None:
            return False, float("inf")
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = np.arange(len(ranges), dtype=np.float32) * scan.angle_increment + scan.angle_min
        
        valid = (np.isfinite(ranges) & (ranges > 0.01) & (np.abs(angles) <= self.p_obs_half_angle))
        if not np.any(valid):
            return False, float("inf")
        min_d = float(np.min(ranges[valid]))
        return min_d < self.p_obs_dist, min_d

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _drive(self, vx: float, wz: float = 0.0):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self._cmd_vel_pub.publish(t)

    def _drive_forward(self, speed: float = 0.15, duration: float = 3.0):
        if not hasattr(self, '_fwd_t0') or self._fwd_t0 is None:
            self._fwd_t0 = self._now_sec()
            self.get_logger().info(f"Driving forward gently to explore...")
        if self._now_sec() - self._fwd_t0 < duration:
            self._drive(speed)
        else:
            self._stop()
            self._fwd_t0 = None
            self.state = State.SELECT_FRONTIER

    def _map_stats(self) -> Tuple[int, int, int, float]:
        if self.map_data is None:
            return 0, 0, 0, 0.0
        d = np.array(self.map_data.data, dtype=np.int8)
        free = int(np.sum(d == 0))
        occ = int(np.sum((d > 0) & (d <= 100)))
        unk = int(np.sum(d == -1))
        pct = (free + occ) / d.size * 100.0 if d.size > 0 else 0.0
        return free, occ, unk, pct

    def _detect_frontiers_wfd(self, rx: float, ry: float) -> List[Frontier]:
        if self._wfd_costmap is None:
            return []

        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = rx
        pose.position.y = ry

        try:
            raw_frontiers = getFrontier(pose, self._wfd_costmap, self.get_logger())
        except Exception as e:
            self.get_logger().warn(f"WFD exception: {e}")
            return []

        frontiers = []
        for (wx, wy) in raw_frontiers:
            if not self._frontier_in_inflation(wx, wy):
                frontiers.append(Frontier(wx, wy))
        return frontiers

    def _frontier_in_inflation(self, wx: float, wy: float) -> bool:
        cm = self.costmap_data
        if cm is None:
            return False
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        w = cm.info.width
        h = cm.info.height
        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        if not (0 <= col < w and 0 <= row < h):
            return False
        cost = cm.data[row * w + col]
        return 0 <= cost < 255 and cost >= COSTMAP_LETHAL_THRESH

    def _score_frontiers(self, frontiers: List[Frontier], rx: float, ry: float, ryaw: float) -> List[Frontier]:
        result = []
        for f in frontiers:
            dist = math.hypot(f.cx - rx, f.cy - ry)
            if dist < 0.5:
                continue

            # Prefer closer frontiers without sharp turns back
            # Continuous mode: prefer the one that is ahead.
            angle_to = math.atan2(f.cy - ry, f.cx - rx)
            angle_diff = abs(math.atan2(math.sin(angle_to - ryaw), math.cos(angle_to - ryaw)))
            
            # Penalize frontiers behind us to prevent 180 deg spins
            # angle_diff is [0, pi]. 0 means ahead, pi means behind.
            dir_score = max(0.0, 1.0 - (angle_diff / math.pi))
            
            # Distance scoring (prefer nearby smooth points)
            if dist <= 4.0:
                dist_score = 1.0 - abs(dist - 2.0) / 4.0
            else:
                dist_score = max(0.0, 1.0 - (dist - 4.0) / 6.0)

            visit_pen = 0.0
            for vx, vy in self._visited:
                if math.hypot(f.cx - vx, f.cy - vy) < 0.6:
                    visit_pen = 0.50
                    break

            f.score = (0.5 * dist_score) + (0.4 * dir_score) - visit_pen
            result.append(f)

        result.sort(key=lambda f: f.score, reverse=True)
        return result

    def _send_goal(self, fx: float, fy: float) -> bool:
        if not self._nav_ac.server_is_ready():
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.p_map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = fx
        goal.pose.pose.position.y = fy
        goal.pose.pose.orientation.w = 1.0 # No desired orientation, allow Nav2 to just reach point.

        self._nav_done = False
        self._nav_ok = False
        self._nav_t0 = self._now_sec()

        fut = self._nav_ac.send_goal_async(goal)
        fut.add_done_callback(self._goal_resp_cb)
        self.get_logger().info(f"Navigating to -> ({fx:.2f}, {fy:.2f})")
        return True

    def _goal_resp_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self._nav_done = True
            self._nav_ok = False
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        self._nav_ok = (status == GoalStatus.STATUS_SUCCEEDED)
        self._nav_done = True
        self._goal_handle = None

    def _cancel_nav(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._nav_done = True

    def _publish_frontiers(self, frontiers: List[Frontier]):
        ma = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        stamp = self.get_clock().now().to_msg()
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.p_map_frame
            m.header.stamp = stamp
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = f.cx
            m.pose.position.y = f.cy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.05
            s = max(0.0, min(1.0, f.score + 0.5))
            m.color.r = 1.0 - s
            m.color.g = s
            m.color.b = 0.2
            m.color.a = 0.85
            ma.markers.append(m)
        self._viz_pub.publish(ma)

    def _log_progress(self):
        free, occ, unk, pct = self._map_stats()
        if free + occ > 0:
            self.get_logger().info(f"Exploration: {pct:.1f}% explored. State={self.state.name}")

    def _ctrl_loop(self):
        if not self._enabled:
            return

        now = self._now_sec()
        if self.state == State.SELECT_FRONTIER:
            self._state_select(now)
        elif self.state == State.NAVIGATING:
            self._state_navigating(now)
        elif self.state == State.AVOIDING:
            self._state_avoiding(now)
        elif self.state == State.COMPLETE:
            self._state_complete(now)

    def _state_select(self, now: float):
        if self.map_data is None:
            return

        pose = self._robot_in_map()
        if pose is None:
            return
        rx, ry, ryaw = pose
        self.rx, self.ry, self.ryaw = rx, ry, ryaw

        raw_frontiers = self._detect_frontiers_wfd(rx, ry)
        if not raw_frontiers:
            self.recov_spins += 1
            if self.recov_spins >= self.p_no_front_done:
                self.state = State.COMPLETE
                return
            # Slightly backup as a small "recovery" instead of spin
            self.state = State.AVOIDING
            self._avoid_t0 = now
            return

        self.recov_spins = 0
        scored = self._score_frontiers(raw_frontiers, rx, ry, ryaw)
        self._publish_frontiers(scored[:20])

        if not scored:
            # No frontiers left after filtering, drive forward briefly to reveal space
            self._drive_forward(speed=0.15, duration=2.0)
            return

        best = scored[0]
        self._visited.append((best.cx, best.cy))

        if self._send_goal(best.cx, best.cy):
            self.state = State.NAVIGATING

    def _state_navigating(self, now: float):
        # Emergency 120 deg checking
        obs, dist = self._obstacle_in_sector()
        if obs and dist < self.p_obs_dist:
            self.get_logger().warn(f"Obstacle detected at {dist:.2f} m. Stopping & avoiding.")
            self._cancel_nav()
            self._avoid_t0 = now
            self.state = State.AVOIDING
            return

        if self._nav_done:
            self._nav_done = False
            self.state = State.SELECT_FRONTIER
            return

        if self._nav_t0 is not None and (now - self._nav_t0) > self.p_nav_timeout:
            self._cancel_nav()
            self._nav_done = False
            self.total_fail += 1
            self.state = State.SELECT_FRONTIER

    def _state_avoiding(self, now: float):
        # Simply back up straight for `backup_duration`, then recalculate
        elapsed = now - (self._avoid_t0 or now)
        if elapsed < self.p_backup_dur:
            self._drive(self.p_backup_speed, 0.0) # Straight reverse, no rotation
        else:
            self._stop()
            self.state = State.SELECT_FRONTIER

    def _state_complete(self, now: float):
        if self._completed:
            return
        self._completed = True
        self._stop()
        self.get_logger().info("EXPLORATION COMPLETE - Map exhausted.")
        if self.p_save_map:
            subprocess.Popen(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", self.p_map_path],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
