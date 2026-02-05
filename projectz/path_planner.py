#!/usr/bin/env python3
"""
Planner node (ROS2 Python)

- Subscribes to /pose (Pose2D) to know robot start
- Listens for /goal_pose (PoseStamped) from RViz
- Can plan using:
    * A* on an occupancy grid  (planner_type = "astar")
    * RRT* in continuous space (planner_type = "rrtstar")
- Publishes nav_msgs/Path on 'planned_path'
- Publishes MarkerArray for circular obstacles
- All path and markers use a configurable frame_id (default: "world")
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math
import heapq
import time
import random


def parse_obstacles_string(s: str):
    """Parse 'x1,y1,r1; x2,y2,r2; ...' into list of (x, y, r)."""
    obs = []
    if s is None:
        return obs
    s = s.strip()
    if not s:
        return obs
    for token in s.split(';'):
        tok = token.strip()
        if not tok:
            continue
        parts = [p.strip() for p in tok.split(',')]
        if len(parts) != 3:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            r = float(parts[2])
        except Exception:
            continue
        obs.append((x, y, r))
    return obs


class Planner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # ---------- PARAMETERS ----------
        self.declare_parameter('obstacles', '')           # "x,y,r; x,y,r; ..."
        self.declare_parameter('map_min_x', -6.0)
        self.declare_parameter('map_max_x', 6.0)
        self.declare_parameter('map_min_y', -6.0)
        self.declare_parameter('map_max_y', 6.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('frame_id', 'world')     
        self.declare_parameter('publish_markers', True)
        # choose between "astar" and "rrtstar"
        self.declare_parameter('planner_type', 'rrtstar')

        # RRT* parameters
        self.declare_parameter('rrt_max_iters', 2000)
        self.declare_parameter('rrt_step_size', 0.4)
        self.declare_parameter('rrt_goal_radius', 0.5)
        self.declare_parameter('rrt_goal_sample_rate', 0.10)
        self.declare_parameter('rrt_rewire_radius', 0.8)

        # read parameters
        self.obstacles = parse_obstacles_string(self.get_parameter('obstacles').value)
        self.map_min_x = float(self.get_parameter('map_min_x').value)
        self.map_max_x = float(self.get_parameter('map_max_x').value)
        self.map_min_y = float(self.get_parameter('map_min_y').value)
        self.map_max_y = float(self.get_parameter('map_max_y').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_markers_flag = bool(self.get_parameter('publish_markers').value)
        self.planner_type = str(self.get_parameter('planner_type').value).lower()

        self.rrt_max_iters = int(self.get_parameter('rrt_max_iters').value)
        self.rrt_step_size = float(self.get_parameter('rrt_step_size').value)
        self.rrt_goal_radius = float(self.get_parameter('rrt_goal_radius').value)
        self.rrt_goal_sample_rate = float(self.get_parameter('rrt_goal_sample_rate').value)
        self.rrt_rewire_radius = float(self.get_parameter('rrt_rewire_radius').value)

        self.get_logger().info(f'Parsed obstacles: {self.obstacles}')
        self.get_logger().info(f'Frame id: {self.frame_id}')
        self.get_logger().info(f'Planner type: {self.planner_type}')

        # ---------- GRID (for A*) ----------
        self.width = max(1, int(math.ceil((self.map_max_x - self.map_min_x) / self.resolution)))
        self.height = max(1, int(math.ceil((self.map_max_y - self.map_min_y) / self.resolution)))
        self.get_logger().info(f'Grid = {self.width} x {self.height} (res {self.resolution})')

        # occupancy grid: 1 = occupied (inflated by robot radius), 0 = free
        self.occupancy = np.zeros((self.width, self.height), dtype=np.uint8)
        self.build_occupancy()

        # ---------- PUBLISHERS / SUBSCRIBERS ----------
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacles', 10)

        # sim publishes Pose2D on /pose
        self.pose_sub = self.create_subscription(Pose2D, '/pose', self.pose_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # robot pose cache
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # prebuild marker array
        self.marker_array = self.build_marker_array()
        if self.publish_markers_flag:
            self.marker_pub.publish(self.marker_array)
        self.marker_timer = self.create_timer(0.5, self._publish_marker_array)

        # seed randomness for RRT*
        random.seed()
        np.random.seed()

        self.get_logger().info('Planner ready and listening for /goal_pose')

    # -------------------- pose handling --------------------
    def pose_cb(self, msg: Pose2D):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_yaw = msg.theta

    # -------------------- world/cell conversion --------------------
    def world_to_cell(self, x, y):
        cx = int(round((x - self.map_min_x) / self.resolution))
        cy = int(round((y - self.map_min_y) / self.resolution))
        return cx, cy

    def cell_to_world(self, cx, cy):
        x = self.map_min_x + cx * self.resolution
        y = self.map_min_y + cy * self.resolution
        return x, y

    # -------------------- occupancy --------------------
    def build_occupancy(self):
        """Inflate circular obstacles by robot_radius and mark occupancy grid."""
        self.occupancy.fill(0)
        rr = self.robot_radius
        for (ox, oy, orad) in self.obstacles:
            inflated = orad + rr + 1e-9
            min_cx = max(0, int(math.floor((ox - inflated - self.map_min_x) / self.resolution)))
            max_cx = min(self.width - 1, int(math.ceil((ox + inflated - self.map_min_x) / self.resolution)))
            min_cy = max(0, int(math.floor((oy - inflated - self.map_min_y) / self.resolution)))
            max_cy = min(self.height - 1, int(math.ceil((oy + inflated - self.map_min_y) / self.resolution)))
            for cx in range(min_cx, max_cx + 1):
                wx = self.map_min_x + cx * self.resolution
                for cy in range(min_cy, max_cy + 1):
                    wy = self.map_min_y + cy * self.resolution
                    if (wx - ox) ** 2 + (wy - oy) ** 2 <= inflated ** 2:
                        self.occupancy[cx, cy] = 1

    # -------------------- marker array (stable) --------------------
    def build_marker_array(self):
        arr = MarkerArray()
        cur_t = self.get_clock().now().to_msg()
        id_counter = 0

        for (ox, oy, orad) in self.obstacles:
            # main obstacle cylinder (red)
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = cur_t
            m.ns = "obstacles"
            m.id = id_counter; id_counter += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.0
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = orad * 2.0
            m.scale.y = orad * 2.0
            m.scale.z = 0.1
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
            m.lifetime.sec = 0; m.lifetime.nanosec = 0
            arr.markers.append(m)

            # inflated obstacle (orange, translucent)
            m2 = Marker()
            m2.header = m.header
            m2.ns = "obstacles_inflated"
            m2.id = id_counter; id_counter += 1
            m2.type = Marker.CYLINDER
            m2.action = Marker.ADD
            m2.pose = m.pose
            m2.scale.x = (orad + self.robot_radius) * 2.0
            m2.scale.y = (orad + self.robot_radius) * 2.0
            m2.scale.z = 0.05
            m2.color = ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.35)
            m2.lifetime.sec = 0; m2.lifetime.nanosec = 0
            arr.markers.append(m2)

        return arr

    def _publish_marker_array(self):
        if not self.publish_markers_flag:
            return
        now = self.get_clock().now().to_msg()
        for m in self.marker_array.markers:
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.action = Marker.ADD
        self.marker_pub.publish(self.marker_array)

    # ======================================================
    #                   COLLISION CHECKING
    # ======================================================
    def point_in_collision(self, x, y):
        """Check if a point (world coords) is inside any inflated obstacle."""
        for (ox, oy, orad) in self.obstacles:
            inflated = orad + self.robot_radius
            if (x - ox) ** 2 + (y - oy) ** 2 <= inflated ** 2:
                return True
        return False

    def segment_collision_free(self, p1, p2):
        """
        Check if the line segment p1->p2 is free of collisions
        against all inflated circular obstacles.
        p1, p2: (x, y) in world coordinates
        """
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq < 1e-9:
            # just check point
            return not self.point_in_collision(x1, y1)

        for (ox, oy, orad) in self.obstacles:
            inflated = orad + self.robot_radius
            # projection of circle center onto segment
            t = ((ox - x1) * dx + (oy - y1) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            px = x1 + t * dx
            py = y1 + t * dy
            dist_sq = (px - ox) ** 2 + (py - oy) ** 2
            if dist_sq <= inflated ** 2:
                return False
        return True

    # ======================================================
    #                        A*
    # ======================================================
    def neighbors(self, cx, cy):
        # 8-connected grid
        for dx, dy in [(-1, -1), (-1, 0), (-1, 1),
                       (0, -1),           (0, 1),
                       (1, -1),  (1, 0),  (1, 1)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.occupancy[nx, ny] == 0:
                    yield nx, ny

    def astar(self, start, goal, max_iters=None):
        sx, sy = start
        gx, gy = goal
        if max_iters is None:
            max_iters = self.width * self.height * 4

        open_heap = []
        gscore = {}
        came_from = {}

        gscore[(sx, sy)] = 0.0
        heapq.heappush(open_heap, (self._heuristic(sx, sy, gx, gy), (sx, sy)))

        iters = 0
        while open_heap and iters < max_iters:
            iters += 1
            _, (cx, cy) = heapq.heappop(open_heap)
            if (cx, cy) == (gx, gy):
                return self._reconstruct((sx, sy), (gx, gy), came_from)

            for (nx, ny) in self.neighbors(cx, cy):
                tentative = gscore.get((cx, cy), float('inf')) + math.hypot(nx - cx, ny - cy)
                if tentative < gscore.get((nx, ny), float('inf')):
                    gscore[(nx, ny)] = tentative
                    f = tentative + self._heuristic(cx=nx, cy=ny, gx=gx, gy=gy)
                    heapq.heappush(open_heap, (f, (nx, ny)))
                    came_from[(nx, ny)] = (cx, cy)
        return None

    def _heuristic(self, cx, cy, gx, gy):
        wx, wy = self.cell_to_world(cx, cy)
        gxw, gyw = self.cell_to_world(gx, gy)
        return math.hypot(gxw - wx, gyw - wy)

    def _reconstruct(self, start_cell, goal_cell, came_from):
        path = [goal_cell]
        cur = goal_cell
        while cur != start_cell:
            cur = came_from.get(cur, start_cell)
            path.append(cur)
            if len(path) > (self.width * self.height):
                break
        path.reverse()
        return path

    # ======================================================
    #                        RRT*
    # ======================================================
    def plan_rrt_star(self, start_xy, goal_xy):
        """
        Basic RRT* implementation in continuous (x, y) space.
        Returns list of (x, y) waypoints from start to near goal,
        or None if no path found.
        """
        sx, sy = start_xy
        gx, gy = goal_xy

        # reject if start or goal in collision
        if self.point_in_collision(sx, sy):
            self.get_logger().warn("Start is in collision; RRT* cannot plan.")
            return None
        if self.point_in_collision(gx, gy):
            self.get_logger().warn("Goal is in collision; RRT* cannot plan.")
            return None

        class NodeR:
            __slots__ = ('x', 'y', 'parent', 'cost')
            def __init__(self, x, y, parent=-1, cost=0.0):
                self.x = x
                self.y = y
                self.parent = parent
                self.cost = cost

        nodes = [NodeR(sx, sy, parent=-1, cost=0.0)]
        goal_indices = []

        def dist(a_x, a_y, b_x, b_y):
            return math.hypot(b_x - a_x, b_y - a_y)

        for it in range(self.rrt_max_iters):
            # sample random point (with some goal bias)
            if random.random() < self.rrt_goal_sample_rate:
                rx, ry = gx, gy
            else:
                rx = random.uniform(self.map_min_x, self.map_max_x)
                ry = random.uniform(self.map_min_y, self.map_max_y)

            # find nearest node
            dists = [dist(n.x, n.y, rx, ry) for n in nodes]
            nearest_idx = int(np.argmin(dists))
            nearest = nodes[nearest_idx]

            # steer from nearest towards random sample
            theta = math.atan2(ry - nearest.y, rx - nearest.x)
            new_x = nearest.x + self.rrt_step_size * math.cos(theta)
            new_y = nearest.y + self.rrt_step_size * math.sin(theta)

            # keep inside map bounds
            if not (self.map_min_x <= new_x <= self.map_max_x and
                    self.map_min_y <= new_y <= self.map_max_y):
                continue

            # collision check for edge
            if not self.segment_collision_free((nearest.x, nearest.y), (new_x, new_y)):
                continue

            new_cost = nearest.cost + dist(nearest.x, nearest.y, new_x, new_y)
            new_parent = nearest_idx

            # find neighbors for possible better parent / rewiring
            neighbor_indices = []
            for i, n in enumerate(nodes):
                if dist(n.x, n.y, new_x, new_y) <= self.rrt_rewire_radius:
                    neighbor_indices.append(i)

            # choose best parent among neighbors
            for i in neighbor_indices:
                ni = nodes[i]
                if self.segment_collision_free((ni.x, ni.y), (new_x, new_y)):
                    cand_cost = ni.cost + dist(ni.x, ni.y, new_x, new_y)
                    if cand_cost + 1e-6 < new_cost:
                        new_cost = cand_cost
                        new_parent = i

            # add new node
            new_idx = len(nodes)
            nodes.append(NodeR(new_x, new_y, parent=new_parent, cost=new_cost))

            # rewire neighbors through new node if better
            for i in neighbor_indices:
                ni = nodes[i]
                c_old = ni.cost
                c_new_via_new = new_cost + dist(ni.x, ni.y, new_x, new_y)
                if c_new_via_new + 1e-6 < c_old:
                    if self.segment_collision_free((ni.x, ni.y), (new_x, new_y)):
                        ni.parent = new_idx
                        ni.cost = c_new_via_new

            # check if this new node is close enough to goal
            if dist(new_x, new_y, gx, gy) <= self.rrt_goal_radius:
                goal_indices.append(new_idx)

        if not goal_indices:
            self.get_logger().warn("RRT* could not connect to goal.")
            return None

        # pick best goal connection
        best_idx = min(goal_indices, key=lambda i: nodes[i].cost + dist(nodes[i].x, nodes[i].y, gx, gy))

        # reconstruct path
        path_xy = []
        cur = best_idx
        while cur != -1:
            n = nodes[cur]
            path_xy.append((n.x, n.y))
            cur = n.parent
            if len(path_xy) > len(nodes) + 5:  # safety
                break
        path_xy.reverse()

        # append exact goal as last point (if collision-free)
        if self.segment_collision_free(path_xy[-1], (gx, gy)):
            path_xy.append((gx, gy))

        return path_xy

    # ======================================================
    #                    GOAL HANDLING
    # ======================================================
    def goal_cb(self, msg: PoseStamped):
        if self.robot_x is None:
            self.get_logger().warn('No robot pose yet — ignoring goal.')
            return

        sx_world = self.robot_x
        sy_world = self.robot_y
        tx_world = msg.pose.position.x
        ty_world = msg.pose.position.y

        self.get_logger().info(
            f'Received goal ({tx_world:.3f}, {ty_world:.3f}) — planning from robot ({sx_world:.3f},{sy_world:.3f}) '
            f'using {self.planner_type.upper()}'
        )

        # check: goal inside inflated obstacle?
        if self.point_in_collision(tx_world, ty_world):
            self.get_logger().warn('Goal located inside or too close to obstacle; ignoring.')
            return

        t0 = time.time()

        if self.planner_type == 'astar':
            # ----- grid-based A* -----
            scx, scy = self.world_to_cell(sx_world, sy_world)
            gcx, gcy = self.world_to_cell(tx_world, ty_world)

            if not (0 <= scx < self.width and 0 <= scy < self.height):
                self.get_logger().warn('Start outside map bounds; ignoring goal.')
                return
            if not (0 <= gcx < self.width and 0 <= gcy < self.height):
                self.get_logger().warn('Goal outside map bounds; ignoring goal.')
                return
            if self.occupancy[gcx, gcy]:
                self.get_logger().warn('Goal cell occupied; unreachable.')
                return

            path_cells = self.astar((scx, scy), (gcx, gcy))
            if path_cells is None:
                self.get_logger().warn('No path found (A* failed).')
                return

            # Convert path cells to world coordinates
            path_xy = [self.cell_to_world(cx, cy) for (cx, cy) in path_cells]

        else:
            # ----- RRT* in continuous space -----
            path_xy = self.plan_rrt_star((sx_world, sy_world), (tx_world, ty_world))
            if path_xy is None:
                # try fallback to A* once
                self.get_logger().warn('RRT* failed, falling back to A*.')
                scx, scy = self.world_to_cell(sx_world, sy_world)
                gcx, gcy = self.world_to_cell(tx_world, ty_world)
                if not (0 <= scx < self.width and 0 <= scy < self.height) or \
                   not (0 <= gcx < self.width and 0 <= gcy < self.height) or \
                   self.occupancy[gcx, gcy]:
                    return
                path_cells = self.astar((scx, scy), (gcx, gcy))
                if path_cells is None:
                    self.get_logger().warn('No path found (A* fallback also failed).')
                    return
                path_xy = [self.cell_to_world(cx, cy) for (cx, cy) in path_cells]

        # Publish Path
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for (xw, yw) in path_xy:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = xw
            ps.pose.position.y = yw
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Published planned_path with {len(path_msg.poses)} waypoints '
            f'(plan time {time.time()-t0:.3f}s, planner={self.planner_type})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
