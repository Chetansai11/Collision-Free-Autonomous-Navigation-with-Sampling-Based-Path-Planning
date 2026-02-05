#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
import math


class Controller(Node):
    def __init__(self):
        super().__init__('sim1_controller')

        # --- Parameters (faster defaults) ---
        self.declare_parameter('lookahead', 1.0)     
        self.declare_parameter('max_lin', 1.5)       
        self.declare_parameter('max_ang', 3.0)        
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('odom_topic', '/pose')  
        self.declare_parameter('angle_tolerance', 0.1)  
        self.declare_parameter('angular_k', 1.5)     
        self.declare_parameter('linear_k', 1.5)       
        self.declare_parameter('frame_id', 'world')

        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        self.max_lin = self.get_parameter('max_lin').get_parameter_value().double_value
        self.max_ang = self.get_parameter('max_ang').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.angular_k = self.get_parameter('angular_k').get_parameter_value().double_value
        self.linear_k = self.get_parameter('linear_k').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Path & robot state
        self.path = []        # list of (x, y)
        self.path_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_cb, 10)
        self.pose_sub = self.create_subscription(Pose2D, self.odom_topic, self.pose_cb, 10)

        # Executed path (actual trajectory)
        self.exec_path_pub = self.create_publisher(Path, 'executed_path', 10)
        self.executed_path = Path()
        self.executed_path.header.frame_id = self.frame_id

        # Control timer 
        self.timer = self.create_timer(0.03, self.control_loop) 

    # ----------------- Callbacks -----------------
    def path_cb(self, msg: Path):
        # Convert Path to simple list of (x,y)
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        self.path = pts
        self.path_idx = 0

        # Reset executed path on new plan
        self.executed_path = Path()
        self.executed_path.header.frame_id = msg.header.frame_id or self.frame_id
        self.executed_path.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Received path with {len(self.path)} waypoints')

    def pose_cb(self, msg: Pose2D):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_yaw = msg.theta

        # Log actual trajectory while following a path
        if self.path:
            ps = PoseStamped()
            ps.header.frame_id = self.executed_path.header.frame_id
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = self.robot_x
            ps.pose.position.y = self.robot_y
            ps.pose.position.z = 0.0
            # yaw -> quaternion (only z,w)
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
            ps.pose.orientation.w = math.cos(self.robot_yaw / 2.0)

            self.executed_path.poses.append(ps)
            self.exec_path_pub.publish(self.executed_path)

    # ----------------- Helpers -----------------
    def publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    # ----------------- Control loop -----------------
    def control_loop(self):
        if not self.path or self.path_idx >= len(self.path):
            self.publish_stop()
            return

        # --- Pick lookahead point ---
        lx, ly = None, None
        for i in range(self.path_idx, len(self.path)):
            px, py = self.path[i]
            dist = math.hypot(px - self.robot_x, py - self.robot_y)
            if dist >= self.lookahead or i == len(self.path) - 1:
                lx, ly = px, py
                self.path_idx = i
                break
        if lx is None:
            lx, ly = self.path[-1]

        dx = lx - self.robot_x
        dy = ly - self.robot_y
        target_yaw = math.atan2(dy, dx)
        angle_err = self.angle_diff(target_yaw, self.robot_yaw)

        # distance to final goal
        goal_x, goal_y = self.path[-1]
        dist_goal = math.hypot(goal_x - self.robot_x, goal_y - self.robot_y)

        # ---- Goal reached ----
        if dist_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached — stopping')
            self.path = []
            self.publish_stop()
            return

        twist = Twist()

        # Large heading error: rotate in place
        if abs(angle_err) > self.angle_tolerance:
            twist.linear.x = 0.0
            twist.angular.z = max(-self.max_ang,
                                  min(self.max_ang, self.angular_k * angle_err))
        else:
            # Drive towards lookahead point
            dist = math.hypot(dx, dy)
            lin = min(self.max_lin, self.linear_k * dist)
            ang = max(-self.max_ang,
                      min(self.max_ang, self.angular_k * angle_err))
            twist.linear.x = lin
            twist.angular.z = ang

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
