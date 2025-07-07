#!/usr/bin/env python3
"""
Collision Detector node: reads /obstacles, makes stop/speed decisions.
"""

import math, time, numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header, Float32
from visualization_msgs.msg import Marker, MarkerArray
from navigation_interface.msg import ObstacleArray, Stop
from motor_control_interface.msg import VelAngle
from geometry_msgs.msg import TwistStamped, Point
from tf2_ros import Buffer, TransformListener

class CollisionDetector(Node):

    def __init__(self):
        super().__init__('collision_detector')

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # parameters
        self.declare_parameter('safe_obstacle_dist', 6.0 * 1.25)
        self.declare_parameter('safe_obstacle_time', 2.0 * 1.25)
        self.SAFE_DIST = self.get_parameter('safe_obstacle_dist').value
        self.SAFE_TIME = self.get_parameter('safe_obstacle_time').value

        # State
        self.cur_speed = 0.1
        self.cur_obstacles = []

        # subs & pubs
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.ob_sub    = self.create_subscription(
            ObstacleArray, '/obstacles', self.obstacle_cb, qos)
        self.angle_sub= self.create_subscription(
            VelAngle,      '/nav_cmd', self.angle_cb, 10)
        self.speed_sub= self.create_subscription(
            TwistStamped, '/estimate_twist', self.speed_cb, qos)

        self.stop_pub  = self.create_publisher(Stop, '/stop', 10)
        self.speed_pub = self.create_publisher(Float32, '/speed', 10)
        self.coll_pub  = self.create_publisher(MarkerArray, '/collision_markers', 10)

        # timer
        self.create_timer(1/30.0, self.timer_cb)

    def obstacle_cb(self, msg: ObstacleArray):
        # transform all obstacles into base_link
        out = []
        for obs in msg.obstacles:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', obs.header.frame_id, rclpy.time.Time())
                p = tf2_geometry_msgs.PointStamped()
                p.header = obs.header
                p.point = obs.pos.point
                p_base = tf2_geometry_msgs.do_transform_point(p, trans)
                obs.pos.point = p_base.point
                obs.header.frame_id = 'base_link'
                out.append(obs)
            except:
                continue
        self.cur_obstacles = out

    def angle_cb(self, msg: VelAngle):
        self.req_angle = msg.angle

    def speed_cb(self, msg: TwistStamped):
        self.cur_speed = msg.twist.linear.x or 0.1

    def timer_cb(self):
        markers = MarkerArray()
        clear = True

        for i, obs in enumerate(self.cur_obstacles):
            d = math.hypot(obs.pos.point.x, obs.pos.point.y)
            t = d / self.cur_speed
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = obs.pos.point
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = obs.radius * 2.0
            m.scale.z = 0.2

            if d < (self.SAFE_DIST / 3) or t < (self.SAFE_TIME / 3):
                clear = False
                stop = Stop()
                stop.stop = True
                stop.sender_id.data = 'collision_detector'
                stop.distance = float(d)
                self.stop_pub.publish(stop)
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0
            else:
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5

            markers.markers.append(m)

        if clear:
            stop = Stop()
            stop.stop = False
            stop.sender_id.data = 'collision_detector'
            stop.distance = -1.0
            self.stop_pub.publish(stop)

        self.coll_pub.publish(markers)

    def get_logger(self):
        return super().get_logger()

def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()