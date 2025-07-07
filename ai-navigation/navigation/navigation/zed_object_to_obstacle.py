#!/usr/bin/env python3
"""
This ROS 2 node converts detected ZED objects into Obstacles for the cart.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from navigation_interface.msg import Obstacle, ObstacleArray
from zed_msgs.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # needed for do_transform_point

class ZedObstacleConverter(Node):

    def __init__(self):
        super().__init__('obstacle_converter')

        # Parameters
        self.declare_parameter('objects_in', '/zed_front/zed_node_0/obj_det/objects')
        self.declare_parameter('obstacles_out', '/obstacles')
        self.declare_parameter('marker_frame', '/base_link')
        self.declare_parameter('marker_topic', '/obs_visualization')

        topic_in   = self.get_parameter('objects_in').value
        topic_out  = self.get_parameter('obstacles_out').value
        mark_topic = self.get_parameter('marker_topic').value
        mark_frame = self.get_parameter('marker_frame').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.object_sub = self.create_subscription(
            ObjectsStamped, topic_in, self.receive_objects, qos)

        self.obstacle_pub = self.create_publisher(ObstacleArray, topic_out, 10)
        self.display_pub  = self.create_publisher(Marker, mark_topic, 10)

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Listening for ZED objects on "{topic_in}"')

    def receive_objects(self, msg: ObjectsStamped):
        # Transform and publish
        obstacles = ObstacleArray()
        obstacles.header = msg.header

        for i, obj in enumerate(msg.objects):
            # transform the point into marker_frame
            pt = tf2_geometry_msgs.PointStamped()
            pt.header = msg.header
            pt.point.x = float(obj.position[0])
            pt.point.y = float(obj.position[1])
            pt.point.z = float(obj.position[2])

            try:
                trans = self.tf_buffer.lookup_transform(
                    self.get_parameter('marker_frame').value,
                    msg.header.frame_id,
                    rclpy.time.Time())
                pt_base = tf2_geometry_msgs.do_transform_point(pt, trans)
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
                continue

            # fill Obstacle
            obs = Obstacle()
            obs.header = self.get_clock().now().to_msg(), 
            obs.header.frame_id = self.get_parameter('marker_frame').value
            obs.pos = pt_base
            obs.radius = float(max(obj.dimensions_3d[0], obj.dimensions_3d[2]))
            obstacles.obstacles.append(obs)

            # publish a cylinder marker
            marker = Marker()
            marker.header.frame_id = obs.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'zed_obs'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = obs.pos.point
            marker.pose.orientation.w = 1.0
            r = obs.radius * 2.0
            marker.scale.x = r
            marker.scale.y = r
            marker.scale.z = 0.3
            marker.color.r = 0.5
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            self.display_pub.publish(marker)

        self.obstacle_pub.publish(obstacles)
        self.get_logger().info(f'Published {len(obstacles.obstacles)} obstacles')

def main(args=None):
    rclpy.init(args=args)
    node = ZedObstacleConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()