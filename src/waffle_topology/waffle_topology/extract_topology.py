#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import math
import copy

from waffle_topology.graph import Graph

class Extract_topology(Node):
    def __init__(self):
        # Set parameters
        self.scan_radius = 5.0
        self.scan_map_resolution = 0.05

        self.scan = LaserScan()
        self.scan_map_shape = (
            int((2*self.scan_radius) // self.scan_map_resolution),
            int((2*self.scan_radius) // self.scan_map_resolution), 1)
        self.scan_map_center = (
            int(self.scan_map_shape[0] // 2),
            int(self.scan_map_shape[1] // 2))

        # Initialize a ROS node
        super().__init__('waffle_topology_extract_topology')
        self.area_subscription = self.create_subscription(OccupancyGrid, '/topology/area', self.area_callback, 1)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.nodes_publisher = self.create_publisher(Marker, '/topology/nodes', 10)
        self.edges_publisher = self.create_publisher(Marker, '/topology/edges', 10)
        
    def area_callback(self, msg):
        # Generate graph from gridmap message
        try:
            topology = Graph(msg, self.robot_pose)
        except:
            print("Graph generation failed", msg.header.stamp.sec)
            return

        # Search root
        root = topology.search_closest_edge(self.robot_pose)
        root_idx = root[0].neighbors.index(root[1])
        root[1].color.node.a = 1.
        root[0].color.edges[root_idx][0].a = 1.
        root[0].color.edges[root_idx][1].a = 1.

        # Visualize graph
        nodes_marker, edges_marker = self.generate_markers(msg, topology)
        self.nodes_publisher.publish(nodes_marker)
        self.edges_publisher.publish(edges_marker)

    def odom_callback(self, msg):
        yaw = self.quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
    
    def quaternion_to_euler(self, x, y, z, w):
        """Only returns yaw"""

        # t0 = 2.*(w*x + y*z)
        # t1 = 1. - 2.*(x*x + y*y)
        # roll = math.atan2(t0, t1)
        # t2 = 2.*(w*y - z*x)
        # t2 = 1. if t2 > 1. else t2
        # t2 = -1. if t2 < -1. else t2
        # pitch = math.asin(t2)
        t3 = 2.*(w*z + x*y)
        t4 = 1. - 2.*(y*y + z*z)
        yaw = math.atan2(t3, t4)
        return yaw

    def generate_markers(self, msg, graph:Graph):
        # Initialize node marker
        nodes_marker = Marker()
        nodes_marker.header = copy.deepcopy(msg.header)
        nodes_marker.header.frame_id = 'odom'
        nodes_marker.ns = 'node'
        nodes_marker.id = 0
        nodes_marker.type = Marker.POINTS
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = 0.1      # Point width
        nodes_marker.scale.y = 0.1      # Point height

        # Initialize edge marker
        edges_marker = Marker()
        edges_marker.header = copy.deepcopy(msg.header)
        edges_marker.header.frame_id = 'odom'
        edges_marker.ns = 'edge'
        edges_marker.id = 0
        edges_marker.type = Marker.LINE_LIST
        edges_marker.action = Marker.ADD
        edges_marker.scale.x = 0.03     # Line width

        # Fill in the rest
        for node in graph.nodes:
            nodes_marker.points.append(Point(x=-node.point.x, y=-node.point.y, z=0.1))
            nodes_marker.colors.append(node.color.node)
            for i, neighbor in enumerate(node.neighbors):
                edges_marker.points.append(Point(x=-node.point.x, y=-node.point.y, z=0.1))
                edges_marker.points.append(Point(x=-neighbor.point.x, y=-neighbor.point.y, z=0.1))
                edges_marker.colors.append(node.color.edges[i][0])
                edges_marker.colors.append(node.color.edges[i][1])

        return nodes_marker, edges_marker

def main(args=None):
    rclpy.init(args=args)
    node = Extract_topology()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()