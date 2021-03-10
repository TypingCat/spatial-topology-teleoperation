#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray

from waffle_topology.graph import Graph, EmptyGraph, Clustering
from waffle_topology.visualization import create_graph_marker, create_intersection_marker
from waffle_topology.calculation import quaternion_to_euler

class Generate_topology(Node):
    def __init__(self):
        # Set parameters
        self.scan_radius = 5.0
        self.scan_map_resolution = 0.05
        self.clustering = Clustering(
            cluster_threshold=1.,
            cluster_min=10,
            cluster_max=500)

        self.topology = EmptyGraph()
        self.scan = LaserScan()
        self.scan_map_shape = (
            int((2*self.scan_radius) // self.scan_map_resolution),
            int((2*self.scan_radius) // self.scan_map_resolution), 1)
        self.scan_map_center = (
            int(self.scan_map_shape[0] // 2),
            int(self.scan_map_shape[1] // 2))

        # Initialize a ROS node
        super().__init__('waffle_topology_generate_topology')
        self.area_subscription = self.create_subscription(OccupancyGrid, '/topology/area', self.area_callback, 1)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.nodes_publisher = self.create_publisher(Marker, '/topology/nodes', 10)
        self.edges_publisher = self.create_publisher(Marker, '/topology/edges', 10)
        self.cluster_publisher = self.create_publisher(MarkerArray, '/topology/intersections', 10)

    def area_callback(self, msg):
        # Generate graph from gridmap message
        try:
            local_topology = Graph(msg, self.robot_pose)
        except:
            self.get_logger().warning("Graph generation failed")
            return

        # Intersection node clustering
        self.clustering.append([(node.point.x, node.point.y) for node in local_topology.nodes if len(node.neighbors) > 2])

        # Visualize graph
        nodes_marker, edges_marker = create_graph_marker(local_topology.nodes, msg.header)
        self.nodes_publisher.publish(nodes_marker)
        self.edges_publisher.publish(edges_marker)

        intersections_marker = []
        for label, cluster_info in self.clustering.info.items():
            intersections_marker.append(create_intersection_marker(
                label=label,
                center=cluster_info['mean'],
                axis=cluster_info['components'],
                size=cluster_info['std'],
                header=msg.header))
        self.cluster_publisher.publish(MarkerArray(markers=intersections_marker))

    def odom_callback(self, msg):
        _, _, yaw = quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

def main(args=None):
    rclpy.init(args=args)
    node = Generate_topology()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()