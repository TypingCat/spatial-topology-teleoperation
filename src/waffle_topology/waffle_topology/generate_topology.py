#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from waffle_topology.graph import Graph
from waffle_topology.graph import EmptyGraph

class Generate_topology(Node):
    def __init__(self):
        # Set parameters
        self.scan_radius = 5.0
        self.scan_map_resolution = 0.05

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
        
    def area_callback(self, msg):
        # Generate graph from gridmap message
        try:
            g = Graph(msg, self.robot_pose)
        except:
            self.get_logger().warning("Graph generation failed")
            return

        # Search root
        # try:
        #     root = g.search_closest_edge(self.robot_pose)
        #     root_idx = root[0].neighbors.index(root[1])
        #     root[1].color.node.a = 1.
        #     root[0].color.edges[root_idx][0].a = 1.
        #     root[0].color.edges[root_idx][1].a = 1.
        # except:
        #     pass

        # Save intersections
        # for node in g.nodes:
        #     if len(node.neighbors) > 2:
        #         self.topology.nodes.append(node)
        # print(len(self.topology.nodes))

        # Save local topology
        self.topology = g
        print(len(self.topology.nodes))

        # Visualize graph
        nodes_marker, edges_marker = self.topology.generate_markers(msg)
        self.nodes_publisher.publish(nodes_marker)
        self.edges_publisher.publish(edges_marker)

    def odom_callback(self, msg):
        yaw = Graph.quaternion_to_euler(
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