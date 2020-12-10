import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import cv2
import numpy as np
import math
import copy

from matplotlib import pyplot as plt

class Graph():
    """Topology graph from gridmap image"""

    def __init__(self, msg, pose):
        # Extract features
        img = self.convert_map_to_image(msg)
        topology = cv2.ximgproc.thinning(img)
        contours, _ = cv2.findContours(topology, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corners = cv2.goodFeaturesToTrack(topology, 20, 0.1, 6)

        # Search nodes and edges
        self.nodes = list(map(lambda corner:
            [int(corner[0][0]), int(corner[0][1])], corners))
        self.edges = self.search_edges(
            self.nodes,
            list(map(lambda contour: [contour[0][0], contour[0][1]], contours[0])))
        self.nodes_point = list(map(lambda node:
            [-((node[1] * msg.info.resolution) + msg.info.origin.position.y),
             -((node[0] * msg.info.resolution) + msg.info.origin.position.x)], self.nodes))
        self.num_nodes_neighbor = self.count_neighbors(self.nodes, self.edges)
        self.root = self.search_closest_edge(self.nodes_point, self.edges, pose)

        # Visualize graph
        self.init_colors()
        self.nodes_marker, self.edges_marker = self.generate_markers(self.nodes_point, self.num_nodes_neighbor, self.edges, msg)

    def init_colors(self):
        """Initialize colors for node/edge markers"""

        self.COLOR_NODE = ColorRGBA()   # Green
        self.COLOR_NODE.r = 0.53
        self.COLOR_NODE.g = 0.9
        self.COLOR_NODE.b = 0.5
        self.COLOR_NODE.a = 1.

        self.COLOR_EDGE = ColorRGBA()   # Green
        self.COLOR_EDGE.r = 0.53
        self.COLOR_EDGE.g = 0.9
        self.COLOR_EDGE.b = 0.5
        self.COLOR_EDGE.a = 1.

        self.COLOR_ROOT = ColorRGBA()   # Blue
        self.COLOR_ROOT.r = 0.26
        self.COLOR_ROOT.g = 0.45
        self.COLOR_ROOT.b = 0.85
        self.COLOR_ROOT.a = 1.

        self.COLOR_INTERSECTION = ColorRGBA()  # Yellow
        self.COLOR_INTERSECTION.r = 1.
        self.COLOR_INTERSECTION.g = 0.73
        self.COLOR_INTERSECTION.b = 0.
        self.COLOR_INTERSECTION.a = 1.

        self.COLOR_LEAF = ColorRGBA()   # Red
        self.COLOR_LEAF.r = 0.95
        self.COLOR_LEAF.g = 0.37
        self.COLOR_LEAF.b = 0.37
        self.COLOR_LEAF.a = 1.

    def convert_image_to_map(self, img, corners, msg):
        m = OccupancyGrid()
        m.header = msg.header
        m.info.map_load_time = self.get_clock().now().to_msg()
        m.info.resolution = 0.05
        m.info.width = self.scan_map_shape[0]
        m.info.height = self.scan_map_shape[1]
        m.info.origin.position.x = -self.scan_radius
        m.info.origin.position.y = -self.scan_radius
        m.info.origin.orientation.w = 1.0
        m.data = [-1 for i in range(self.scan_map_shape[0] * self.scan_map_shape[1])]
        for i in range(m.info.width):
            for j in range(m.info.height):
                n = m.info.width * j + i
                if img[i, j] > self.occupancy_img_threshold:
                    m.data[n] = 0   # {-1: unknown, 0: free, 100: occupied}
        for corner in corners:
            i = corner[0][1]
            j = corner[0][0]
            n = int(m.info.width * j + i)
            m.data[n] = 100
        return m

    def convert_map_to_image(self, msg):
        img = np.zeros([msg.info.width, msg.info.height], np.uint8)
        for n, d in enumerate(msg.data):
            if(d == 0):
                i = int(n % msg.info.width)
                j = int(n / msg.info.width)
                img[i, j] = 255
        return img
        
    def search_edges(self, nodes, contours, dist_threshold=3):
        """Search node connectivity from contour"""

        # Search closest node index
        nodes_index = [-1] * len(contours)
        for node_idx, p in enumerate(nodes):
            dist = list(map(lambda c: abs(c[0] - p[0]) + abs(c[1] - p[1]), contours))
            count = 0
            d_ = dist_threshold + 1
            for contour_idx, d in enumerate(dist):
                if d <= dist_threshold:
                    count += 1
                elif d_ <= dist_threshold:
                    count = 0
                d_ = d
            neighbor = [0] * len(dist)
            for contour_idx, d in enumerate(dist):
                if d <= dist_threshold:
                    count += 1
                    neighbor[contour_idx] = count
                elif d_ <= dist_threshold:
                    count = 0
                    idx = contour_idx - int(neighbor[contour_idx-1] / 2)
                    nodes_index[idx] = node_idx
                d_ = d

        # Create edges
        n_ = -1
        for n in nodes_index:
            if(n != -1):
                n_ = n
        edges = []
        for n in nodes_index:
            if(n != -1):
                edges.append([n_, n])
                n_ = n

        return edges

    def count_neighbors(self, nodes, edges):
        """Return number of neighbor"""

        num_nodes_neighbor = [0] * len(nodes)
        for edge in edges:
            num_nodes_neighbor[edge[0]] += 1
        return num_nodes_neighbor

    def search_closest_edge(self, nodes_point, edges, pose):
        """Return closest edge index from pose[x, y, theta]"""

        # Search closest node
        nodes_dist = []
        for node_point in nodes_point:
            nodes_dist.append((node_point[0] - pose[0])**2 + (node_point[1] - pose[1])**2)
        closest_node = nodes_dist.index(min(nodes_dist))

        # Search neighbor edges in distance
        edges_neighbor = []
        for edge in edges:
            if(edge[0] == closest_node):
                edges_neighbor.append(edge)
        
        # Search closest edges in angle
        edges_angle = []
        for edge in edges_neighbor:
            angle = math.atan2(
                0.5 * nodes_point[edge[1]][1] - 0.5 * nodes_point[edge[0]][1],
                0.5 * nodes_point[edge[1]][0] - 0.5 * nodes_point[edge[0]][0]
            ) - pose[2]
            edges_angle.append(abs((angle + 3*math.pi) % (2*math.pi) - math.pi))
        
        return edges_neighbor[edges_angle.index(min(edges_angle))]

    def generate_markers(self, nodes_point, num_nodes_neighbor, edges, msg):
        # Add node marker
        nodes_marker = Marker()
        nodes_marker.header = copy.deepcopy(msg.header)
        nodes_marker.header.frame_id = 'base_footprint'
        nodes_marker.ns = 'node'
        nodes_marker.id = 0
        nodes_marker.type = Marker.POINTS
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = 0.1      # Point width
        nodes_marker.scale.y = 0.1      # Point height
        for i, node_point in enumerate(nodes_point):
            p = Point()
            p.x = -node_point[0]        # Transform from base_scan to base_footprint
            p.y = -node_point[1]        # for Rviz
            nodes_marker.points.append(p)
            if(self.num_nodes_neighbor[i] == 1):
                nodes_marker.colors.append(self.COLOR_LEAF)
            elif(self.num_nodes_neighbor[i] > 2):
                nodes_marker.colors.append(self.COLOR_INTERSECTION)
            else:
                nodes_marker.colors.append(self.COLOR_NODE)

        # Add edge marker
        edges_marker = Marker()
        edges_marker.header = copy.deepcopy(msg.header)
        edges_marker.header.frame_id = 'base_footprint'
        edges_marker.ns = 'edge'
        edges_marker.id = 0
        edges_marker.type = Marker.LINE_LIST
        edges_marker.action = Marker.ADD
        edges_marker.scale.x = 0.03     # Line width
        for edge in edges:
            edges_marker.points.append(nodes_marker.points[edge[0]])
            edges_marker.points.append(nodes_marker.points[edge[1]])
            edges_marker.colors.append(self.COLOR_EDGE)
            edges_marker.colors.append(self.COLOR_EDGE)
        
        return nodes_marker, edges_marker

    def update_node_color(self, node, color):
        self.nodes_marker.colors[node] = color

    def update_edge_color(self, edge, color1, color2=0):
        for i, e in enumerate(self.edges):
            if(e == edge or e[::-1] == edge):
                self.edges_marker.colors[2*i] = color1
                self.edges_marker.colors[2*i+1] = color1 if color2 == 0 else color2

class Extract_graph(Node):
    def __init__(self):
        self.scan = LaserScan()
        self.scan_angle_min = -2.35
        self.scan_angle_max = 2.35
        self.scan_radius = 5.0
        self.scan_map_resolution = 0.05
        self.scan_map_shape = (int(2*self.scan_radius/self.scan_map_resolution), int(2*self.scan_radius/self.scan_map_resolution), 1)
        self.scan_map_center = (int(self.scan_map_shape[0]/2), int(self.scan_map_shape[1]/2))
        self.occupancy_img_threshold = 127

        # Initialize ROS node
        super().__init__('waffle_topology_server')
        self.area_subscription = self.create_subscription(OccupancyGrid, '/topology/area', self.area_callback, 1)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.marker_publisher = self.create_publisher(MarkerArray, '/topology/marker', 10)
        self.nodes_publisher = self.create_publisher(Marker, '/topology/nodes', 10)
        self.edges_publisher = self.create_publisher(Marker, '/topology/edges', 10)

        # Initialize history
        self.history = Marker()
        self.history.header.frame_id = 'odom'
        self.history.ns = 'intersection'
        self.history.id = 0
        self.history.type = Marker.POINTS
        self.history.action = Marker.ADD
        self.history.scale.x = 0.1      # Point width
        self.history.scale.y = 0.1      # Point height

        p = Point()
        p.x = 0.
        p.y = 0.
        p.z = 1.
        self.history.points.append(p)
        color = ColorRGBA()
        color.r = 1.
        color.g = 1.
        color.b = 1.
        color.a = 0.5
        self.history.colors.append(color)

        self.history_publisher = self.create_publisher(Marker, '/topology/history', 10)
    
    def area_callback(self, msg):
        # Generate graph from gridmap message
        try:
            graph = Graph(msg, [0, 0, 0])
        except:
            print("Graph generation failed")
            return

        # Color important elements
        # graph.update_node_color(graph.root[1], graph.COLOR_ROOT)
        graph.update_edge_color(graph.root, graph.COLOR_ROOT, graph.COLOR_EDGE)

        # Publish results
        self.nodes_publisher.publish(graph.nodes_marker)
        self.edges_publisher.publish(graph.edges_marker)

        # Log
        self.draw_history(graph)

    def draw_history(self, graph):
        """Draw intersection point on odometry frame"""

        # Transform intersection point into odometry
        try:
            self.robot_pose
        except:
            return
        target = graph.nodes_point[graph.root[1]]

        # target = [0, 0]


        # x = target[0]*math.cos(self.robot_pose[2]) + target[1]*math.sin(self.robot_pose[2]) + self.robot_pose[0]
        # y = target[1]*math.cos(self.robot_pose[2]) - target[0]*math.sin(self.robot_pose[2]) + self.robot_pose[1]
        x = target[0]*math.cos(self.robot_pose[2]) - target[1]*math.sin(self.robot_pose[2]) + self.robot_pose[0]
        y = target[0]*math.sin(self.robot_pose[2]) + target[1]*math.cos(self.robot_pose[2]) + self.robot_pose[1]
        # x = graph.nodes_point[graph.root[0]][0] + self.robot_pose[0]
        # y = graph.nodes_point[graph.root[0]][1] + self.robot_pose[1]

        # x = 1.
        # y = 0.

        print("---")
        print(self.robot_pose)
        print(target)        
        # print(math.degrees(self.robot_pose[2]))

        # Append a marker point
        # p = Point()
        # p.x = -x
        # p.y = -y
        # p.z = 1.
        # self.history.points.append(p)
        # color = ColorRGBA()
        # color.r = 1.
        # color.g = 1.
        # color.b = 1.
        # color.a = 0.1
        # self.history.colors.append(color)
        self.history.points[0].x = -x
        self.history.points[0].y = -y
        self.history_publisher.publish(self.history)

    def odom_callback(self, msg):
        theta = self.quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, theta]
    
    def quaternion_to_euler(self, x, y, z, w):
        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # X = math.atan2(t0, t1)
        # t2 = +2.0 * (w * y - z * x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # Y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
        return Z

def main(args=None):
    rclpy.init(args=args)
    node = Extract_graph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
