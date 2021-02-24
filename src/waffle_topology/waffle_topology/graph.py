#!/usr/bin/env python3

import cv2
import numpy as np
import math
import copy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

class Node:
    def __init__(self, point:Point):
        self.point = point
        self.neighbors = []
        self.color = Color()

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)
        self.color.edges.append(
            [copy.deepcopy(Color.EDGE), copy.deepcopy(Color.EDGE)])

class Color:
    NODE = ColorRGBA(r=0.53, g=0.9, b=0.5, a=0.2)        # Green
    EDGE = ColorRGBA(r=0.53, g=0.9, b=0.5, a=0.2)        # Green
    INTERSECTION = ColorRGBA(r=1., g=0.73, b=0., a=0.2)  # Yellow
    LEAF = ColorRGBA(r=0.95, g=0.37, b=0.37, a=0.2)      # Red

    def __init__(self):
        self.node = copy.deepcopy(Color.NODE)
        self.edges = []

class Graph:
    """Linked list graph"""

    def __init__(self, msg:OccupancyGrid, pose):
        # Extract features
        img = Graph._convert_map_to_image(msg)
        topology = cv2.ximgproc.thinning(img)
        contours, _ = cv2.findContours(topology, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corners = cv2.goodFeaturesToTrack(topology, 20, 0.1, 6)

        # Search nodes and edges
        nodes_idx = list(map(lambda corner: [
            int(corner[0][0]), int(corner[0][1])], corners))
        edges_idx = Graph._search_edges_idx(
            nodes_idx,
            list(map(lambda contour: [contour[0][0], contour[0][1]], contours[0])))
        
        # Calculate global xy coordinates of nodes
        nodes_point_local = list(map(lambda node: [
            -((node[1] * msg.info.resolution) + msg.info.origin.position.y),
            -((node[0] * msg.info.resolution) + msg.info.origin.position.x)], nodes_idx))
        nodes_point_global = list(map(lambda point: [
            point[0]*math.cos(pose[2]) - point[1]*math.sin(pose[2]) + pose[0],
            point[0]*math.sin(pose[2]) + point[1]*math.cos(pose[2]) + pose[1]], nodes_point_local))

        # Construct nodes
        self.nodes = []
        for p in nodes_point_global:
            point = Point(x=p[0], y=p[1])
            self.add_node(point)
        for edge in edges_idx:
            # self.add_neighbor(edge)
            self.nodes[edge[0]].add_neighbor(self.nodes[edge[1]])            

        # Update nodes color
        for node in self.nodes:
            if(len(node.neighbors) == 1):
                node.color.node = copy.deepcopy(Color.LEAF)
            elif(len(node.neighbors) > 2):
                node.color.node = copy.deepcopy(Color.INTERSECTION)
    
    def add_node(self, point):
        self.nodes.append(Node(point))
        return self.nodes[-1]

    def add_neighbor(self, edge):
        if self.nodes[edge[0]] != self.nodes[edge[1]]:  # Reject circular loop
            self.nodes[edge[0]].neighbors.append(self.nodes[edge[1]])
            self.nodes[edge[0]].color.edges.append(
                [copy.deepcopy(Color.EDGE), copy.deepcopy(Color.EDGE)])

    def _convert_map_to_image(msg:OccupancyGrid):
        img = np.zeros([msg.info.width, msg.info.height], np.uint8)
        for n, d in enumerate(msg.data):
            if(d == 0):
                i = int(n % msg.info.width)
                j = int(n / msg.info.width)
                img[i, j] = 255
        return img

    def quaternion_to_euler(x, y, z, w):
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

    def generate_markers(self, msg):
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
        for node in self.nodes:
            nodes_marker.points.append(Point(x=-node.point.x, y=-node.point.y, z=0.2))
            nodes_marker.colors.append(node.color.node)
            for i, neighbor in enumerate(node.neighbors):
                edges_marker.points.append(Point(x=-node.point.x, y=-node.point.y, z=0.1))
                edges_marker.points.append(Point(x=-neighbor.point.x, y=-neighbor.point.y, z=0.1))
                edges_marker.colors.append(node.color.edges[i][0])
                edges_marker.colors.append(node.color.edges[i][1])

        return nodes_marker, edges_marker
        
    def _search_edges_idx(nodes, contours, dist_threshold=3):
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
                    idx = contour_idx - (neighbor[contour_idx-1] // 2)
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

    def search_closest_edge(self, pose):
        """Return closest edge[departure node, destination node] from pose"""

        # Search closest node
        nodes_dist = []
        for node in self.nodes:
            nodes_dist.append((node.point.x - pose[0])**2 + (node.point.y - pose[1])**2)
        node_closest = self.nodes[nodes_dist.index(min(nodes_dist))]
        edges = list(map(lambda neighbor: [node_closest, neighbor], node_closest.neighbors))

        # Search closest edge
        edges_dist = []
        for edge in edges:
            edge_point = [0.5*(edge[0].point.x + edge[1].point.x),
                 0.5*(edge[0].point.y + edge[1].point.y)]
            edges_dist.append((edge_point[0] - pose[0])**2 + (edge_point[1] - pose[1])**2)
        edge_closest = edges[edges_dist.index(min(edges_dist))]

        # Calculate relative angle of the edge
        edge_angle = math.atan2(
            edge_closest[1].point.x - edge_closest[0].point.x,
            edge_closest[1].point.y - edge_closest[0].point.y
        ) - pose[2]
        edges_angle = (edge_angle + 3*math.pi) % (2*math.pi) - math.pi

        return edge_closest if abs(edges_angle) < math.pi/2 else edge_closest[::-1]

class EmptyGraph(Graph):
    def __init__(self):
        self.nodes = []