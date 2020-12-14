#!/usr/bin/env python3

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

import cv2
import numpy as np
import math
import copy

class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbors = []

class Graph():
    """Topology graph from gridmap image"""

    def __init__(self, msg, pose):
        # Extract features
        img = self.convert_map_to_image(msg)
        topology = cv2.ximgproc.thinning(img)
        contours, _ = cv2.findContours(topology, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corners = cv2.goodFeaturesToTrack(topology, 20, 0.1, 6)

        # Search nodes and edges
        self.nodes = list(map(lambda corner: [
            int(corner[0][0]), int(corner[0][1])], corners))
        self.edges = self.search_edges(
            self.nodes,
            list(map(lambda contour: [contour[0][0], contour[0][1]], contours[0])))

        # Calculate global xy coordinates of nodes
        nodes_point_local = list(map(lambda node: [
            -((node[1] * msg.info.resolution) + msg.info.origin.position.y),
            -((node[0] * msg.info.resolution) + msg.info.origin.position.x)], self.nodes))
        self.nodes_point = list(map(lambda point: [
            point[0]*math.cos(pose[2]) - point[1]*math.sin(pose[2]) + pose[0],
            point[0]*math.sin(pose[2]) + point[1]*math.cos(pose[2]) + pose[1]], nodes_point_local))

        # Analysis graph
        self.num_nodes_neighbor = self.count_neighbors(self.nodes, self.edges)
        self.root = self.search_closest_edge(self.nodes_point, self.edges, pose)

        # Visualize graph
        self.init_colors()
        self.nodes_marker, self.edges_marker = self.generate_markers(
            self.nodes_point, self.num_nodes_neighbor, self.edges, msg)

    def init_colors(self):
        self.COLOR_NODE = ColorRGBA(r=0.53, g=0.9, b=0.5, a=0.2)        # Green
        self.COLOR_EDGE = ColorRGBA(r=0.53, g=0.9, b=0.5, a=0.2)        # Green
        self.COLOR_INTERSECTION = ColorRGBA(r=1., g=0.73, b=0., a=0.2)  # Yellow
        self.COLOR_LEAF = ColorRGBA(r=0.95, g=0.37, b=0.37, a=0.2)      # Red

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

    def count_neighbors(self, nodes, edges):
        """Return number of neighbor nodes"""

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
        nodes_marker.header.frame_id = 'odom'
        nodes_marker.ns = 'node'
        nodes_marker.id = 0
        nodes_marker.type = Marker.POINTS
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = 0.1      # Point width
        nodes_marker.scale.y = 0.1      # Point height
        for i, node_point in enumerate(nodes_point):
            p = Point(
                x = -node_point[0],
                y = -node_point[1],
                z = 0.1)
            nodes_marker.points.append(p)
            if(num_nodes_neighbor[i] == 1):
                nodes_marker.colors.append(copy.deepcopy(self.COLOR_LEAF))
            elif(num_nodes_neighbor[i] > 2):
                nodes_marker.colors.append(copy.deepcopy(self.COLOR_INTERSECTION))
            else:
                nodes_marker.colors.append(copy.deepcopy(self.COLOR_NODE))                

        # Add edge marker
        edges_marker = Marker()
        edges_marker.header = copy.deepcopy(msg.header)
        edges_marker.header.frame_id = 'odom'
        edges_marker.ns = 'edge'
        edges_marker.id = 0
        edges_marker.type = Marker.LINE_LIST
        edges_marker.action = Marker.ADD
        edges_marker.scale.x = 0.03     # Line width
        for edge in edges:
            edges_marker.points.append(nodes_marker.points[edge[0]])
            edges_marker.points.append(nodes_marker.points[edge[1]])
            edges_marker.colors.append(copy.deepcopy(self.COLOR_EDGE))
            edges_marker.colors.append(copy.deepcopy(self.COLOR_EDGE))
        
        return nodes_marker, edges_marker

    def update_node_color_alpha(self, node, alpha):
        self.nodes_marker.colors[node].a = alpha

    def update_edge_color_alpha(self, edge, alpha):
        for i, e in enumerate(self.edges):
            if(e == edge or e[::-1] == edge):
                self.edges_marker.colors[2*i].a = alpha
                self.edges_marker.colors[2*i+1].a = alpha
