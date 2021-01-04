#!/usr/bin/env python3

import cv2
import numpy as np
import math

class Node():
    def __init__(self, id, point):
        self.id = id
        self.point = point
        self.neighbors = []

class Graph():
    """Topology graph from a gridmap topology image"""
    seq = -1

    def __init__(self, msg, pose):
        Graph.seq += 1

        # Extract features
        img = self.convert_map_to_image(msg)
        topology = cv2.ximgproc.thinning(img)
        contours, _ = cv2.findContours(topology, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corners = cv2.goodFeaturesToTrack(topology, 20, 0.1, 6)

        # Search nodes and edges
        nodes_idx = list(map(lambda corner: [
            int(corner[0][0]), int(corner[0][1])], corners))
        edges_idx = self.search_edges(
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
        for i, point in enumerate(nodes_point_global):
            self.nodes.append(Node([Graph.seq, i], point))
        for edge in edges_idx:
            self.nodes[edge[0]].neighbors.append(self.nodes[edge[1]])

        # Analysis graph
        self.root = self.search_closest_edge(pose)

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

    def search_closest_edge(self, pose):
        """Return closest edge[departure node, destination node] from pose"""

        # Search closest node
        nodes_dist = []
        for node in self.nodes:
            nodes_dist.append((node.point[0] - pose[0])**2 + (node.point[1] - pose[1])**2)
        node_departure = self.nodes[nodes_dist.index(min(nodes_dist))]

        # Search closest edges in angle
        edges_abs_angle = []
        for node in node_departure.neighbors:
            angle = math.atan2(
                node.point[1] - node_departure.point[1],
                node.point[0] - node_departure.point[0]
            ) - pose[2]
            edges_abs_angle.append(abs((angle + 3*math.pi) % (2*math.pi) - math.pi))
        
        return [node_departure, self.nodes[edges_abs_angle.index(min(edges_abs_angle))]]