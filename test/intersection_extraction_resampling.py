#!/usr/bin/env python3

import pickle
import plotly.graph_objects as go
from geometry_msgs.msg import Point
import heapq
import math
import time

from waffle_topology.graph import EmptyGraph

def create_graph(points, edges=[]):
    graph = EmptyGraph()

    for point in points:
        graph.add_node(Point(x=float(point[0]), y=float(point[1])))
    for edge in edges:
        graph.add_neighbor(edge)

    return graph

def resample(graph, robot_position, resolution):
    """Resample graph using spanning tree and priority queue"""
    get_distance = lambda a, b: math.sqrt((a.point.x - b.point.x)**2 + (a.point.y - b.point.y)**2)

    # Initialize root node
    dist = [(node.point.x - robot_position[0])**2 + (node.point.y - robot_position[1])**2 for node in graph.nodes]
    root = graph.nodes[dist.index(min(dist))]
    g = create_graph(points=[(root.point.x, root.point.y)])
    root.parent = root
    root.resample = [g.nodes[0], 0]

    # Initialize heap
    heap = [(0., 0, root)]
    heap_cnt = 1
    visit = []

    while heap:
        dist_accum, _, node = heapq.heappop(heap)
        if node not in visit:
            visit.append(node)

            # Add neighbor into heap
            for neighbor in node.neighbors:
                if neighbor == node.parent: continue
                heapq.heappush(heap,
                    (dist_accum + get_distance(node, neighbor), heap_cnt, neighbor))
                heap_cnt += 1
                neighbor.parent = node

            # Get distance from outmost resampled node
            sample, dist_outermost = node.parent.resample
            dist_segment = get_distance(node, node.parent)
            dist = dist_outermost + dist_segment

            # Resampling
            while dist >= resolution:
                ratio = [(dist-resolution)/dist, resolution/dist]
                s = g.add_node(Point(
                    x=ratio[0]*sample.point.x + ratio[1]*node.point.x,
                    y=ratio[0]*sample.point.y + ratio[1]*node.point.y))
                sample.add_neighbor(s)
                s.add_neighbor(sample)
                sample = s
                dist -= resolution
            node.resample = [sample, dist]

    return g

if __name__=='__main__':
    with open('test/intersection_extraction.pkl', 'rb') as f:
        data = pickle.load(f)
    edge_x, edge_y = [], []
    edge_resample_x, edge_resample_y = [], []
    edge_global_x, edge_global_y = [], []
    topology = EmptyGraph()
    get_distance_2 = lambda a, b: math.sqrt((a.point.x - b.point.x)**2 + (a.point.y - b.point.y)**2)

    # Set parameters
    resolution = 1.
    weight = 0.98

    # Global topology
    start = time.time()    
    for count, graph in enumerate(data):
        start = time.time()

        # Resample local topology
        g = create_graph(points=graph['nodes'], edges=list(set(graph['edges'])))
        g = resample(g, graph['pose'], 1.)

        # Merge graphs
        if not topology.nodes:
            topology.nodes = g.nodes
        else:
            for n in g.nodes:
                dist = [get_distance_2(n, node) for node in topology.nodes]
                d = min(dist)
                i = dist.index(d)

                # Estimate position with near nodes
                if d < resolution:
                    len_n = len(n.neighbors)
                    len_m = len(topology.nodes[i].neighbors)
                    if (len_n < 3 and len_m < 3) or (len_n >= 3 and len_m >= 3):
                        topology.nodes[i].point.x = weight*topology.nodes[i].point.x + (1-weight)*n.point.x
                        topology.nodes[i].point.y = weight*topology.nodes[i].point.y + (1-weight)*n.point.y
                # Add far nodes
                else:
                    m = topology.add_node(Point(x=n.point.x, y=n.point.y))
                    topology.nodes[i].add_neighbor(m)

        print(f'{count} Time elapsed: {time.time() - start}')

        # Save resampled local topology edges
        for node in g.nodes:
            for neighbor in node.neighbors:
                edge_resample_x.extend([node.point.x, neighbor.point.x, None])
                edge_resample_y.extend([node.point.y, neighbor.point.y, None])

    # Save local topology edges
    for graph in data:
        for e0, e1 in graph['edges']:
            x0, y0 = graph['nodes'][e0]
            x1, y1 = graph['nodes'][e1]
            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])

    # Save global topology edges
    for node in topology.nodes:
        for neighbor in node.neighbors:
            edge_global_x.extend([node.point.x, neighbor.point.x, None])
            edge_global_y.extend([node.point.y, neighbor.point.y, None])

    # Show results
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        name='Resampled Local topology',
        x=edge_resample_x,
        y=edge_resample_y,
        mode='lines+markers'))
    fig.add_trace(go.Scatter(
        name='Global topology',
        x=edge_global_x,
        y=edge_global_y,
        mode='lines+markers'))
    fig.show()