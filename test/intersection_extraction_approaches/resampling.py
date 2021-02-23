#!/usr/bin/env python3

import pickle
import time
import heapq
import math
import plotly.graph_objects as go

from collections import deque
from geometry_msgs.msg import Point

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

    # Visit nodes using heap
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

def merge(A, B, resolution, weight):
    """Merge graph B into A"""
    if not A.nodes: return B
    get_distance_2 = lambda a, b: math.sqrt((a.point.x - b.point.x)**2 + (a.point.y - b.point.y)**2)
        
    for b in B.nodes:
        dist = [get_distance_2(b, a) for a in A.nodes]
        d = min(dist)
        i = dist.index(d)

        # Estimate position with near nodes
        if d < resolution:
            len_n = len(b.neighbors)
            len_m = len(A.nodes[i].neighbors)
            if (len_n < 3 and len_m < 3) or (len_n >= 3 and len_m >= 3):
                A.nodes[i].point.x = weight*A.nodes[i].point.x + (1-weight)*b.point.x
                A.nodes[i].point.y = weight*A.nodes[i].point.y + (1-weight)*b.point.y
        # Add far nodes
        else:
            m = A.add_node(Point(x=b.point.x, y=b.point.y))
            A.nodes[i].add_neighbor(m)

    return A

if __name__=='__main__':
    with open('test/data/L8401-L8454.pkl', 'rb') as f:
        data = pickle.load(f)
    queue = deque(maxlen=5)
    frames, steps = [], []
    topology = EmptyGraph()

    # Set parameters
    resolution = 1.
    weight = 0.98
    
    # Span local topology over time
    start = time.time()
    for idx, graph in enumerate(data):
        queue.append(graph)

        # Resample local topology
        g = create_graph(points=graph['nodes'], edges=list(set(graph['edges'])))
        g = resample(g, graph['pose'], 1.)

        # Merge global and local topologies
        topology = merge(topology, g, resolution, weight)

        # Create frame and step
        topology_local_x, topology_local_y = [], []
        for q in queue:
            for e0, e1 in q['edges']:
                topology_local_x.extend([q['nodes'][e0][0], q['nodes'][e1][0], None])
                topology_local_y.extend([q['nodes'][e0][1], q['nodes'][e1][1], None])
        topology_local_trace = go.Scatter(
            name='Local topology',
            x=topology_local_x, y=topology_local_y,
            mode='lines+markers')

        topology_global_x, topology_global_y = [], []
        for node in topology.nodes:
            for neighbor in node.neighbors:
                topology_global_x.extend([node.point.x, neighbor.point.x, None])
                topology_global_y.extend([node.point.y, neighbor.point.y, None])
        topology_global_trace = go.Scatter(
            name='Global topology',
            x=topology_global_x, y=topology_global_y,
            mode='lines+markers')

        robot_x, robot_y = graph['pose']
        robot_trace = go.Scatter(
            name='Robot position',
            x=[robot_x], y=[robot_y],
            mode='markers', marker=dict(size=20))

        frames.append(go.Frame(
            name=idx,
            data=[topology_local_trace, topology_global_trace, robot_trace]))
        steps.append({
            'args': [[idx], dict(
                frame=dict(duration=0),
                transition=dict(duration=0))],
            'label': idx,
            'method': 'animate'})

    print(f'Time elapsed: {time.time() - start}')

    # Show results
    fig = go.Figure(
        data=frames[0].data, frames=frames)
    fig.update_layout(
        sliders=[dict(steps=steps)],
        xaxis_range=[0, 18], yaxis_range=[-14, 2])
    fig.show()

    print(f'Time elapsed: {time.time() - start}')