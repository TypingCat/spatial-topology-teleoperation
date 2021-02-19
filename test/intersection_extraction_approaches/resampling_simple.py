#!/usr/bin/env python3

from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
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

def draw_graph(graph):
    plt.scatter(
        [node.point.x for node in graph.nodes],
        [node.point.y for node in graph.nodes])
    for node in graph.nodes:
        for neighbor in node.neighbors:
            plt.plot(
                [node.point.x, neighbor.point.x],
                [node.point.y, neighbor.point.y])

def resample(graph, root, resolution):
    """Resample graph using spanning tree and priority queue"""
    g = create_graph(points=[(root.point.x, root.point.y)])
    root.parent = graph.nodes[0]
    root.resample = [g.nodes[0], 0]
    heap = [(0., root)]
    visit = []
    get_distance = lambda a, b: math.sqrt((a.point.x - b.point.x)**2 + (a.point.y - b.point.y)**2)

    while heap:
        dist_accum, node = heapq.heappop(heap)
        if node not in visit:
            visit.append(node)

            for neighbor in node.neighbors:
                if neighbor == node.parent: continue
                heapq.heappush(heap, (dist_accum + get_distance(node, neighbor), neighbor))
                neighbor.parent = node
            sample, dist_outermost = node.parent.resample
            dist_segment = get_distance(node, node.parent)
            dist = dist_outermost + dist_segment

            while dist >= resolution:
                ratio = [(dist-resolution)/dist, resolution/dist]
                s = g.add_node(Point(
                    x=ratio[0]*sample.point.x + ratio[1]*node.point.x,
                    y=ratio[0]*sample.point.y + ratio[1]*node.point.y))
                sample.add_neighbor(s)
                sample = s
                dist -= resolution
            node.resample = [sample, dist]

    return g

if __name__ == '__main__':
    graph = create_graph(
        points=[(-0.2, 4.2), (0.2, 3.8), (0.5, 3.9), (2.2, 1.5), (1.7, 4.5), (2.2, 4.6), (2.6, 4.7), (3.1, 4.8), (-1.7, 3.0), (-2.4, 2.1), (-0.3, 4.6), (-0.6, 5.3), (-0.5, 6.1), (-0.9, 5.3), (-1.7, 7.3), (-1.8, 7.6)],
        edges=[(0, 1), (1, 2), (1, 3), (2, 4), (4, 5), (5, 6), (6, 7), (0, 8), (8, 9), (0, 10), (10, 11), (11, 12), (11, 13), (13, 14), (14, 15)])

    # Test resample function
    start = time.time()
    g_03 = resample(graph, graph.nodes[0], 0.3)
    print(f'Time elapsed: {time.time() - start:.4f}sec')
    g_10 = resample(graph, graph.nodes[0], 1.0)
    print(f'Time elapsed: {time.time() - start:.4f}sec')
    
    # Visualize results
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 3, 1)
    plt.title('raw')
    draw_graph(graph)
    plt.subplot(1, 3, 2)
    plt.title('resolution=0.3')
    draw_graph(g_03)
    plt.subplot(1, 3, 3)
    plt.title('resolution=1.0')
    draw_graph(g_10)
    plt.show()
