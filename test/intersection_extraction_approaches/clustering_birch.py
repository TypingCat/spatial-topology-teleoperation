#!/usr/bin/env python3

import pickle
import time
import plotly.graph_objects as go
import numpy as np

from collections import deque, defaultdict
from sklearn.cluster import Birch
from sklearn.decomposition import PCA

from waffle_topology.visualization import create_frame, show, get_ellipse_contour

def get_intersection(graph):
    """Return intersection nodes and its neighbors position"""
    intersection_pos, intersection_edge = [], []
    edges = defaultdict(list)

    for e0, e1 in graph['edges']:
        edges[e0].append(e1)    
    for i in edges:
        if len(set(edges[i])) < 3: continue
        intersection_pos.append(graph['nodes'][i])
        intersection_edge.append([graph['nodes'][j] for j in edges[i]])

    return intersection_pos, intersection_edge

if __name__=='__main__':
    with open('test/data/L8401-L8454.pkl', 'rb') as f:
        data = pickle.load(f)

    # Initialize values
    graphs = deque(maxlen=5)
    clusters = defaultdict(deque)
    clusters_maxlen = 300
    birch = Birch(
        threshold=1.,
        n_clusters=None,
        branching_factor=clusters_maxlen)
    frames = []
    
    # Span local topology over time
    start = time.time()
    for idx, graph in enumerate(data):
        graphs.append(graph)

        # Get intersection
        intersection_pos, intersection_edge, pos = [], [], []
        for i, q in enumerate(graphs):
            pos, edge = get_intersection(q)
            intersection_pos.extend(pos)
            intersection_edge.extend(edge)

        # Cluster intersections
        if pos: birch.partial_fit(pos)
        if intersection_pos:
            label = birch.predict(intersection_pos)
            for i, label in enumerate(birch.predict(intersection_pos)):
                clusters[label].appendleft(intersection_pos[i])
                if len(clusters[label]) > clusters_maxlen: clusters[label].pop()

        # Analysis clusters
        




        # Create frame
        topology_local_x, topology_local_y = [], []
        for q in graphs:
            for e0, e1 in q['edges']:
                topology_local_x.extend([q['nodes'][e0][0], q['nodes'][e1][0], None])
                topology_local_y.extend([q['nodes'][e0][1], q['nodes'][e1][1], None])
        topology_local_trace = go.Scatter(
            name='Local topology',
            x=topology_local_x, y=topology_local_y,
            mode='lines+markers')

        robot_x, robot_y = graph['pose']
        robot_trace = go.Scatter(
            name='Robot position',
            x=[robot_x], y=[robot_y],
            mode='markers', marker=dict(size=20))

        intersection_trace = []
        for k, v in clusters.items():
            if len(v) < 3: continue

            # Represent cluster PCA results with ellipse
            pca = PCA(n_components=2)
            p = pca.fit(v)
            ex, ey = get_ellipse_contour(
                center=p.mean_,
                axis=p.components_,
                size=[np.sqrt(v) for v in p.explained_variance_])
            intersection_trace.append(go.Scatter(
                name=f'Intersection {k}',
                x=ex, y=ey,
                mode='lines', fill='toself'))

        frames.append(create_frame(
            [topology_local_trace, robot_trace] + intersection_trace, idx))

    # Show results
    print(f'Time elapsed: {time.time() - start}')
    show(frames)
    print(f'Time elapsed: {time.time() - start}')
