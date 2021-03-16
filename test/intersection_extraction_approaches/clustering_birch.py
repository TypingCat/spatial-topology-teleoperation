#!/usr/bin/env python3

import pickle
import time
import plotly.graph_objects as go
import math

from collections import deque, defaultdict
from sklearn.cluster import Birch, MeanShift
from sklearn.decomposition import PCA

from waffle_topology.visualization import create_frame, show_frames
from waffle_topology.calculation import get_ellipse_contour

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

def store_intersection(label, intersection_pos, intersection_edge, cluster_pos, cluster_leg):
    """Store intersections in clusters by label"""
    # cluster_pos: Position of the intersection
    # cluster_leg: Unit vector of the intersection legs

    for i, l in enumerate(label):
        cluster_pos[l].appendleft(intersection_pos[i])
        leg = [math.atan2(neighbor[1] - intersection_pos[i][1], neighbor[0] - intersection_pos[i][0]) for neighbor in intersection_edge[i] if neighbor[0]!=intersection_pos[i][0] or neighbor[1] - intersection_pos[i][1]]
        cluster_leg[l].extendleft([(math.cos(l), math.sin(l)) for l in leg])

        if len(cluster_pos[l]) > cluster_maxlen:
            cluster_pos[l].pop()
        while len(cluster_leg[l]) > cluster_maxlen:
            cluster_leg[l].pop()

if __name__=='__main__':
    with open('test/data/L8401-L8454.pkl', 'rb') as f:
        data = pickle.load(f)

    # Initialize values
    graphs = deque(maxlen=5)
    cluster_pos, cluster_leg, cluster_info = defaultdict(deque), defaultdict(deque), defaultdict(dict)
    cluster_maxlen = 500
    meanshift = MeanShift(  # Clustering for intersection leg
        bandwidth=0.3*math.sqrt(2))
    birch = Birch(          # Clustering for intersection position
        threshold=1.,
        n_clusters=None,
        branching_factor=cluster_maxlen)
    pca = PCA(n_components=2)
    frames = []
    
    # Span local topology over time
    start = time.time()
    for idx, graph in enumerate(data['topology']):
        graphs.append(graph)

        # Get intersection
        intersection_pos, intersection_edge, pos = [], [], []
        for i, q in enumerate(graphs):
            pos, edge = get_intersection(q)
            intersection_pos.extend(pos)
            intersection_edge.extend(edge)

        # Clustering intersections
        if pos:
            birch.partial_fit(pos)
        if intersection_pos:
            label = birch.predict(intersection_pos)
            store_intersection(label, intersection_pos, intersection_edge, cluster_pos, cluster_leg)

        # Analysis clusters
        for k, v in cluster_pos.items():
            if len(v) < 3: continue

            # Get principal component of intersection position
            p = pca.fit(v)
            cluster_info[k]['mean'] = p.mean_
            cluster_info[k]['components'] = p.components_
            cluster_info[k]['std'] = [math.sqrt(v) for v in p.explained_variance_]

            # # Clustering intersection legs
            # m = meanshift.fit(cluster_leg[k])
            # # cluster_leg_cluster = defaultdict(list)
            # angles = [[math.atan2(cluster_leg[k][j][1], cluster_leg[k][j][0]) for j, l in enumerate(m.labels_) if l==i] for i in range(m.labels_.max())]

        # Create frame
        map_trace = go.Scatter(
            name='Map',
            x=data['map'][0], y=data['map'][1],
            mode='lines')

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
        for k, info in cluster_info.items():
            ex, ey = get_ellipse_contour(   # Represent cluster PCA results with ellipse
                center=info['mean'],
                axis=info['components'],
                size=info['std'])
            intersection_trace.append(go.Scatter(
                name=f'Intersection {k}',
                x=ex, y=ey,
                mode='lines', fill='toself'))

        frames.append(create_frame(
            [map_trace, topology_local_trace, robot_trace] + intersection_trace, idx))

    # Show results
    print(f'Time elapsed: {time.time() - start}')
    show_frames(frames)
    print(f'Time elapsed: {time.time() - start}')
