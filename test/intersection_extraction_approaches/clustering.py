#!/usr/bin/env python3

import pickle
import time
import plotly.graph_objects as go

from collections import deque
from collections import defaultdict
from sklearn.cluster import MeanShift
from sklearn.decomposition import PCA

def clustering(points, bandwidth):
    if not points: return [], [], []
    points_cluster, pca = [], []

    # Intersection clustering
    meanshift = MeanShift(bandwidth=bandwidth)
    meanshift.fit(points)

    # Principal component analysis for each cluster
    for idx in range(meanshift.cluster_centers_.shape[0]):
        points_cluster.extend([
            [points[i] for i, label in enumerate(meanshift.labels_) if label == idx]])
    # for point in points_cluster:
    #     p = PCA(n_components=2)
    #     p.fit(point)
    #     pca.append(p)

    return meanshift, pca, points_cluster

def get_intersection(graph):
    """Return intersection nodes and its neighbors"""
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
    queue = deque(maxlen=5)
    resolution = 1.
    frames, steps = [], []
    
    # Span local topology over time
    start = time.time()
    for idx, graph in enumerate(data):
        queue.append(graph)

        # Get intersection
        intersection_pos, intersection_edge = [], []
        for q in queue:
            pos, edge = get_intersection(q)
            if pos:
                intersection_pos.extend(pos)
                intersection_edge.extend(edge)

        # Intersection Clustering
        meanshift, pca, intersection_pos_cluster = clustering(intersection_pos, resolution)

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

        robot_x, robot_y = graph['pose']
        robot_trace = go.Scatter(
            name='Robot position',
            x=[robot_x], y=[robot_y],
            mode='markers', marker=dict(size=20))

        frame_data = []
        for i, pos_cluster in enumerate(intersection_pos_cluster):
            frame_data.append(go.Scatter(
                name=f'Intersection {i}',
                x=[pos[0] for pos in pos_cluster], y=[pos[1] for pos in pos_cluster],
                mode='markers'))

        frames.append(go.Frame(
            name=idx,
            data=[topology_local_trace, robot_trace] + frame_data))
        steps.append({
            'args': [[idx], dict(
                frame=dict(duration=0),
                transition=dict(duration=0))],
            'label': idx,
            'method': 'animate'})

    print(f'Time elapsed: {time.time() - start}')

    # Fill up the frame blanks
    trace_num = max([len(frame.data) for frame in frames])
    blank = [go.Scatter(name='Blank', x=[], y=[])]
    for frame in frames:
        frame.data = list(frame.data) + blank*(trace_num - len(frame.data))

    # Show results
    fig = go.Figure(
        data=frames[0].data, frames=frames)
    fig.update_layout(
        sliders=[dict(steps=steps)],
        xaxis_range=[0, 18], yaxis_range=[-14, 2])
    fig.show()

    print(f'Time elapsed: {time.time() - start}')