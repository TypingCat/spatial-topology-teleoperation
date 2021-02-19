#!/usr/bin/env python3

import pickle
import plotly.graph_objects as go
from collections import defaultdict
from sklearn.cluster import MeanShift
from sklearn.decomposition import PCA
import math
import time

def clustering(data, bandwidth):
    # Intersection clustering
    meanshift = MeanShift(bandwidth=bandwidth)
    meanshift.fit(data)

    # Principal component analysis for each cluster
    points, pca = [], []
    for i in range(meanshift.cluster_centers_.shape[0]):
        points.extend([
            [(inter_pos[0][idx], inter_pos[1][idx]) for idx, label in enumerate(meanshift.labels_) if label == i]])
            
    for i, point in enumerate(points):
        p = PCA(n_components=2)
        p.fit(point)
        pca.append(p)

    return meanshift, pca

def get_inter(graph_list):
    pos_x, pos_y = [], []
    edge_x, edge_y = [], []    

    for graph in graph_list:
        edges = defaultdict(list)
        for e0, e1 in graph['edges']:
            edges[e0].append(e1)
        
        for i in edges:
            if len(set(edges[i])) < 3: continue

            pos_x.append(graph['nodes'][i][0])
            pos_y.append(graph['nodes'][i][1])
            for j in edges[i]:
                x0, y0 = graph['nodes'][i]
                x1, y1 = graph['nodes'][j]
                edge_x.extend([x0, x1, None])
                edge_y.extend([y0, y1, None])

    return [pos_x, pos_y], [edge_x, edge_y]

if __name__=='__main__':
    with open('test/data/L8401-L8454.pkl', 'rb') as f:
        data = pickle.load(f)
    inter_pos, inter_edge = get_inter(data)
    robot_position = [
        [d['pose'][0] for d in data],
        [d['pose'][1] for d in data]]
    resolution = 1.

    # Clustering intersection points
    start = time.time()
    meanshift, pca = clustering(
        [(inter_pos[0][i], inter_pos[1][i])for i in range(len(inter_pos[0]))], resolution)
    print(f'Time elapsed: {time.time() - start}')

    # Show results
    for i, p in enumerate(pca):
        print(f'Cluster {i}')
        print(f'Number of Samples: {p.n_samples_}')
        print(f'Mean: {list(p.mean_)}')
        print(f'Components: {list(p.components_[0])}, {list(p.components_[1])}')
        print(f'Std.: {math.sqrt(p.explained_variance_[0])}, {math.sqrt(p.explained_variance_[1])}')

    fig = go.Figure()
    fig.add_trace(go.Scatter(
        name='Intersection edge',
        x=inter_edge[0],
        y=inter_edge[1],
        mode='lines+markers'))
    for i in range(meanshift.cluster_centers_.shape[0]):
        fig.add_trace(go.Scatter(
            name=f'Intersection position cluster {i}',
            x=[inter_pos[0][idx] for idx, label in enumerate(meanshift.labels_) if label == i],
            y=[inter_pos[1][idx] for idx, label in enumerate(meanshift.labels_) if label == i],
            mode='markers'))
    fig.show()