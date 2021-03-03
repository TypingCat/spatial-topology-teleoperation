#!/usr/bin/env python3

import pickle
import time
import plotly.graph_objects as go

from collections import deque
from waffle_topology.visualization import create_frame, show

if __name__=='__main__':
    with open('test/data/L8401-L8454.pkl', 'rb') as f:
        data = pickle.load(f)
    queue = deque(maxlen=5)
    frames = []
    
    # Span local topology over time
    start = time.time()
    for idx, graph in enumerate(data['topology']):
        queue.append(graph)

        # ...

        # Create frame
        map_trace = go.Scatter(
            name='Map',
            x=data['map'][0], y=data['map'][1],
            mode='lines')
        
        edges_x, edges_y = [], []
        for q in queue:
            for e0, e1 in q['edges']:
                edges_x.extend([q['nodes'][e0][0], q['nodes'][e1][0], None])
                edges_y.extend([q['nodes'][e0][1], q['nodes'][e1][1], None])
        edges_trace = go.Scatter(
            name='Local topology',
            x=edges_x, y=edges_y,
            mode='lines+markers')

        robot_x, robot_y = graph['pose']
        robot_trace = go.Scatter(
            name='Robot position',
            x=[robot_x], y=[robot_y],
            mode='markers', marker=dict(size=20))

        frames.append(create_frame([map_trace, edges_trace, robot_trace], idx))

    # Show results
    print(f'Time elapsed: {time.time() - start}')
    show(frames)
    print(f'Time elapsed: {time.time() - start}')