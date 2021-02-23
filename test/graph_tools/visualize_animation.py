#!/usr/bin/env python3

import pickle
import time
import plotly.graph_objects as go

from collections import deque

if __name__=='__main__':
    with open('test/data/L8401-L8454.pkl', 'rb') as f:
        data = pickle.load(f)
    queue = deque(maxlen=5)
    frames, steps = [], []
    
    # Span local topology over time
    start = time.time()
    for idx, graph in enumerate(data):
        queue.append(graph)

        # ...

        # Create frame and step
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

        frames.append(go.Frame(
            name=idx,
            data=[edges_trace, robot_trace]))
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