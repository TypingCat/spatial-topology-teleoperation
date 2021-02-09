#!/usr/bin/env python3

import pickle
# import plotly.express as px
import plotly.graph_objects as go

def get_edge(graph_list):
    edge_x, edge_y = [], []

    for graph in graph_list:
        for e0, e1 in graph['edges']:
            x0, y0 = graph['nodes'][e0]
            x1, y1 = graph['nodes'][e1]
            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])

    return [edge_x, edge_y]

if __name__=='__main__':
    with open('test/intersection_extraction.pkl', 'rb') as f:
        data = pickle.load(f)
    local_topology = get_edge(data)
    robot_position = [
        [d['pose'][0] for d in data],
        [d['pose'][1] for d in data]]

    fig = go.Figure()
    fig.add_trace(go.Scatter(
        name='Local topology',
        x=local_topology[0],
        y=local_topology[1],
        mode='lines+markers'
    ))
    fig.add_trace(go.Scatter(
        name='Robot position',
        x=robot_position[0],
        y=robot_position[1],
        mode='lines+markers'
    ))
    fig.show()