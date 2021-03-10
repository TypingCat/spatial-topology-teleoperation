#!/usr/bin/env python3

import copy
import math
import plotly.graph_objects as go

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from waffle_topology.calculation import euler_to_quaternion

class Color:
    NODE = ColorRGBA(r=0.53, g=0.9, b=0.5, a=0.2)        # Green
    EDGE = ColorRGBA(r=0.53, g=0.9, b=0.5, a=0.2)        # Green
    INTERSECTION = ColorRGBA(r=0., g=0.33, b=1., a=0.2)  # Blue
    CLUSTER = ColorRGBA(r=1., g=0.73, b=0., a=0.6)       # Yellow
    LEAF = ColorRGBA(r=0.95, g=0.37, b=0.37, a=0.2)      # Red

    def __init__(self):
        self.node = copy.deepcopy(Color.NODE)
        self.edges = []

def create_frame(traces, idx):
    frame = go.Frame(name=idx, data=traces)
    step = dict(
        args=[[idx], dict(
            frame={'duration': 0, 'redraw': True},
            transition={'duration': 0})],
        label=idx,
        method='animate')
    
    return [frame, step]

def show_frames(frames, hz=50):
    """Visualize frames with play button and slider"""

    # Fill up the frame blanks
    frame, step = [fs[0] for fs in frames], [fs[1] for fs in frames]
    trace_num = max([len(f.data) for f in frame])
    blank = [go.Scatter(name='Blank', x=[], y=[])]
    for f in frame:
        f.data = list(f.data) + blank*(trace_num - len(f.data))

    # Get axis range
    criterion_idx = 0
    range_x, range_y = [float('inf'), -float('inf')], [float('inf'), -float('inf')]
    for f in frame:
        X = [x for x in f.data[criterion_idx].x if x is not None]
        Y = [y for y in f.data[criterion_idx].y if y is not None]
        range_x = [min(X + [range_x[0]]), max(X + [range_x[1]])]
        range_y = [min(Y + [range_y[0]]), max(Y + [range_y[1]])]

    # Create buttons
    fig_button_play = dict(
        args=[None, dict(
            frame={'duration': 1000//hz, 'redraw': True},
            mode='immediate',
            transition={'duration': 0})],
        label='Play',
        method='animate')
    fig_button_stop = dict(
        args=[[None], dict(
            frame={'duration': 1000//hz, 'redraw': False},
            mode='immediate',
            transition={'duration': 0})],
        label='Stop',
        method='animate')

    # Show frames
    fig = go.Figure(
        data=frame[0].data, frames=frame)
    fig.update_layout(
        sliders=[{'steps': step}],
        updatemenus = [{'buttons': [fig_button_play, fig_button_stop], 'type': 'buttons'}],
        xaxis_range=range_x, yaxis_range=range_y)
    fig.show()

def create_graph_marker(nodes, header):
    # Initialize node marker
    nodes_marker = Marker()
    nodes_marker.header = copy.deepcopy(header)
    nodes_marker.header.frame_id = 'odom'
    nodes_marker.ns = 'node'
    nodes_marker.id = 0
    nodes_marker.type = Marker.POINTS
    nodes_marker.action = Marker.ADD
    nodes_marker.scale.x = 0.1      # Point width
    nodes_marker.scale.y = 0.1      # Point height

    # Initialize edge marker
    edges_marker = Marker()
    edges_marker.header = copy.deepcopy(header)
    edges_marker.header.frame_id = 'odom'
    edges_marker.ns = 'edge'
    edges_marker.id = 0
    edges_marker.type = Marker.LINE_LIST
    edges_marker.action = Marker.ADD
    edges_marker.scale.x = 0.03     # Line width

    # Fill in the rest
    for node in nodes:
        nodes_marker.points.append(Point(x=-node.point.x, y=-node.point.y, z=0.2))
        nodes_marker.colors.append(node.color.node)
        for i, neighbor in enumerate(node.neighbors):
            edges_marker.points.append(Point(x=-node.point.x, y=-node.point.y, z=0.1))
            edges_marker.points.append(Point(x=-neighbor.point.x, y=-neighbor.point.y, z=0.1))
            edges_marker.colors.append(node.color.edges[i][0])
            edges_marker.colors.append(node.color.edges[i][1])

    return nodes_marker, edges_marker

def create_intersection_marker(label, center, axis, size, header):
    qx, qy, qz, qw = euler_to_quaternion(math.atan2(axis[0][1], axis[0][0]), 0., 0.)
    marker = Marker()

    marker.header = copy.deepcopy(header)
    marker.header.frame_id = 'odom'
    marker.ns = 'intersection'
    marker.id = int(label)
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.orientation.x = qx
    marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz
    marker.pose.orientation.w = qw
    marker.scale.x = size[0]    # Diameter x
    marker.scale.y = size[1]    # Diameter y
    marker.scale.z = 1.         # Height
    marker.color = Color.CLUSTER

    return marker