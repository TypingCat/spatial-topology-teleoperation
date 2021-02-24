#!/usr/bin/env python3

import plotly.graph_objects as go

def create_frame(traces, idx):
    frame = go.Frame(name=idx, data=traces)
    step = dict(
        args=[[idx], dict(
            frame={'duration': 0, 'redraw': True},
            transition={'duration': 0})],
        label=idx,
        method='animate')
    
    return [frame, step]

def show(frames, hz=50):
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