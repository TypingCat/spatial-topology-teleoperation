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

def show(frames):
    """Visualize frames with play button and slider"""
    frame, step = [f[0] for f in frames], [f[1] for f in frames]

    # Create buttons
    fig_button_play = dict(
        args=[None, dict(
            frame={'duration': 20, 'redraw': True},
            mode='immediate',
            transition={'duration': 0})],
        label='Play',
        method='animate')
    fig_button_stop = dict(
        args=[[None], dict(
            frame={'duration': 20, 'redraw': False},
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
        xaxis_range=[0, 18], yaxis_range=[-14, 2])
    fig.show()