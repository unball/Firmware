import plotly
import plotly.graph_objs as go
import numpy as np


y = []
x = 

trace0 = go.Scatter(y=array[0], x=array[3], name='input')
trace1 = go.Scatter(y=array[1], x=array[3], name='Motor A', mode = 'lines+markers')
trace2 = go.Scatter(y=array[2], x=array[3], name='Motor B', mode = 'lines+markers')

trace3 = go.Scatter(y=func(xAxis, optA, x=xAxis), name='regression A', mode = 'lines')
trace4 = go.Scatter(y=func(xAxis, optB, x=xAxis), name='regression B', mode = 'lines')

graph = [trace0, trace1, trace2, trace3, trace4]
layout = dict(title = 'Field',
            xaxis = dict(title = 'X axis'),
            yaxis = dict(title = 'Y axis'),
)
fig = dict(data=graph, layout=layout)
plotly.offline.plot(fig, filename='Motor_graph.html')