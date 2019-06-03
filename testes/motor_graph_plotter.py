import xlsxwriter
import serial
import plotly
import plotly.graph_objs as go
import numpy as np
from scipy.optimize import curve_fit


#for x in range(1,3):
#	t = input('tensao: ')
#	print x
#	worksheet.write(x, 0, t)
#	ea = input("erro A: ")
#	worksheet.write(x, 1, ea)
#	eb = input("erro B: ")
#	worksheet.write(x, 2, eb)

def func(x, k, t):
    return 100*k*(1- np.exp(-x/t))

def ler_serial():
    alpha = 1000
    array = [[i*0 for i in range(alpha)],[i*0 for i in range(alpha)],[i*0 for i in range(alpha)],[i*0 for i in range(alpha)]]
    x = 1
    y = 0
    c = 0
    poptArrayA = np.array([])
    poptArrayB = np.array([])
    while(True):
        data = arduino.readline()[:-2]
        if data == "$" and x < (alpha+1):
            y=0

            data = arduino.readline()[:-2]
            array[y][x-1] = data
            #print(data)
            #worksheet.write(x, y, data)
            y+=1

            data = arduino.readline()[:-2]
            array[y][x-1] = data
            #print(data)
            #worksheet.write(x, y, data)
            y+=1

            data = arduino.readline()[:-2]
            array[y][x-1] = data
            #print(data)
            #worksheet.write(x, y, data)
            y+=1

            data = arduino.readline()[:-2]
            array[y][x-1] = data
            #print(data)
            #worksheet.write(x, y, data)
            x+=1
        elif data == "#" or x >= alpha:
            c+=1
            x = 0
            xAxis = np.asarray(array[3], dtype=float)
            yAxisA = np.asarray(array[1], dtype=float)
            yAxisB = np.asarray(array[2], dtype=float)

            popt, _ = curve_fit(func, xAxis, yAxisA)
            poptArrayA = np.append(poptArrayA, popt)
            popt, _ = curve_fit(func, xAxis, yAxisB)
            poptArrayB = np.append(poptArrayB, popt)

            x[:argmin]

            if c == 10:
                print("TERMINOU AQUI")
                trace0 = go.Scatter(y=array[0], x=array[3], name='input')
                trace1 = go.Scatter(y=array[1], x=array[3], name='Motor A', mode = 'lines+markers')
                trace2 = go.Scatter(y=array[2], x=array[3], name='Motor B', mode = 'lines+markers')
                optA = (((2*poptArrayA)/poptArrayA.size).sum(0))
                optB = (((2*poptArrayB)/poptArrayB.size).sum(0))
                trace3 = go.Scatter(y=func(xAxis, optA, x=xAxis), name='regression A', mode = 'lines')
                trace4 = go.Scatter(y=func(xAxis, optB, x=xAxis), name='regression B', mode = 'lines')

                graph = [trace0, trace1, trace2, trace3, trace4]
                layout = dict(title = 'Motor Identification',
                            xaxis = dict(title = 'Time [ms]'),
                            yaxis = dict(title = 'PWM / Ticks'),
                )
                fig = dict(data=graph, layout=layout)
                plotly.offline.plot(fig, filename='Motor_graph.html')
                break
		
	


#setup
workbook = xlsxwriter.Workbook('Motor_ID.xlsx')
worksheet = workbook.add_worksheet()
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)

#worksheet.write(2, 0, 123)
worksheet.write('A1', 'input')
worksheet.write('B1', 'motor A')
worksheet.write('C1', 'motor B')
worksheet.write('D1', 'Time')
ler_serial()
workbook.close()