# import pyqtgraph.examples
# pyqtgraph.examples.run()

# !/usr/bin/python
# -*- coding: utf-8 -*-
"""
Update a simp
le plot as rapidly as possible to measure speed.
"""


from PyQt5 import QtGui, QtCore  # (the example applies equally well to PySide)

import pyqtgraph as pg
import numpy as np
from rplidar import RPLidar
lidar = RPLidar('COM3')
info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

iterator = lidar.iter_scans(max_buf_meas=2000)

app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.resize(800,800)
view = pg.GraphicsLayoutWidget()  ## GraphicsView with GraphicsLayout inserted by default
mw.setCentralWidget(view)
mw.setWindowTitle('pyqtgraph example: ScatterPlot')
plot = view.addPlot(row=1, col=1, rowspan=2, colspan=1)
plot1 = view.addPlot(row=1, col=2, rowspan=1, colspan=1)
plot2 = view.addPlot(row=2, col=2, rowspan=1, colspan=1)
# plot = pg.plot()

plot.setAspectLocked()
#plot1.setAspectLocked(lock=True, ratio=1)
#plot2.setAspectLocked(lock=True, ratio=1)
plot1.setXRange(0, 360, padding=0)
plot2.setXRange(0, 360, padding=0)
plot2.setYRange(0, 40, padding=0)
plot1.setYRange(0, 6000, padding=0)
# Add polar grid lines
plot.addLine(x=0, pen=0.2)
plot.addLine(y=0, pen=0.2)

for r in range(1000, 10001, 1000):
    circle = pg.QtGui.QGraphicsEllipseItem(-r, -r, r*2, r*2)
    circle.setPen(pg.mkPen(0.2))
    plot.addItem(circle)

curve = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None))
curve1 = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 100, 255))
curve2 = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None), brush=pg.mkBrush(255, 100, 255, 255))
#curve1 = pg.PlotDataItem(size=5, pen=pg.mkPen('r'), fillBrush=pg.mkBrush(255, 255, 100, 255))
#curve2 = pg.PlotDataItem(size=5, pen=pg.mkPen('g'), fillBrush=pg.mkBrush(255, 100, 255, 255))

plot1.addItem(curve1)
plot2.addItem(curve2)
plot.addItem(curve)
mw.show()
marker = [0,0]

def update():
    global iterator, lidar, timer, marker
    max_quality = 30
    error_rate = 3
    simple_point = 3
    damping = 200

    try:
        # make polar data
        scan = next(iterator)
        angleDegree = np.array(scan[1])#np.linspace(0, 360, 1000) #numpy array
        distance = np.array(scan[2])#np.random.normal(loc=10, size=1000) #numpy array
        quality = np.array(scan[0])#np.random.normal(loc=10, size=1000) #numpy array
        #print("got data")
        # angleDegree = np.linspace(0, 360, 1000) #numpy array
        # distance = np.random.normal(loc=10, size=1000) #numpy array
        # quality = np.random.normal(loc=10, size=1000) #numpy array

        # Transform to cartesian and plot
        x = distance * np.cos(angleDegree*(np.pi/180))
        y = distance * np.sin(angleDegree*(np.pi/180))

        #set brush colors for quality points
        brushes = []
        for i in range(len(x)):
            brushes.append(pg.intColor(quality[i], 100))


        #update data
        curve.setData(x, y)  # polar
        curve.setBrush(brushes)  # set point color according to quality number.

        curve1.setData(angleDegree, distance) #rect, theta in degrees, distance
        curve2.setData(angleDegree, quality) #rect, theta in degrees, quality

        #apply quality filter
        quality_filter = []
        for j in range(len(x)):
            if(quality[j]> max_quality - distance[j] / damping):
                quality_filter.append([angleDegree[j],distance[j],quality[j]])

        #apply neighbor points filter
        neighbor_filter = []
        error = 0
        for k in quality_filter:
            cood_x = k[1] * np.cos(k[0]*(np.pi/180))
            cood_y = k[1] * np.sin(k[0]*(np.pi/180))
            if(len(neighbor_filter) == 0):
                neighbor_filter.append([cood_x,cood_y])
            elif(abs(neighbor_filter[-1][0] - cood_x) <= (50 + k[1] / damping * 10) and abs(neighbor_filter[-1][1] - cood_y <= (50 + k[1] / damping * 10))):
                neighbor_filter.append([cood_x,cood_y])
            else:
                error += 1
            if(error > error_rate):
                neighbor_filter = []
                error = 0

        #output valid marker
        sum_x = 0
        sum_y = 0
        if(len(neighbor_filter) >= simple_point):
            for i in neighbor_filter:
                sum_x += i[0]
                sum_y += i[1]
            avg_x = sum_x / (len(neighbor_filter))
            avg_y = sum_y / (len(neighbor_filter))
            rho = np.sqrt(avg_x**2 + avg_y**2)
            phi = np.arctan2(avg_y, avg_x)
            marker = [int(avg_x),int(avg_y)] 
            print(marker)
        else:
            print(marker)

        app.processEvents()  ## force complete redraw for every plot
    except KeyboardInterrupt:
        timer.stop()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10) #update frequency
app.exec_()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()