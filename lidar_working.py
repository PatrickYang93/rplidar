from rplidar import RPLidar
from pylab import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time
import math

PORT_NAME = 'COM3'
DMAX = 500
IMIN = 0
IMAX = 50
error_rate = 3
simple_point = 3
mark_inter = 100
damping = 200
ann_list = []
scan_count = 0
scan_list = []
def update_line(num, iterator, line):
    global scan_count, scan_list
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    #print(len(scan))
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)

    if(scan_count == 0):
        scan_list.append(scan)
        scan_count += 1
    elif(scan_count == 1):
        scan_count += 1
        scan_list.append(scan)
    elif(scan_count == 2):
        scan_list.append(scan)
        scan_count = 0
        scan_list = np.append(scan_list[0],[scan_list[1],scan_list[2]])
        print(len(scan_list))
        for i, a in enumerate(ann_list):
            a.remove()

        ann_list[:] = []
        sum_x = 0
        sum_y = 0
        sum_distance = 0
        sum_angle = 0
        scan_list = []
        mark_list = []
        error = 0

        for k in range(len(scan)):
            #j = k // 3 * 3
            j = k
            angle = scan[j][1]*np.pi/180
            distance = scan[j][2]
            q = scan[j][0]

            if(q > 35 - distance / damping):
                scan_list.append([q,angle,distance])
                ann = plt.annotate(q, xy = (angle,distance), color = "purple", fontsize = 9)
                ann_list.append(ann)

        #for i in scan_list:
            #sum_x += i[2] * np.cos(i[1])
            #sum_y += i[2] * np.sin(i[1])
            #sum_angle += i[1]
            #sum_distance += i[2]

        #if(len(marker_list) > 10):
            #cood_x = sum_x / (len(marker_list))
            #cood_y = sum_y / (len(marker_list))
            #angle = sum_angle / (len(marker_list))
            #distance = sum_distance / (len(marker_list))
            #print("average",angle, distance)
            #ann = plt.annotate((int(angle), int(distance)), (angle,distance), color = "purple", fontsize = 9)
            #ann_list.append(ann)
            #print('marker_x = ' , angle)
            #print('marker_y = ' , distance)

        for i in scan_list:
            x = i[2] * np.cos(i[1])
            y = i[2] * np.sin(i[1])
            if(len(mark_list) == 0):
                mark_list.append([x,y])
            elif(abs(mark_list[-1][0] - x) <= mark_inter + i[2] / damping *5 and abs(mark_list[-1][1] - y <= mark_inter + i[2] / damping *5)):
                mark_list.append([x,y])
            else:
               error += 1
            if(error > error_rate):
                mark_list = []
                error = 0

        if(len(mark_list) >= simple_point):
            for i in mark_list:
                sum_x += i[0]
                sum_y += i[1]
            cood_x = sum_x / (len(mark_list))
            cood_y = sum_y / (len(mark_list))
            #print(int(cood_x),int(cood_y))
            rho = np.sqrt(cood_x**2 + cood_y**2)
            phi = np.arctan2(cood_y, cood_x)
            #print('Angle: ', phi/np.pi*180)
            ann = plt.annotate(('marker'), (phi,rho), color = "purple", fontsize = 9)
            ann_list.append(ann)

        else:
            print('Marker did not detected!')
        scan_list = []
    return line,

def run():
    lidar = RPLidar(PORT_NAME)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=cm.gnuplot2, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    iterator = lidar.iter_scans(max_buf_meas = 2000)
    ani = animation.FuncAnimation(fig, update_line, fargs=(iterator, line), interval=500)
    plt.show()
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    run()