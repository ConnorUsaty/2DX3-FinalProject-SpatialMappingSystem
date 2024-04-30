""" 

Connor Usaty
usatyc
400409624

Assigned Bus Speed: 24MHz
Assigned Measurement Status LED: PF4 (D3)
Assigned Additional  Status LED: PF0 (D4)

2DX3 - Final Project - Python Script for data collection, processing, and visualization from the MCU

To be used in combination with FinalProject.c Keil project to create a 3D spatial reconstruction of area scanned

"""


import serial
from math import sin, cos
import numpy as np
import open3d as o3d

# define important constants
POINTS_PER_SCAN = 32     # number of points in each 360-degree scan
X_INCREMENT = 100        # x-axis distance in mm between each scan -> 100mm = 10cm

f = open("Scan.xyz", "w")    #create a new file for writing

# currently set COM3 as serial port at 115.2kbps 8N1
s = serial.Serial('COM3')
s.baudrate = 115200
s.bytesize = 8
s.parity = 'N'
s.stopbits = 1

print("Opening: " + s.name)

# wait for user's signal to start the program
input("Press Enter to start communication...")

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

print("Sending 's' to MCU")
s.write(b's')
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission

# recieve measurements from UART of MCU
print("Receiving measurements from MCU")

# initialize variables
points = 0
colours = []
x = 0
measuring = True

# data collection / processing loop
while measuring == True:
    for i in range(POINTS_PER_SCAN):
        # read the data from the UART
        data = s.readline()
        # decode the data from bytes to string
        data = data.decode()
         # print the data
        print("Data received: " + data)

        if data == "Measurements done\n":
            measuring = False
            break


        # split the transmitted data into distance and range status
        data = data.split(",")
        distance = float(data[0])
        rangeStatus = int(data[1])

        # translate measurements to x,y,z coordinate system
        y = distance * sin(-11.25*(i*(3.14/180.0)))
        z = distance * cos(-11.25*(i*(3.14/180.0)))

        # write the data point to the file
        f.write('{0:.2f} {1:.2f} {2:.2f}\n'.format(x, y, z))

        # assign colours to the points based on range status
        if rangeStatus == 0: # point is green
            colours.append([0, 1, 0])
        elif rangeStatus == 1 or rangeStatus == 2: # point is yellow
            colours.append([1, 1, 0])
        else: # rangeStatus == 4 or 7, point is red
            colours.append([1, 0, 0])

        points += 1

    # increment x by X_INCREMENT each scan   
    x += X_INCREMENT

    
#close the port
print("Closing: " + s.name)
s.close()
    
# close the file
f.close()

# start the data visualization process

# read the data in from the file we created
pcd = o3d.io.read_point_cloud("Scan.xyz", format="xyz")

# assign colours to the points
pcd.colors = o3d.utility.Vector3dVector(colours) 

# give each vertex a unique number
yz_slice_vertex = []
for i in range(points):
    yz_slice_vertex.append([i])

# define coordinates to connect lines in each yz slice        
lines = []  

# connect the points in adjacent scans
for i in range(points - POINTS_PER_SCAN):
    lines.append([yz_slice_vertex[i], yz_slice_vertex[i+32]])

# connect adjacent points in the same scan
for i in range(points):
    if (i+1) % 32 == 0: # connects last point in scan to first point in scan
        lines.append([yz_slice_vertex[i], yz_slice_vertex[i-31]])
    elif i < points - 1:
        lines.append([yz_slice_vertex[i], yz_slice_vertex[i+1]])

# this line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

# visualize the colour coded point cloud as well as the lines connecting the points
o3d.visualization.draw_geometries([pcd, line_set])
