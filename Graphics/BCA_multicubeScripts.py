# -*- coding: utf-8 -*-
"""
Created on Thu May 23 11:22:16 2024

@author: eenmv
"""

import numpy as np
import pandas as pd
import time
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

packet = ""

def getUpdatedVoxels(NewFrame, OldFrame):
    Diff = np.bitwise_xor(NewFrame, OldFrame)
    DiffPos = np.nonzero(Diff)
    UpdatePacket = ""
    
    i_last = -1
    j_last = -1
    k_last = -1
    
    for Pn in range(len(DiffPos[0])):
        
        i = DiffPos[0][Pn]
        j = DiffPos[1][Pn]
        k = DiffPos[2][Pn]
        
        if i_last == i and j_last == j and k_last == k:
            continue
        else:
            i_last = i
            j_last = j
            k_last = k
            
            UpdatePacket += chr(0b01000000 |
                                NewFrame[i, j, k, 0] << 5 |
                                i)
            UpdatePacket += chr(0b00000000 |
                                NewFrame[i, j, k, 1] << 5 |
                                j)
            UpdatePacket += chr(0b00000000 |
                                NewFrame[i, j, k, 2] << 5 |
                                k)
    return UpdatePacket

def read_excel_data(file_path):
    df = pd.read_excel(file_path)
    frames = {}
    max_frame = df['Frame'].max()
    
    for frame in range(max_frame + 1):
        frame_data = df[df['Frame'] == frame]
        cube_frame = np.zeros([24, 24, 32, 3], dtype='bool')
        
        for _, row in frame_data.iterrows():
            x, y, z = int(row['X']), int(row['Y']), int(row['Z'])
            if row['LED'] == 1:
                cube_frame[x, y, z] = [True, False, False]
            elif row['LED'] == 2:
                cube_frame[x, y, z] = [False, True, False]
            elif row['LED'] == 3:
                cube_frame[x, y, z] = [False, True, True]
        
        frames[frame] = cube_frame
    
    return frames

def visualize_cube(cube):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([0, 24])
    ax.set_ylim([0, 24])
    ax.set_zlim([0, 32])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    def plot_voxels(voxels, color):
        x, y, z = np.nonzero(voxels)
        ax.scatter(x, y, z, c=color, marker='o')

    plot_voxels(cube[..., 0], 'r')
    plot_voxels(cube[..., 1], 'g')
    plot_voxels(cube[..., 2], 'b')

    plt.draw()
    plt.pause(0.001)

def testSpeed(commPortID, frames):
    global packet
    
    try:
        cubePort = serial.Serial("COM" + str(commPortID), 115200)
        print("Connected to cube")
    except:
        cubePort = None
        print("Failed to connect to cube")
    
    oldCube = np.zeros([24, 24, 32, 3], dtype='bool')
    
    frame_keys = sorted(frames.keys())
    N = 0

    plt.ion()  # Turn on interactive mode for live updating
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([0, 24])
    ax.set_ylim([0, 24])
    ax.set_zlim([0, 32])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    while N < len(frame_keys):
        cube = frames[frame_keys[N]]
        
        start = time.perf_counter()
        
        packet = getUpdatedVoxels(cube, oldCube)
        
        stop = time.perf_counter()
        
        print("Packet Generation Time {}".format(stop - start))
        
        start = time.perf_counter()
        
        if cubePort is not None:
            cubePort.write(bytearray(packet, 'utf-8'))
        
        stop = time.perf_counter()
        
        print("Packet Send Time {}".format(stop - start))
        
        ax.clear()
        ax.set_xlim([0, 24])
        ax.set_ylim([0, 24])
        ax.set_zlim([0, 32])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        visualize_cube(cube)
        
        N = (N + 1)
        
        oldCube = np.copy(cube)
        
        time.sleep(0.1)  # Adjust the sleep time as per the desired frame rate

if __name__ == "__main__":
    try:
        Ports = serial.tools.list_ports.comports()
        if len(Ports) == 0:
            raise
        print(*Ports)
        CommPortID = int(input("Select Port: "))
    except:
        CommPortID = None
        print("Could not connect to THE CUBE")
    
    file_path = input("Enter the path to the Excel file: ")
    frames = read_excel_data(file_path)
    
    testSpeed(CommPortID, frames)
