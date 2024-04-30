# -*- coding: utf-8 -*-
"""
Created on Mon Apr 29 17:50:17 2024

@author: eenmv
"""
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt


lightCube = np.zeros([8,8,32,3],dtype='bool')


# LightCube[:,:,10,0] = True
# LightCube[:,:,11,1] = True
# LightCube[:,:,12,2] = True



# voxelarray = LightCube[:,:,:,0] | LightCube[:,:,:,1] | LightCube[:,:,:,2]

# colors = np.empty(voxelarray.shape, dtype=object)
# colors[LightCube[:,:,:,0]] = 'red'
# colors[LightCube[:,:,:,1]] = 'green'
# colors[LightCube[:,:,:,2]] = 'blue'

# # and plot everything
# ax = plt.figure().add_subplot(projection='3d')
# ax.voxels(voxelarray, facecolors=colors)

# plt.show()


def Main(comm):
    
    global lightCube
    
    cubePort = serial.Serial("COM"+str(comm),115200)
    
    lightCube[:,:,10,0] = True
    #lightCube[:,:,11,1] = True
    #lightCube[:,:,12,2] = True
    
    voxelarray = lightCube[:,:,:,0] | lightCube[:,:,:,1] | lightCube[:,:,:,2]

    colors = np.empty(voxelarray.shape, dtype=object)
    colors[lightCube[:,:,:,0]] = 'red'
    colors[lightCube[:,:,:,1]] = 'green'
    colors[lightCube[:,:,:,2]] = 'blue'

    # and plot everything
    ax = plt.figure().add_subplot(projection='3d')
    ax.voxels(voxelarray, facecolors=colors)

    plt.show()
    
    cubePort.close()

def Send(port,lightCube):
    
    framepacket = chr(0b10000000) # start with the reset character
    
    for i in range(4):
        for j in range(8):
            for k in range(32):
                
                char = (chr(0b01000000) |
                        lightCube[2*i,j,k,0] << 0 |
                        lightCube[2*i,j,k,1] << 1 |
                        lightCube[2*i,j,k,2] << 2 |
                        lightCube[2*i+1,j,k,1] << 3 |
                        lightCube[2*i+1,j,k,1] << 4 |
                        lightCube[2*i+1,j,k,1] << 5
                        )
                
                framepacket += char
    
    
    port.write(framepacket)
                       
                       
    
    pass

if __name__ == "__main__":
    ports = serial.tools.list_ports.comports()
    print("Pick a Comm port numder: "+str(ports))
    comm = int(input("Selection: "))
    Main(comm)
    pass
