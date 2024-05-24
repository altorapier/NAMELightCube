# -*- coding: utf-8 -*-
"""
Created on Mon Apr 29 17:50:17 2024

@author: eenmv
"""
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt

import time


lightCube = np.zeros([8,8,32,3],dtype='bool')


def Main(port):
    
    global lightCube
    
    cubePort = port
    
    while(1):
        
        for col in range(3):
            for layer in range(32):
                lightCube[:,:,:,:] = False
                #lightCube[:,:,layer,col] = True
                
                lightCube[:,:,layer,0] = True
                lightCube[:,:,layer,1] = True
                lightCube[:,:,layer,2] = True
                
                Send(cubePort,lightCube)
                
                foo = input("Next")
                
                time.sleep(0)
        


def Send(port,lightCube):
    
    framepacket = chr(0b10000000) # start with the reset character
    
    for k in range(32):
        for j in range(8):
            for i in range(4):
                
                char = ( 0b01000000 |
                        lightCube[2*i,j,k,0] << 0 |
                        lightCube[2*i,j,k,1] << 1 |
                        lightCube[2*i,j,k,2] << 2 |
                        lightCube[2*i+1,j,k,0] << 3 |
                        lightCube[2*i+1,j,k,1] << 4 |
                        lightCube[2*i+1,j,k,2] << 5
                        )
                
                framepacket += chr(char)
    
    
    port.write(bytearray(framepacket,'utf-8'))
                       
                       
    
    pass

if __name__ == "__main__":
    ports = serial.tools.list_ports.comports()
    print("Pick a Comm port numder: "+str(ports))
    comm = int(input("Selection: "))
    port = serial.Serial("COM"+str(comm),115200)
    Main(port)
    cubePort.close()
