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

lightCube = np.zeros([24,24,32,3],dtype='bool')


def Main(port):
    
    # 100 characters
    testStr = "abcdefghi-abcdefghi-abcdefghi-abcdefghi-abcdefghi-abcdefghi-abcdefghi-abcdefghi-abcdefghi-abcdefghi-"
    #testStr = "abcdefghi-abcdefghi-abcdefghi-"
    
    
    packet = packaging()
    print("packet size: {}".format(len(packet)))
    
    PacketSize = len(packet)
    totalSend = 0
    
    start = time.time()
    
    received = 0
    
    while(1):
        
        start = time.time()
        
        packet = packaging()
        
        now = time.time()
        
        print("Packaging Array time: {}".format(now-start))
        
        totalSend += PacketSize
        
        start = time.time()
        
        port.write(bytearray(packet,'utf-8'))
        
        now = time.time()
        
        print("Sending Packet time: {}".format(now-start))
        
        if port.inWaiting()>0:
            port.flushInput()
            received += 2**16
        
        
        if totalSend%1e5 < 10:
            now = time.time()
            
            if now-start != 0:
                rate = totalSend/(now-start) * 8
            
                print("Baudrate roughly: {} : total send {} : total Received {}".format(rate, totalSend,received))



def packaging():
    
    global lightCube
    
    framepacket = chr(0b10000000) # start with the reset character
    
    for k in range(32):
        for j in range(24):
            for i in range(12):
                
                char = ( 0b01000000 |
                        lightCube[2*i,j,k,0] << 0 |
                        lightCube[2*i,j,k,1] << 1 |
                        lightCube[2*i,j,k,2] << 2 |
                        lightCube[2*i+1,j,k,0] << 3 |
                        lightCube[2*i+1,j,k,1] << 4 |
                        lightCube[2*i+1,j,k,2] << 5
                        )
                
                framepacket += chr(char)
    
    return framepacket


if __name__ == "__main__":
    ports = serial.tools.list_ports.comports()
    #print("Pick a Comm port numder: "+str(ports))
    #comm = int(input("Selection: "))
    port = serial.Serial("COM"+str(18),900000)
    Main(port)
    port.close()
