# -*- coding: utf-8 -*-
"""
Created on Thu May 23 11:22:16 2024

@author: eenmv
"""

import numpy as np
import time


def getUpdatedVoxels(NewFrame,OldFrame):
    Diff = np.bitwise_xor(NewFrame,OldFrame)
    DiffPos = np.nonzero(Diff)
    UpdatePacket = ""
    
    i_last = -1
    j_last = -1
    k_last = -1
    
    for Pn in range(len(DiffPos[0])):
        
        i = DiffPos[0][Pn]
        j = DiffPos[1][Pn]
        k = DiffPos[2][Pn]
        
        if i_last == i and j_last == j  and k_last == k:
            continue
        else:
            
            i_last = i
            j_last = j
            k_last = k
            
            UpdatePacket += chr(0b10000000 |
                                NewFrame[i,j,k,0] << 6 |
                                i)
            UpdatePacket += chr(0b00000000 |
                                NewFrame[i,j,k,1] << 6 |
                                j)
            UpdatePacket += chr(0b00000000 |
                                NewFrame[i,j,k,2] << 6 |
                                k)
    return UpdatePacket
    
def testSpeed():
    
    cube = np.zeros([24,24,32,3],dtype='bool')
    oldCube = np.zeros([24,24,32,3],dtype='bool')
    
    N = 0
    
    while(True):
        cube[:,:,:,:] = False
        cube[:,:,N,1] = True
        
        start = time.perf_counter()
        
        packet = getUpdatedVoxels(cube, oldCube)
        
        stop = time.perf_counter()
        
        print("Packet Generation Time {}".format(stop-start))
        
        print(len(packet))
        
        oldCube = np.copy(cube)
        
        N = (N+1)%32

if __name__=="__main__":
    testSpeed()
                    