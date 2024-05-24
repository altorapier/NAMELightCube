# -*- coding: utf-8 -*-
"""
Created on Thu May 23 11:22:16 2024

@author: eenmv
"""

import numpy as np


def getUpdatedVoxels(NewFrame,OldFrame):
    Diff = NewFrame - OldFrame
    UpdateCommands = []
    for i in range(Diff.shape[0]):
        for j in range(Diff.shape[1]):
            for k in range(Diff.shape[2]):
                if Diff[i,j,k] != np.zeros(3,dtype='bool'):
                    #Encoding
                    
                    UpdateCommands.append(12345)