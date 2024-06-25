# -*- coding: utf-8 -*-
"""
Created on Mon Feb 19 15:38:58 2024

@author: eenmv
"""

import numpy as np
import tkinter as tk
from tkinter import font as tkFont
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import os

# Constants and configurations
dT = 0.1
MinDelay = 0.01
SurfaceOnly = False

LightN = [24, 24, 32]
LightCube = np.zeros([LightN[0], LightN[1], LightN[2], 3], dtype="bool")

SimConfig = {
    "Ion_N": 1,
    "Ion_Mass": 2,
    "Ion_Position": np.array([0.5, 0.5, 0.95]),
    "Ion_Velocity": np.array([0, 0, -1]),
    "Film_Thickness": 0.75,
    "Film_Mass": 29,
    "Film_N_Density": 40,
    "Film_SecondaryThreshold": 25,
    "Film_StickThreshold": 0.15
}

# Base particle class
class Particle:
    def __init__(self, Pos, Vel, Mass=1, Charge=1, Col=[1, 0, 0], DrawPri=1, ID="None"):
        self.ID = ID
        self.Pos = Pos
        self.Vel = Vel
        self.Mass = Mass
        self.Charge = Charge
        self.Col = Col
        self.DrawPri = DrawPri
        self.Trail = False
        self.PastPos = [Pos]

    def move(self, ThinFilm, dT):
        E = np.linalg.norm(self.Vel) ** 2 * self.Mass * 0.5
        InFilm = (self.Pos[2] < ThinFilm.Thk)
        Dis = np.linalg.norm(self.Vel) * dT
        
        if InFilm and E != 0:
            E_lost = np.clip(np.power(E, 0.5) * 5 * Dis, 0, E)
            fraction = np.sqrt((E - E_lost) * 2 / self.Mass) / np.linalg.norm(self.Vel)
            self.Vel = self.Vel * fraction
            
        self.Pos = self.Pos + dT * self.Vel
        
        if InFilm:
            self.PastPos.append(self.Pos)

# Base thin film class
class ThinFilm:
    def __init__(self, Thk=0.75, Mass=1, N_Den=1, SecondThreshold=0.5, StickThreshold=0.01):
        self.Thk = Thk
        self.Mass = Mass
        self.N_Den = 1 / N_Den
        self.SecondThreshold = SecondThreshold
        self.StickThreshold = StickThreshold

# Visualization function
def plotCube(particles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([0, 1])
    ax.set_ylim([0, 1])
    ax.set_zlim([0, 1])
    
    for p in particles:
        xs, ys, zs = zip(*p.PastPos)
        ax.plot(xs, ys, zs, label=f'Particle {p.ID}', color=p.Col)
    
    plt.show()

# Initialize particles and thin film
particles = []
ThinFilmInstance = ThinFilm(SimConfig["Film_Thickness"], SimConfig["Film_Mass"], SimConfig["Film_N_Density"], SimConfig["Film_SecondaryThreshold"], SimConfig["Film_StickThreshold"])

Mass = SimConfig["Ion_Mass"]
Pos = SimConfig["Ion_Position"]
Vel = SimConfig["Ion_Velocity"]
particle_instance = Particle(Pos, Vel, Mass)
particles.append(particle_instance)

# Simulation loop
for t in range(100):
    for particle in particles:
        particle.move(ThinFilmInstance, dT)

# Visualization
plotCube(particles)
