# -*- coding: utf-8 -*-
"""
Created on Mon Feb 19 15:38:58 2024

@author: eenmv
"""


import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
import time

# cube is always 1 unit
LightN = 32
LightCube = np.zeros([LightN,LightN,LightN,3],dtype="bool") # cube N*N*N*3 RGB
DrawPriority = np.zeros([LightN,LightN,LightN],dtype="byte")

#Time step
dT = 0.01


# Setup simulation configuration
SimConfig = {}

SimConfig["Ion_N"] = 25
SimConfig["Ion_Mass"] = 208
SimConfig["Ion_Position"] = np.array([0.5,0.5,0.95])
SimConfig["Ion_Velocity"] = np.array([0.1,0.1,-1])

SimConfig["Film_Thickness"] = 0.75
SimConfig["Film_Mass"] = 29
SimConfig["Film_N_Density"] = 100
SimConfig["Film_SecondaryThreshold"] = 25
SimConfig["Film_StickThreshold"] = 0.05


# Base particle class
class particle:
    def __init__(self,Pos,Vel,Mass=1,Charge=1,Col=[1,0,0],DrawPri=1,ID="None"):
        
        #Might be useful later???
        self.ID = ID
        self.LineIDs = []
        #inital condition
        self.Pos = Pos
        self.Vel = Vel
        self.Mass = Mass
        self.Charge = Charge
        #colour
        self.Col = Col
        #draw priority, high more important
        # Intial ion should have larger priority for example
        self.DrawPri = DrawPri
        self.Trail = False
        self.PastPos = [Pos]
    
    def move(self,ThinFilm,dT):
        
        E = np.linalg.norm(self.Vel)*self.Mass*0.5 # Calculate kinetic energy
        
        InFilm = (self.Pos[2]<ThinFilm.Thk)
        
        # Film is sticka and below threshold strongly damp velocity
        if (E < ThinFilm.StickThreshold) & InFilm:
            self.Vel = self.Vel*0.1
        
        self.Pos = self.Pos + dT*self.Vel
        
        Dis = np.linalg.norm(self.Vel*dT)
        
        #Probablity of Collision
        
        #Check if in thin film, 50% chance of collision per N_Den distance
        if ( (Dis/2)>np.random.random()*ThinFilm.N_Den ) & InFilm:
            #inside film, do the 'monto carlo'
            
            # Only store collisions points
            if self.Trail:
                self.PastPos.append(self.Pos)
            
            #get unit velocity vector of particle
            UnitVel = self.Vel/np.linalg.norm(self.Vel)
            
            #pick random point in space, as contact vector
            ConVec = np.random.random(3)*2-1
            ConVec = ConVec/np.linalg.norm(ConVec) #normalize
            DotProd = np.dot(ConVec,UnitVel)
            

            if DotProd<0:#mirror for only forward facing collisions
                ConVec = ConVec - 2*UnitVel*DotProd
            
            
            IntDotVel = np.dot(ConVec,self.Vel)
            
            # calculate momentum exchange
            FinalDotVel = ( (self.Mass-ThinFilm.Mass)/(self.Mass+ThinFilm.Mass) ) * IntDotVel
            
            self.Vel = self.Vel + (FinalDotVel-IntDotVel)*ConVec
            
            #Return momentum exchange for particle generation
            return self.Mass*(FinalDotVel-IntDotVel)*ConVec
        return np.array([0,0,0])


# Base thin film class
class thinFilm:
    def __init__(self,Thk = 0.75, Mass = 1, N_Den = 1, SecondThreshold = 0.5, StickThreshold = 0.01):
        self.Thk = Thk
        self.Mass = Mass #mean mass of thin film particles
        self.N_Den = 1/N_Den #mean distance between collision
        self.SecondThreshold = SecondThreshold
        self.StickThreshold = StickThreshold


# Main GUI window to display animation, outputs lightcube data at end
# TODO convert canvas to matplotlib 3d plotting instead.
class Window(tk.Frame):
    """
    Main window for AutoLab
    This class should handle GUI
    """
    
    def __init__(self, master):
        """
        Initial setup of widgets and the general window position
        """
        
        global SimConfig
        
        super().__init__(master)
        
        self.Size = 400
        
        self.Angle = 0
        
        self.Draw = tk.Canvas(master,width=self.Size,height=self.Size)
        self.Draw.pack()
        
        self.Film = thinFilm(SimConfig["Film_Thickness"],
                             SimConfig["Film_Mass"],
                             SimConfig["Film_N_Density"],
                             SimConfig["Film_SecondaryThreshold"],
                             SimConfig["Film_StickThreshold"])
        
        self.drawFilmBoundary()
        
        self.Particles = []
        
        
        for I in range(25):
            
            Ion = particle(SimConfig["Ion_Position"],
                           SimConfig["Ion_Velocity"],
                           SimConfig["Ion_Mass"])
            
            Ion.Trail = True
            Ion.DrawPri = 2
            
            self.Particles.append(Ion)
        
        
        self.drawParticles()
        
        self.PreviosTime = time.time()
        self.Steps = 0
        
        self.update()
        
    
    def update(self):
        global dT
        
        NewParticles = []
        
        self.Angle = self.Angle + 0.01
        
        Energy = 0
        
        for P in self.Particles:
            
            M = P.move(self.Film,dT)
            if type(M) != None:
                MagM = np.linalg.norm(M)
                
                if MagM > self.Film.SecondThreshold:
                    Secondary = particle(P.Pos, -M/self.Film.Mass,Mass=self.Film.Mass,Col=[0,1,0])
                    NewParticles.append(Secondary)
                
                Energy += np.linalg.norm(P.Vel)**2*P.Mass*0.5
        
        print("Total Energy: {}".format(Energy))
        
        for P in NewParticles:
            self.Particles.append(P)
        
        self.drawParticles()
        self.Steps += 1
        
        if self.Steps > 300:
            self.outputCube()
            self.plotCube()
            self.destroy()
        else:
            self.after(16,self.update)
    
    def drawFilmBoundary(self):
        
        Y = self.Film.Thk
        
        Y = self.Size*(1-Y)
        
        self.ThinFilm_Line = self.Draw.create_line(0,Y,self.Size,Y,fill="Blue")
    
    def drawParticles(self):
        
        for P in self.Particles:
            
            # flatten depth
            X = self.Size * ((0.5-P.Pos[0])*np.sin(self.Angle) + 
                             (0.5-P.Pos[1])*np.cos(self.Angle) +
                              0.5)
            Y = self.Size * (1-P.Pos[2])
            
            R = 3
            
            #create new oval
            if P.ID == "None":
            #if True:
                
                Col = self.convertColour(P.Col)
                
                ID = self.Draw.create_oval(X-R,Y-R,X+R,Y+R,fill=Col)
                
                P.ID = ID
            else:
                self.Draw.coords(P.ID,X-R,Y-R,X+R,Y+R)
            
            if P.Trail:
                if len(P.PastPos) != len(P.LineIDs):
                    
                    X1 = self.Size * ((0.5-P.Pos[0])*np.sin(self.Angle) + 
                                     (0.5-P.Pos[1])*np.cos(self.Angle) +
                                      0.5)
                    Y1 = self.Size * (1-P.Pos[2])
                    
                    X2 = self.Size * ((0.5-P.PastPos[-1][0])*np.sin(self.Angle) + 
                                     (0.5-P.PastPos[-1][1])*np.cos(self.Angle) +
                                      0.5)
                    Y2 = self.Size * (1-P.PastPos[-1][2])
                    
                    ID = self.Draw.create_line(X1,Y1,X2,Y2,width=2,fill="red")
                    
                    P.LineIDs.append(ID)
                
                else:
                    
                    X1 = self.Size * ((0.5-P.Pos[0])*np.sin(self.Angle) + 
                                     (0.5-P.Pos[1])*np.cos(self.Angle) +
                                      0.5)
                    Y1 = self.Size * (1-P.Pos[2])
                    
                    X2 = self.Size * ((0.5-P.PastPos[-1][0])*np.sin(self.Angle) + 
                                     (0.5-P.PastPos[-1][1])*np.cos(self.Angle) +
                                      0.5)
                    Y2 = self.Size * (1-P.PastPos[-1][2])
                    
                    self.Draw.coords(P.LineIDs[-1],X1,Y1,X2,Y2)
                    
                N = len(P.LineIDs)
                for n in range(N-1):
                    
                    ID = P.LineIDs[n]
                    
                    
                    
                    X1 = self.Size * ((0.5-P.PastPos[n][0])*np.sin(self.Angle) + 
                                     (0.5-P.PastPos[n][1])*np.cos(self.Angle) +
                                      0.5)
                    Y1 = self.Size * (1-P.PastPos[n][2])
                    
                    X2 = self.Size * ((0.5-P.PastPos[n+1][0])*np.sin(self.Angle) + 
                                     (0.5-P.PastPos[n+1][1])*np.cos(self.Angle) +
                                      0.5)
                    Y2 = self.Size * (1-P.PastPos[n+1][2])
                    
                    self.Draw.coords(ID,X1,Y1,X2,Y2)
                    
                    
                    
    
    def convertColour(self,Col):
        
        Str = "#"
        Str=Str+"{:02x}".format(round(Col[0]*255))
        Str=Str+"{:02x}".format(round(Col[1]*255))
        Str=Str+"{:02x}".format(round(Col[2]*255))

            
        return Str
    
    
    def outputCube(self):
        
        global LightCube,LightN,DrawPriority
        
        Start = time.perf_counter()
        
        LightCube[:,:,:,:] = False #clear cube
        DrawPriority[:,:,:] = 0 #Clear Draw Priority
        
        for P in self.Particles:
            Pos = P.Pos
            Pos = np.round(Pos*LightN)
            
            i = int(Pos[0])
            j = int(Pos[1])
            k = int(Pos[2])
            
            if max([i,j,k]) < LightN and min([i,j,k]) >= 0:
                if DrawPriority[i,j,k] < P.DrawPri: #Check is the draw priority is larger
                    if P.Col[0]>0.5:
                        LightCube[i,j,k,0] = True
                    if P.Col[1]>0.5:
                        LightCube[i,j,k,1] = True
                    if P.Col[2]>0.5:
                        LightCube[i,j,k,2] = True
                
                    DrawPriority[i,j,k] = P.DrawPri
            
            # Draw any trails for the particle
            if P.Trail:
                N = len(P.PastPos)
                self.drawLineCube(P.Pos, P.PastPos[-1], P.Col, P.DrawPri)
                for n in range(N-1):
                    self.drawLineCube(P.PastPos[n], P.PastPos[n+1], P.Col, P.DrawPri)

        End = time.perf_counter()
        
        print("Seconds to draw cube: {}".format((End-Start)))
    
    
    def drawLineCube(self,P1,P2,Col,DrawPri):
        
        global LightCube,LightN,DrawPriority
        N = int(np.linalg.norm(P2-P1)*LightN*2) # Number of sample points
        if N==0:
            return
        for n in range(N+1):
            
            Pos = ( n/N * (P2-P1) + P1) * LightN
            i = int(Pos[0])
            j = int(Pos[1])
            k = int(Pos[2])
            
            #Check if in bounds
            if max([i,j,k]) < LightN and min([i,j,k]) >= 0:
                if DrawPriority[i,j,k] < DrawPri:
                    if Col[0]>0.5:
                        LightCube[i,j,k,0] = True
                    if Col[1]>0.5:
                        LightCube[i,j,k,1] = True
                    if Col[2]>0.5:
                        LightCube[i,j,k,2] = True
                
                    DrawPriority[i,j,k] = DrawPri
    
    def plotCube(self):
        
        global LightCube
        
        voxelarray = LightCube[:,:,:,0] | LightCube[:,:,:,1] | LightCube[:,:,:,2]
        
        colors = np.empty(voxelarray.shape, dtype=object)
        colors[LightCube[:,:,:,0]] = 'red'
        colors[LightCube[:,:,:,1]] = 'green'
        colors[LightCube[:,:,:,2]] = 'blue'

        # and plot everything
        ax = plt.figure().add_subplot(projection='3d')
        ax.voxels(voxelarray, facecolors=colors)

        plt.show()
            
                
        
    

if __name__=="__main__":
    
    #Make and start main window
    root = tk.Tk()
    Sim = Window(root)
    root.title("Autolab")

    Sim.mainloop()
