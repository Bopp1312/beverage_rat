#! /usr/bin/env python3

import numpy as np

def rotx(angle):
    return np.matrix([
         [1,0,0],
         [0,np.cos(angle),-1*np.sin(angle)],
         [0,np.sin(angle),np.cos(angle)]
         ])

def roty(angle):
    return np.matrix([
         [np.cos(angle),0,np.sin(angle)],
         [0,1,0],
         [-1*np.sin(angle),0,np.cos(angle)]
         ])

def rotz(angle):
    return np.matrix([
         [np.cos(angle),-1*np.sin(angle),0],
         [np.sin(angle),np.cos(angle),0],
         [0,0,1]
         ])

def transformTranx(x):
    return np.matrix([[1,0,0,x],
                      [0,1,0,0],
                      [0,0,1,0],
                      [0,0,0,1]])

def transformTrany(y):
    return np.matrix([[1,0,0,0],
                      [0,1,0,y],
                      [0,0,1,0],
                      [0,0,0,1]])

def transformTranz(z):
    return np.matrix([[1,0,0,0],
                      [0,1,0,0],
                      [0,0,1,z],
                      [0,0,0,1]])

def transformTranxyz(x,y,z):
    return transformTranx(x)*transformTrany(y)*transformTranz(z)

def transformRotx(x):
    mat = np.identity(4)
    mat[:3, :3] = rotx(x)
    return mat
    
def transformRoty(y):
    mat = np.identity(4)
    mat[:3, :3] = roty(y)
    return mat
 
def transformRotz(z):
    mat = np.identity(4)
    mat[:3, :3] = rotz(z)
    return mat

def homogenousTranspose(transform):
    oldRotation = transform[:3,:3]
    rotation  = oldRotation.transpose()
    position = transform[:3,3]
    newPosition = -1*(rotation * position)
    return np.matrix([[rotation[0,0],rotation[0,1],rotation[0,2],newPosition[0]],
                      [rotation[1,0],rotation[1,1],rotation[1,2],newPosition[1]],
                      [rotation[2,0],rotation[2,1],rotation[2,2],newPosition[2]],
                      [0,0,0,1]])

def dhTransform(alpha,a,theta,d):
    # Denevitt Hartenburg convention states
    # A rotation      about xi-1   of alpha
    # A translation   about xi-1   of a
    # A rotation      about zi     of theta
    # A translation   about zi     of d
    return transformRotx(alpha)*transformTranx(a)*transformRotz(theta)*transformTranz(d)


#print(dhTransform(0,0,0,0))
        



if __name__ == "__main__":
    print("I am main")

