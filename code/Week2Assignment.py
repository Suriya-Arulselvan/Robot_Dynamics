# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 00:15:43 2020

@author: asuri
"""
import numpy as np
import modern_robotics as mr


def GetTrajectoryForZeroJointTorqueAndFtip(time_in_secs, thetalist0, dthetalist0, g, Mlist, Glist, Slist, dt, intRes):
    time_steps = int(time_in_secs/dt)
    #Zero torque and FtipMatrix matrix generation
    taumat = np.zeros((time_steps, len(thetalist0)))
    FtipMat = np.zeros((time_steps, len(thetalist0)))
    return mr.ForwardDynamicsTrajectory(thetalist0, dthetalist0, taumat, g, FtipMat, Mlist, Glist, Slist, dt, intRes)
    
    
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

'''
Initial configuration and given data: 
    0 velocity
    0 torques at joints
    gravity= 9.81m/s2 in -z direction
'''

thetalist0_config1 = [0,0,0,0,0,0] 
thetalist0_config2 = [0,-1,0,0,0,0]
dthetalist0 =  [0,0,0,0,0,0]
g = np.array([0,0,-9.81])

'''Simulation 1: 
    configuration 1
    timeStep= 1 seconds, 
    integration_resolution = 4, 
    totalSimulationTime = 3 seconds
'''
dt = 0.01
intRes = 4
time_in_secs = 3
[thetamat1, dthetamat1] = GetTrajectoryForZeroJointTorqueAndFtip(time_in_secs, thetalist0_config1, dthetalist0, g, Mlist, Glist, Slist, dt, intRes)
np.savetxt("simulation1Test.csv", thetamat1, delimiter=",")

'''Simulation 2:
    configuration 2
    timeStep = 1 seconds,
    integration_resolution = 4,
    totalSimulationTime = 5 seconds
'''
dt = 0.01
intRes = 4
time_in_secs= 5
[thetamat2, dthetamat2] = GetTrajectoryForZeroJointTorqueAndFtip(time_in_secs, thetalist0_config2, dthetalist0, g, Mlist, Glist, Slist, dt, intRes)
np.savetxt("simulation2Test.csv", thetamat2, delimiter=",")