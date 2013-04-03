import pymorse
import time
import numpy as np
from scipy.linalg import solve_continuous_are

'''
If you are confident in the measures, increase
the confidence so that measure_confidence >> 1.
Otherwise, make it << 1. 
'''
measure_confidence = 1

Aext = np.mat('[0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0]')
Cext = np.mat('[1 0 0 0; 0 0 1 0]')
n = Aext.shape[0]
ny = Cext.shape[0]
G = np.identity(n);
Q = measure_confidence * np.identity(n);
R = np.identity(ny);

P = solve_continuous_are(Aext.T, Cext.T, Q, R)

L = P * Cext.T * R;
  
dt = .1

A = np.identity(n) + dt * (Aext - L * Cext)
B = L * dt
C = Cext

xest_k = np.mat('[0; 0; 0; 0]')

with pymorse.Morse() as morse:

    while True:
        
        pose = morse.robot.pose.get()
        z = np.mat([[pose['x']], [pose['y']]])
        
        x_ = C * xest_k
        
        pose['x'] = float(x_[0][0])
        pose['y'] = float(x_[1][0])
        morse.ghost.teleport.publish(pose)

        xest_k = A * xest_k + B * z
        
        time.sleep(dt)
