import numpy as np
import scipy as sp

class LQR():
    def __init__(self, A, B, Q, R):
        self.P = sp.linalg.solve_discrete_are(A, B, Q, R)
        self.K = np.linalg.inv(R)*np.transpose(B)*self.P
        
    def update(self, states):
        output = - self.K * states
        self.forces = output[1:3,:]
        self.torques = output[4:,:]