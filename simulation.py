import numpy as np
from non_linear_plant import nl_Quadrotor
from lqr import LQR

simtime = 100
Ts = 0.1
quad = Quadrotor(initStates = np.zeros(12,1))
#cont = Controller()
reference = np.array([1,0,1,0,0,0]).reshape(-1,1)
for i in simtime:
    error = quad.output-reference
    quad.update(np.zeros((3,1)), np.zeros((3,1)), Ts)
    #cont.update(quad.states, error)