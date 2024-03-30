import numpy as np
from numpy import sin, cos, tan

class l_Quadrotor():
    def __init__(self, initStates):
        #Phisical characteristics
        self.g = 9.81
        self.sphmass = 8
        self.sphR = 0.1
        self.armlength = 0.2
        self.rotormass = 0.1
        self.qmass = 10

        #Moment of inertia 
        self.Jx = 2/5. * self.sphmass * self.sphR**2 + 2 * self.rotormass * self.armlength**2
        self.Jy = 2/5. * self.sphmass * self.sphR**2 + 2 * self.rotormass * self.armlength**2
        self.Jz = 2/5. * self.sphmass * self.sphR**2 + 4 * self.rotormass * self.armlength**2

        #Output matrix
        self.out_mat = np.matrix([[1,0,0,0,0,0,0,0,0,0,0,0],
                                 [0,1,0,0,0,0,0,0,0,0,0,0],
                                 [0,0,1,0,0,0,0,0,0,0,0,0],
                                 [0,0,0,0,0,0,1,0,0,0,0,0],
                                 [0,0,0,0,0,0,0,1,0,0,0,0],
                                 [0,0,0,0,0,0,0,0,1,0,0,0]])

    def update(self, Ts, statesPrev,force_input, torque_input):
        self.unpack_states(statesPrev)
        self.get_R_mat()
        self.get_T_mat()

        #Calculate derivatives
        pos_dot = self.R*np.array([[self.u],[self.v],[self.w]])
        v_dot = np.array([[self.r*self.v-self.q*self.w],[self.p*self.w-self.r*self.u],[self.q*self.u-self.p*self.v]])+1/self.m*force_input
        rot_dot = self.T * self.array([[self.p],[self.q],[self.r]])
        rot_vec_dot = self.array([[(self.Jy-self.Jz)/self.Jx * self.q*self.r],[(self.Jz-self.Jx)/self.Jy * self.p * self.r],[(self.Jx-self.Jy)/self.Jz * self.p * self.q]]) + np.array([[1/self.Jx*torque_input[0]],[1/self.Jy*torque_input[1]],[1/self.Jz*torque_input[2]]])
        #Unify the state derivative vector
        state_vec_dot = np.vstack([pos_dot, v_dot, rot_dot, rot_vec_dot])

        #solve the differential equations
        self.states = state_vec_dot*Ts + statesPrev
        self.output = self.out_mat*self.states

    def unpack_states(self, states_in):
        self.x = states_in[0]
        self.y = states_in[1]
        self.h = states_in[2]
        self.u = states_in[3]
        self.v = states_in[4]
        self.w = states_in[5]
        self.phi = states_in[6]
        self.theta = states_in[7]
        self.psi = states_in[8]
        self.p = states_in[9]
        self.q = states_in[10]
        self.r = states_in[11]
   
    def get_R_mat(self):
        self.R = np.matrix([[cos(self.theta)*cos(self.psi), sin(self.phi)*sin(self.theta)*cos(self.psi) - cos(self.phi)*sin(self.psi), cos(self.phi)*sin(self.theta)*cos(self.psi)+sin(self.phi)*sin(self.psi)],
                            [cos(self.theta)*sin(self.psi), sin(self.phi)*sin(self.theta)*sin(self.psi)+cos(self.phi)*cos(self.psi), cos(self.phi)*sin(self.theta)*sin(self.psi)-sin(self.phi)*cos(self.psi)],
                            [sin(self.theta), -sin(self.phi)*cos(self.theta), -cos(self.phi)*cos(self.theta)]]) 

    def get_T_mat(self):
        self.T = np.matrix([[1, sin(self.phi)*tan(self.theta), cos(self.phi)*tan(self.theta)],
                            [0, cos(self.phi), -sin(self.phi)],
                            [0, sin(self.phi)/cos(self.theta), cos(self.phi)/cos(self.theta)]])

#state vec is 
# x y z u v w phi theta psi p q r
