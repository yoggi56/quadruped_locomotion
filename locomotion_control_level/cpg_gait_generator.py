from math import *
from scipy.integrate import ode
from locomotion_control_level import locomotion_constants as consts

class CPGGaitGenerator(object):
    def __init__(self):
        # internal ocillator states and parameters
        self.r = [0]*4
        self.w = [0]*4
        self.sum_con = [[0,0],[0,0],[0,0],[0,0]]
        self.x = [0]*4
        self.y = [0]*4

        # gait type params
        self.phi_walk   = [pi, 0, 0.5*pi, 1.5*pi] # walk gait
        self.phi_trot   = [pi, 0, 0, pi] # trot gait
        self.phi_pace   = [pi, 0, pi, 0] # pace gait
        self.phi_gallop = [0, 0, pi, pi] # gallop gait

        # initial leg positions
        self.__P_B3 = {1: {"x": consts.EF_F_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        2: {"x": consts.EF_F_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        3: {"x": -consts.EF_R_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        4: {"x": -consts.EF_R_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                    }

        self.ef_ref_pos = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                2: {"x": 0.0, "y": 0.0, "z": 0.0},
                3: {"x": 0.0, "y": 0.0, "z": 0.0},
                4: {"x": 0.0, "y": 0.0, "z": 0.0},
                }

        # initial oscillator positions
        self.xy_init = [0.0, 0.0, 0.01, 0.01, 0.01, 0.01, 0.01,0.01]
        # self.x1_0 = 0.0#-pi/125
        # self.y1_0 = 0.0#-pi/4
        # self.x2_0 = 0.01#-pi/150
        # self.y2_0 = 0.01#-pi/4
        # self.x3_0 = 0.01#-pi/150
        # self.y3_0 = 0.01#-pi/4
        # self.x4_0 = 0.01#-pi/150
        # self.y4_0 = 0.01#-pi/4
        self.t0 = 0.0

    def set_static_params(self, ampl=1, beta=1.1, alpha=0.25, lamb=1, a=1, x_offset=0.084, y_offset=0.0):
        self.mu = sqrt(ampl)
        self.beta = beta
        self.alpha = alpha
        self.lamb = lamb
        self.a = a
        
        self.x_offset = x_offset
        self.y_offset = y_offset

    def set_gait_params(self, w_sw=0, phi=consts.WALK, Hg=0.08, Ls=0.12, dir=0, rot_dir=0, rot_r=0):
        self.w_sw = w_sw
        self.Hg = Hg
        self.Ls = Ls
        self.dir = dir
        self.rot_dir = rot_dir
        self.rot_r = rot_r

        if phi == consts.WALK:
            self.phi = self.phi_walk
        elif phi == consts.TROT:
            self.phi = self.phi_trot
        elif phi == consts.PACE:
            self.phi = self.phi_pace
        elif phi == consts.GALLOP:
            self.phi = self.phi_gallop

    def hopf_osc(self, t, xy):
        self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3] = xy

        for i in range(4):
            self.r[i] = sqrt(self.x[i]**2 + self.y[i]**2)
            self.w[i] = self.w_sw*((self.beta/(exp(-self.a*self.y[i])+1)) + (1/(exp(self.a*self.y[i])+1)))
            self.sum_con[i][0] = 0
            self.sum_con[i][1] = 0
            for j in range(4):
                self.sum_con[i][0] += self.lamb*(cos(self.phi[i]-self.phi[j])*self.x[j] - sin(self.phi[i]-self.phi[j])*self.y[j])
                self.sum_con[i][1] += self.lamb*(sin(self.phi[i]-self.phi[j])*self.x[j] + cos(self.phi[i]-self.phi[j])*self.y[j])

        
        return [self.alpha*(self.mu-self.r[0]**2)*self.x[0]-self.w[0]*self.y[0]+self.sum_con[0][0], 
                self.alpha*(self.mu-self.r[0]**2)*self.y[0]+self.w[0]*self.x[0]+self.sum_con[0][1],
                self.alpha*(self.mu-self.r[1]**2)*self.x[1]-self.w[1]*self.y[1]+self.sum_con[1][0], 
                self.alpha*(self.mu-self.r[1]**2)*self.y[1]+self.w[1]*self.x[1]+self.sum_con[1][1],
                self.alpha*(self.mu-self.r[2]**2)*self.x[2]-self.w[2]*self.y[2]+self.sum_con[2][0], 
                self.alpha*(self.mu-self.r[2]**2)*self.y[2]+self.w[2]*self.x[2]+self.sum_con[2][1],
                self.alpha*(self.mu-self.r[3]**2)*self.x[3]-self.w[3]*self.y[3]+self.sum_con[3][0], 
                self.alpha*(self.mu-self.r[3]**2)*self.y[3]+self.w[3]*self.x[3]+self.sum_con[3][1]]

    def from_dir_to_angle(self, dx, dy):
        if dx == 0 and dy == 0:
            return 0
        
        return atan2(dy, dx)

    # rot_dir:
    #       1 - CCW
    #       0 - no rotation
    #       -1 - CW
    def mapping_function(self, phi_out, leg_number=1, dir=0, rot_dir=1, r=0):
        # map ef parameters
        #k_y = 0.03
        if -pi <= phi_out < 0:
            Px = -((1/pi)*self.Ls*phi_out + self.Ls/2)# + P_B3[leg_number]["x"]
            Pz = 0
            # Py = 0
        elif 0 <= phi_out <= pi:
            Px = -((1/(2*pi))*self.Ls*sin(2*phi_out) - (1/pi)*self.Ls*phi_out + self.Ls/2)# + P_B3[leg_number]["x"]
            if 0 <= phi_out < pi/2:
                Py = sin(phi_out)
                Pz = (-1/(2*pi))*self.Hg*sin(4*phi_out) + (2/pi)*self.Hg*phi_out# + self.P_B3[leg_number]["z"]
            else:
                Pz = (1/(2*pi))*self.Hg*sin(4*phi_out) - (2/pi)*self.Hg*phi_out + 2*self.Hg# + self.P_B3[leg_number]["z"]
            # if leg_number == 1 or leg_number == 3:
            #     Py = k_y*sin(phi_out)
            # else:
            #     Py = -k_y*sin(phi_out)
        Py = 0#P_B3[leg_number]["y"]

        # change Y direction
        Px_new = Px*cos(dir) - Py*sin(dir)
        Py_new = Px*sin(dir) + Py*cos(dir)
        Px = Px_new
        Py = Py_new

        # Z turning
        if rot_dir != 0:
            if leg_number == 1 or leg_number == 2:
                sign = 1
            else:
                sign = -1

            if rot_dir == -1:
                alpha = -atan2(0.5631, 2*r-0.367)
                beta = -atan2(0.5631, 2*r+0.367)
            else:
                alpha = atan2(0.5631, 2*r+0.367)
                beta = atan2(0.5631, 2*r-0.367)

            if leg_number == 1 or leg_number == 3:
                turn = alpha
            else:
                turn = beta
                
            Px_new = Px*cos(sign*turn) - Py*sin(sign*turn)
            Py_new = Px*sin(sign*turn) + Py*cos(sign*turn)
            Px = Px_new
            Py = Py_new

        # add offset from CoG to endeffector
        Px = Px + self.__P_B3[leg_number]["x"]
        Py = Py + self.__P_B3[leg_number]["y"]
        Pz = Pz + self.__P_B3[leg_number]["z"]

        
        #return [P_B3[leg_number]["x"], P_B3[leg_number]["y"], P_B3[leg_number]["z"]]
        return [Px, Py, Pz]

    def init_integrator(self, dt):
        self.de = ode(self.hopf_osc)
        self.de.set_integrator('dopri5')
        self.de.set_initial_value(self.xy_init, self.t0)
        self.dt = dt # direiative time

    def get_phase(self):
        return self.phi_out1, self.phi_out2, self.phi_out3, self.phi_out4

    def step(self):
        self.de.integrate(self.de.t + self.dt)

        self.phi_out1 = atan2(self.de.y[1], self.de.y[0])
        self.phi_out2 = atan2(self.de.y[3], self.de.y[2])
        self.phi_out3 = atan2(self.de.y[5], self.de.y[4])
        self.phi_out4 = atan2(self.de.y[7], self.de.y[6])

        P_RF = self.mapping_function(self.phi_out1, 1, self.dir, self.rot_dir, self.rot_r)
        P_LF = self.mapping_function(self.phi_out2, 2, self.dir, self.rot_dir, self.rot_r)
        P_RH = self.mapping_function(self.phi_out3, 3, self.dir, self.rot_dir, self.rot_r)
        P_LH = self.mapping_function(self.phi_out4, 4, self.dir, self.rot_dir, self.rot_r)

        self.ef_ref_pos[1]["x"] = P_RF[0]-self.x_offset
        self.ef_ref_pos[1]["y"] = P_RF[1]-self.y_offset
        self.ef_ref_pos[1]["z"] = P_RF[2]
        self.ef_ref_pos[2]["x"] = P_LF[0]-self.x_offset
        self.ef_ref_pos[2]["y"] = P_LF[1]-self.y_offset
        self.ef_ref_pos[2]["z"] = P_LF[2]
        self.ef_ref_pos[3]["x"] = P_RH[0]-self.x_offset
        self.ef_ref_pos[3]["y"] = P_RH[1]-self.y_offset
        self.ef_ref_pos[3]["z"] = P_RH[2]
        self.ef_ref_pos[4]["x"] = P_LH[0]-self.x_offset
        self.ef_ref_pos[4]["y"] = P_LH[1]-self.y_offset
        self.ef_ref_pos[4]["z"] = P_LH[2]

        return self.ef_ref_pos