from math import *

class FKine(object):
    def __init__(self):        
        self.bx = 0.257
        self.by = 0.093
        self.bz = 0.0
        self.L1 = 0.0905
        self.L2 = 0.202
        self.L3 = 0.17629

        self.ef_curs = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                        2: {"x": 0.0, "y": 0.0, "z": 0.0},
                        3: {"x": 0.0, "y": 0.0, "z": 0.0},
                        4: {"x": 0.0, "y": 0.0, "z": 0.0},
                        }
    def fkine_R1(self, theta):
        ef_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        ef_pos["x"] = self.bx + self.L3*cos(theta[1]+theta[2]) + self.L2*cos(theta[1])
        ef_pos["y"] = -self.by - self.L3*sin(theta[0])*sin(theta[1]+theta[2]) - self.L1*cos(theta[0]) - self.L2*sin(theta[0])*sin(theta[1])
        ef_pos["z"] = self.L3*cos(theta[0])*sin(theta[1]+theta[2]) - self.L1*sin(theta[0]) + self.L2*cos(theta[0])*sin(theta[1])
        return ef_pos

    def fkine_R2(self, theta):
        ef_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        ef_pos["x"] = -(self.bx + self.L3*cos(theta[1]+theta[2]) + self.L2*cos(theta[1]))
        ef_pos["y"] = -self.by - self.L3*sin(theta[0])*sin(theta[1]+theta[2]) - self.L1*cos(theta[0]) - self.L2*sin(theta[0])*sin(theta[1])
        ef_pos["z"] = -(self.L3*cos(theta[0])*sin(theta[1]+theta[2]) - self.L1*sin(theta[0]) + self.L2*cos(theta[0])*sin(theta[1]))
        #print(ef_pos["z"])

        return ef_pos

    def fkine_L1(self, theta):
        ef_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        ef_pos["x"] = self.bx + self.L3*cos(theta[1]+theta[2]) + self.L2*cos(theta[1])
        ef_pos["y"] = -(-self.by - self.L3*sin(theta[0])*sin(theta[1]+theta[2]) - self.L1*cos(theta[0]) - self.L2*sin(theta[0])*sin(theta[1]))
        ef_pos["z"] = -(self.L3*cos(theta[0])*sin(theta[1]+theta[2]) - self.L1*sin(theta[0]) + self.L2*cos(theta[0])*sin(theta[1]))
        
        return ef_pos

    def fkine_L2(self, theta):
        ef_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        ef_pos["x"] = -(self.bx + self.L3*cos(theta[1]+theta[2]) + self.L2*cos(theta[1]))
        ef_pos["y"] = -(-self.by - self.L3*sin(theta[0])*sin(theta[1]+theta[2]) - self.L1*cos(theta[0]) - self.L2*sin(theta[0])*sin(theta[1]))
        ef_pos["z"] = self.L3*cos(theta[0])*sin(theta[1]+theta[2]) - self.L1*sin(theta[0]) + self.L2*cos(theta[0])*sin(theta[1])
        return ef_pos

    def calculate(self, theta):
        self.ef_curs[1] = self.fkine_R1(theta[0:3])
        self.ef_curs[2] = self.fkine_L1(theta[3:6])
        self.ef_curs[3] = self.fkine_R2(theta[6:9])
        self.ef_curs[4] = self.fkine_L2(theta[9:])
        return self.ef_curs