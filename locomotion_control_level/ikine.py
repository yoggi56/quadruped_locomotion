from math import *

class IKine(object):
    def __init__(self):
        self.bx = 0.257
        self.by = 0.093
        self.bz = 0.0
        self.L1 = 0.0905
        self.L2 = 0.202
        self.L3 = 0.17629

        self.theta_ref = [0]*12
    # config: m, x, o
    def calculate(self, p_ref, theta_cur=[0]*12, config="m"):
        self.theta_ref[0:3] = self.ikine_R1([p_ref[1]["x"], p_ref[1]["y"], p_ref[1]["z"]], theta_cur[0:3], config)
        self.theta_ref[3:6] = self.ikine_L1([p_ref[2]["x"], p_ref[2]["y"], p_ref[2]["z"]], config)
        self.theta_ref[6:9] = self.ikine_R2([p_ref[3]["x"], p_ref[3]["y"], p_ref[3]["z"]], theta_cur[6:9], config)
        self.theta_ref[9:12] = self.ikine_L2([p_ref[4]["x"], p_ref[4]["y"], p_ref[4]["z"]], config)
        return self.theta_ref

    def ikine_R1(self, p_ref, theta_cur, config="m"):
        if config == "m":
            sign1 = 1
            sign2 = -1
        elif config == "o":
            sign1 = -1
        elif config == "x":
            sign1 = 1
            sign2 = 1

        px = p_ref[2]
        py = -self.by - p_ref[1]
        pz = p_ref[0] - self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = (atan2(py, px) - atan2(self.L1, -sqrt(a2)))
        else:
            theta1 = (atan2(py, px)+2*pi - atan2(self.L1, -sqrt(a2)))
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0

        #print("LEG R1 THETA: {0}".format(theta_cur))
        if theta_cur[1] < -pi:
            theta2 = -sign2*pi + (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, -sqrt(a1))) % -pi
        else:
            theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, -sqrt(a1))) % -pi

        if abs(theta2 - theta_cur[1]) >= (pi-0.2):
            theta2 = theta2 + sign2*pi

        return [theta1, theta2, theta3]

    def ikine_R2(self, p_ref, theta_cur, config="m"):
        if config == "m":
            sign1 = -1
        elif config == "o":
            sign1 = -1
        elif config == "x":
            sign1 = 1

        px = p_ref[2]
        py = -self.by - p_ref[1]
        pz = p_ref[0] + self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = (atan2(py, px) - atan2(self.L1, -sqrt(a2)))
        else:
            theta1 = (atan2(py, px)+2*pi - atan2(self.L1, -sqrt(a2)))
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0
        
        #print("LEG R2 THETA: {0}".format(theta_cur))
        if theta_cur[1] < 0:
            if config == "m":
                theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) % pi
            elif config == "x":
                theta2 = -(pi - (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) % pi)
        else:
            theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) % -pi

        if abs(theta2 - theta_cur[1]) >= (pi-0.2):
            theta2 = theta2 % -pi

        return [-theta1, -theta2, -theta3]

    def ikine_L1(self, p_ref, config="m"):
        if config == "m":
            sign1 = -1
        elif config == "o":
            sign1 = 1
        elif config == "x":
            sign1 = -1

        px = p_ref[2]
        py = self.by - p_ref[1]
        pz = p_ref[0] - self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = (atan2(py, px) - atan2(self.L1, sqrt(a2))) - pi
        else:
            theta1 = (atan2(py, px)+2*pi - atan2(self.L1, sqrt(a2))) - pi
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0
        theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1)))# % -pi

        
        return [theta1, theta2, theta3]

    def ikine_L2(self, p_ref, config="m"):
        if config == "m":
            sign1 = -1
        elif config == "o":
            sign1 = -1
        elif config == "x":
            sign1 = 1

        px = p_ref[2]
        py = self.by - p_ref[1]
        pz = p_ref[0] + self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = -((atan2(py, px) - atan2(self.L1, sqrt(a2))) - pi)
        else:
            theta1 = -((atan2(py, px)+2*pi - atan2(self.L1, sqrt(a2))) - pi)
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0
        theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) - pi

        
        return [theta1, theta2, theta3]