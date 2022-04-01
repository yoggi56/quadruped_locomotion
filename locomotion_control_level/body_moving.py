from math import *
from locomotion_control_level import locomotion_constants as consts

class Body_Moving(object):
    def __init__(self) -> None:
        self.ef_ref_pos = {1: {"x": consts.EF_F_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        2: {"x": consts.EF_F_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        3: {"x": -consts.EF_R_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        4: {"x": -consts.EF_R_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        }

    def move(self, translation, rotation, ef_pos):#, active_legs=[1,1,1,1]):
        self.ef_ref_pos = ef_pos
        for i in range(1, 5):
            # if active_legs[i-1] == 1:
            self.ef_ref_pos[i]["x"] -= translation[0]
            self.ef_ref_pos[i]["y"] -= translation[1]
            self.ef_ref_pos[i]["z"] -= translation[2]

            self.ef_ref_pos[i]["x"] = self.ef_ref_pos[i]["x"]*cos(rotation[2]) - self.ef_ref_pos[i]["y"]*sin(rotation[2])
            self.ef_ref_pos[i]["y"] = self.ef_ref_pos[i]["x"]*sin(rotation[2]) + self.ef_ref_pos[i]["y"]*cos(rotation[2])     

            self.ef_ref_pos[i]["y"] = self.ef_ref_pos[i]["y"]*cos(rotation[0]) - self.ef_ref_pos[i]["z"]*sin(rotation[0])
            self.ef_ref_pos[i]["z"] = self.ef_ref_pos[i]["y"]*sin(rotation[0]) + self.ef_ref_pos[i]["z"]*cos(rotation[0])

            self.ef_ref_pos[i]["x"] = self.ef_ref_pos[i]["x"]*cos(rotation[1]) + self.ef_ref_pos[i]["z"]*sin(rotation[1])
            self.ef_ref_pos[i]["z"] = -self.ef_ref_pos[i]["x"]*sin(rotation[1]) + self.ef_ref_pos[i]["z"]*cos(rotation[1])       

        return self.ef_ref_pos
