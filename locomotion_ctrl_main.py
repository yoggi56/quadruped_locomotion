import configparser
from distutils.command.config import config
import time
from threading import Thread
from locomotion_control_level.lcm_data_exchange import *
from locomotion_control_level.ikine import IKine
from locomotion_control_level.fkine import FKine
from locomotion_control_level.body_moving import Body_Moving
from locomotion_control_level.cpg_gait_generator import CPGGaitGenerator
from math import *
from locomotion_control_level import locomotion_constants as consts


class Locomotion_Control(object):
    def __init__(self):
        print("--= Init Started =--")

        # init variables
        self.kp_sh = 0
        self.kp_hip = 0
        self.kp_knee = 0
        self.kd = consts.KD_INIT
        self.mode = 0
        self.start = False
        self.theta_ref_vel = [0]*12
        self.ef_ref_vel = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                    2: {"x": 0.0, "y": 0.0, "z": 0.0},
                    3: {"x": 0.0, "y": 0.0, "z": 0.0},
                    4: {"x": 0.0, "y": 0.0, "z": 0.0},
                    }
        self.ref_body_vel = [0]*6
        self.robot_params = [0, 0, 0, 0, 0.0, 0.0, 0]
        self.rot_r = 0
        
        self.ef_ref_pos = {1: {"x": consts.EF_F_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        2: {"x": consts.EF_F_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        3: {"x": -consts.EF_R_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        4: {"x": -consts.EF_R_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        }
        self.ef_ref_pos_for_body = {1: {"x": consts.EF_F_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        2: {"x": consts.EF_F_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        3: {"x": -consts.EF_R_INIT_X, "y": -consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        4: {"x": -consts.EF_R_INIT_X, "y": consts.EF_INIT_Y, "z": -consts.ROBOT_HEIGHT},
                        }
        self.ef_cur_pos = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                        2: {"x": 0.0, "y": 0.0, "z": 0.0},
                        3: {"x": 0.0, "y": 0.0, "z": 0.0},
                        4: {"x": 0.0, "y": 0.0, "z": 0.0},
                        }
        self.theta_cur = [0]*12
        self.theta_ref = [0]*12
        self.cute_action_num = 0
        self.hl_cmd_data = [[0 for x in range(5)] for y in range(12)]
        for i in range(12):
            self.hl_cmd_data[i][1] = 0
            self.hl_cmd_data[i][2] = 0
            self.hl_cmd_data[i][3] = 0
            self.hl_cmd_data[i][4] = self.kd

        # reading config file for interaction with hardware level
        ll_hl_config = configparser.ConfigParser()
        ll_hl_config.sections()
        ll_hl_config.read("./configs/LL_HL_Config.ini")
        self.ll_hl_freq = float(ll_hl_config['DATA_TRANSFER']['Frequency'])

        # reading config file for interaction with navigation level
        nl_ll_config = configparser.ConfigParser()
        nl_ll_config.sections()
        nl_ll_config.read("./configs/NL_LL_Config.ini")
        self.nl_ll_freq = float(nl_ll_config['DATA_TRANSFER']['Frequency'])

        # count periods of exchange
        self.ll_hl_period = 1.0/self.ll_hl_freq
        self.nl_ll_period = 1.0/self.nl_ll_freq

        print("INFO: Messaging frequency for Hardware Level: ",
              self.ll_hl_freq, ". Period: ", self.ll_hl_period)
        print("INFO: Messaging frequency for Navigation Level: ",
              self.nl_ll_freq, ". Period: ", self.nl_ll_period)

        self.lcm_exch = LCM_Data_Exchange()
        # start getting messages threads
        th_ll_command = Thread(target=self.lcm_exch.ll_cmd_thread, args=())
        th_hl_meas = Thread(target=self.lcm_exch.hl_state_thread, args=())
        th_ll_command.start()
        th_hl_meas.start()

        # init control algorithms
        self.ik = IKine()
        self.fk = FKine()
        self.b_mov = Body_Moving()
        self.cpg = CPGGaitGenerator()
        if consts.ISREALROBOT:
            self.x_offset = consts.COG_OFFSET_X_HARD
            self.y_offset = consts.COG_OFFSET_Y_HARD
        else:
            self.x_offset = consts.COG_OFFSET_X_SIM#0.02#0.028#0.035
            self.y_offset = consts.COG_OFFSET_Y_SIM
        self.cpg.set_static_params(ampl=1, beta=consts.DUTY_FACTOR, alpha=0.25, lamb=1, a=1, x_offset=self.x_offset, y_offset=self.y_offset)
        self.cpg.set_gait_params(w_sw=0, phi="walk", Hg=0.08, Ls=0.12, dir=0, rot_dir=0, rot_r=consts.ROT_R)
        self.cpg.init_integrator(self.ll_hl_period)

        # variables for state machine
        self.cur_state = 0
        self.u1, self.u2, self.u3, self.u4 = 0,0,0,0

        print("--= Init Finished =--")

    def get_ll_hl_period(self):
        return self.ll_hl_period

    def send_theta_ref(self, theta_ref):
        for j in range(12):
            self.hl_cmd_data[j][0] = theta_ref[j]

        self.hl_cmd_data[3-1][2] = -7.5
        self.hl_cmd_data[6-1][2] = 7.5
        self.hl_cmd_data[9-1][2] = -7.5
        self.hl_cmd_data[12-1][2] = 7.5

        self.lcm_exch.send_hl_cmd(self.hl_cmd_data)

    def get_theta_cur(self):
        theta = self.lcm_exch.get_theta_cur()
        return theta

    def do_nothing(self):
        self.kp_sh = 0
        self.kp_hip = 0
        self.kp_knee = 0
        #self.kd = 0
        #self.kd += kd_inc   
        for k in range(4):           
            self.hl_cmd_data[3*k][3] = self.kp_sh
            self.hl_cmd_data[3*k+1][3] = self.kp_hip
            self.hl_cmd_data[3*k+2][3] = self.kp_knee
        self.lcm_exch.send_hl_cmd(self.hl_cmd_data)
        #print("State: Do Nothing!")


    def kpkd_inc(self, kp_inc):
        # self.kp_sh = 0
        # self.kp_hip = 0
        # self.kp_knee = 0
        if (self.kp_sh < consts.KP_SH_INIT):
            self.kp_sh += kp_inc[0]

        if (self.kp_hip < consts.KP_HIP_INIT):    
            self.kp_hip += kp_inc[1]

        if (self.kp_knee < consts.KP_KNEE_INIT):
            self.kp_knee += kp_inc[2]

        #self.kd += kd_inc   
        for k in range(4):           
            self.hl_cmd_data[3*k][3] = self.kp_sh
            self.hl_cmd_data[3*k+1][3] = self.kp_hip
            self.hl_cmd_data[3*k+2][3] = self.kp_knee
        self.kd = consts.KD_INIT

    def get_up(self):
        print("State: Get Up")

        kp_sh_inc = consts.KP_SH_INIT/consts.KPKD_DEN
        kp_hip_inc = consts.KP_HIP_INIT/consts.KPKD_DEN
        kp_knee_inc = consts.KP_KNEE_INIT/consts.KPKD_DEN
        self.kd = consts.KD_INIT
        for i in range(12):
            self.hl_cmd_data[i][4] = self.kd
        self.kpkd_inc([kp_sh_inc, kp_hip_inc, kp_knee_inc])

        self.theta_ref = self.get_theta_cur()
        
        step = consts.BODY_MOTION_STEP
        step_angle = consts.BODY_MOTION_STEP*200
        # start going up to 10 cm
        self.ef_ref_pos = self.fk.calculate(self.theta_ref)
        print("Start theta: {0}".format(self.theta_ref))
        print("Start pos: {0}".format(self.ef_ref_pos))
        print("Start ik desired theta: {0}".format(self.ik.calculate(self.ef_ref_pos, self.theta_ref)))
        self.send_theta_ref(self.theta_ref)
        
        # turn knee
        while(True):
            self.theta_cur = self.get_theta_cur()
            e_R1_sh = -2.8 - self.theta_cur[1]
            e_L1_sh = 3 - self.theta_cur[4]
            e_R2_sh = 0.17 - self.theta_cur[7]
            e_L2_sh = -0.17 - self.theta_cur[10]
            e_R1_hip = 2.5 - self.theta_cur[2]
            e_L1_hip = 2.5 - self.theta_cur[5]
            e_R2_hip = 2.5 - self.theta_cur[8]
            e_L2_hip = 2.5 - self.theta_cur[11]
            if abs(e_R1_sh) > 0.002:
                if e_R1_sh > 0:
                    self.theta_ref[1] += step_angle
                else:
                    self.theta_ref[1] -= step_angle
            if abs(e_R1_hip) > 0.002:
                if e_R1_hip > 0:
                    self.theta_ref[2] += step_angle
                else:
                    self.theta_ref[2] -= step_angle
            
            if abs(e_L1_sh) > 0.002:
                if e_L1_sh > 0:
                    self.theta_ref[4] += step_angle
                else:
                    self.theta_ref[4] -= step_angle
            if abs(e_L1_hip) > 0.002:
                if e_L1_hip > 0:
                    self.theta_ref[5] -= step_angle
                else:
                    self.theta_ref[5] += step_angle

            if abs(e_R2_sh) > 0.002:
                if e_R2_sh > 0:
                    self.theta_ref[7] += step_angle
                else:
                    self.theta_ref[7] -= step_angle
            if abs(e_R2_hip) > 0.002:
                if e_R2_hip > 0:
                    self.theta_ref[8] += step_angle
                else:
                    self.theta_ref[8] -= step_angle

            if abs(e_L2_sh) > 0.002:
                if e_L2_sh > 0:
                    self.theta_ref[10] += step_angle
                else:
                    self.theta_ref[10] -= step_angle
            if abs(e_L2_hip) > 0.002:
                if e_L2_hip > 0:
                    self.theta_ref[11] -= step_angle
                else:
                    self.theta_ref[11] += step_angle
            

            if abs(e_R1_sh) <= 0.02 and abs(e_R1_hip) <= 0.02:
                break 

            self.kpkd_inc([kp_sh_inc, kp_hip_inc, kp_knee_inc])
            self.send_theta_ref(self.theta_ref)
            time.sleep(self.ll_hl_period)

        # turn all hips
        while(True):
            self.theta_cur = self.get_theta_cur()
            e_R1 = 0 - self.theta_cur[0]
            e_L1 = 0 - self.theta_cur[3]
            e_R2 = 0 - self.theta_cur[6]
            e_L2 = 0 - self.theta_cur[9]
            if abs(e_R1) > 0.002:
                if e_R1 > 0:
                    self.theta_ref[0] += step_angle
                else:
                    self.theta_ref[0] -= step_angle
            if abs(e_L1) > 0.002:
                if e_L1 > 0:
                    self.theta_ref[3] += step_angle
                else:
                    self.theta_ref[3] -= step_angle
            if abs(e_R2) > 0.002:
                if e_R2 > 0:
                    self.theta_ref[6] += step_angle
                else:
                    self.theta_ref[6] -= step_angle
            if abs(e_L2) > 0.002:
                if e_L2 > 0:
                    self.theta_ref[9] += step_angle
                else:
                    self.theta_ref[9] -= step_angle

            if  abs(e_R1) <= 0.002 and abs(e_R2) <= 0.005 and abs(e_L1) <= 0.002 and abs(e_L2) <= 0.002:
                break

            self.kpkd_inc([kp_sh_inc, kp_hip_inc, kp_knee_inc])
            self.send_theta_ref(self.theta_ref)
            time.sleep(self.ll_hl_period)
        
        self.theta_cur = self.get_theta_cur()
        self.ef_ref_pos = self.fk.calculate(self.theta_cur)

        # move feet to XY start pos
        while(True):
            if self.ef_ref_pos[1]["x"] < consts.EF_F_INIT_X:
                self.ef_ref_pos[1]["x"] += step
            if self.ef_ref_pos[1]["y"] < -consts.EF_INIT_Y:
                self.ef_ref_pos[1]["y"] += step

            if self.ef_ref_pos[2]["x"] < consts.EF_F_INIT_X:
                self.ef_ref_pos[2]["x"] += step
            if self.ef_ref_pos[2]["y"] > consts.EF_INIT_Y:
                self.ef_ref_pos[2]["y"] -= step

            if self.ef_ref_pos[3]["x"] < -consts.EF_R_INIT_X:
                self.ef_ref_pos[3]["x"] += step
            if self.ef_ref_pos[3]["y"] < -consts.EF_INIT_Y:
                self.ef_ref_pos[3]["y"] += step

            if self.ef_ref_pos[4]["x"] < -consts.EF_R_INIT_X:
                self.ef_ref_pos[4]["x"] += step
            if self.ef_ref_pos[4]["y"] > consts.EF_INIT_Y:
                self.ef_ref_pos[4]["y"] -= step

            print(self.ef_ref_pos[1]["y"])

            if (self.ef_ref_pos[1]["x"] >= consts.EF_F_INIT_X and
                self.ef_ref_pos[1]["y"] >= -consts.EF_INIT_Y and
                self.ef_ref_pos[2]["x"] >= consts.EF_F_INIT_X and
                self.ef_ref_pos[2]["y"] <= consts.EF_INIT_Y and
                self.ef_ref_pos[3]["x"] >= -consts.EF_R_INIT_X and
                self.ef_ref_pos[3]["y"] >= -consts.EF_INIT_Y and
                self.ef_ref_pos[4]["x"] >= -consts.EF_R_INIT_X and
                self.ef_ref_pos[4]["y"] <= consts.EF_INIT_Y):
                print("stop")
                break

            self.theta_ref = self.ik.calculate(self.ef_ref_pos)
            self.send_theta_ref(self.theta_ref)
            time.sleep(self.ll_hl_period)

        # stand up
        if self.ef_ref_pos[1]["z"] > -consts.ROBOT_HEIGHT:
            while ((self.ef_ref_pos[1]["z"] > -consts.ROBOT_HEIGHT) and
                    (self.ef_ref_pos[2]["z"] > -consts.ROBOT_HEIGHT) and
                    (self.ef_ref_pos[3]["z"] > -consts.ROBOT_HEIGHT) and
                    (self.ef_ref_pos[4]["z"] > -consts.ROBOT_HEIGHT)):
                self.ef_ref_pos = self.b_mov.move([0,0,step], [0,0,0], self.ef_ref_pos)
                self.theta_ref = self.ik.calculate(self.ef_ref_pos, self.theta_ref)
                # print("theta_ref: {0}".format(self.theta_ref))
                # print("ef_ref_pos: {0}".format(self.ef_ref_pos))
                
                self.kpkd_inc([kp_sh_inc, kp_hip_inc, kp_knee_inc])
                self.send_theta_ref(self.theta_ref)
                time.sleep(self.ll_hl_period)

        #time.sleep(5)
        

    def control(self):
        #print("State: Control")
        print("Mode: {0}".format(self.mode))
        if self.mode == 0:
            self.loc_ctrl_mode()
        elif self.mode == 1:
            self.ikine_check_mode()
        elif self.mode == 2:
            self.body_moving_mode()

        # calulate ikine
        self.theta_ref = self.ik.calculate(self.ef_ref_pos)
        # self.theta_ref[0] = 0.256
        # self.theta_ref[3] = -0.256
        # self.theta_ref[6] = -0.256
        # self.theta_ref[9] = 0.256
        self.send_theta_ref(self.theta_ref)
        

    def loc_ctrl_mode(self):
        gait_dir = self.cpg.from_dir_to_angle(self.robot_params[DX], self.robot_params[DY])
        
        # get rotating values
        if self.robot_params[DZ] > 0:
            gait_rot_dir = -1
        elif self.robot_params[DZ] < 0:
            gait_rot_dir = 1
        else:
            gait_rot_dir = 0

        if self.robot_params[DZ] == 0:
            self.rot_r = 0
        else:
            self.rot_r = 1 - abs(self.robot_params[DZ])

        self.cpg.set_gait_params(w_sw=self.robot_params[STEP_FREQ], 
                                phi=self.robot_params[GAIT_TYPE], 
                                Hg=self.robot_params[STEP_HEIGHT], 
                                Ls=self.robot_params[STEP_LENGHT], 
                                dir=gait_dir, 
                                rot_dir=gait_rot_dir, 
                                rot_r=self.rot_r)
        self.ef_ref_pos = self.cpg.step()

    def servo_check_mode(self):
        pass

    def body_moving_mode(self):
        self.ef_ref_pos = self.b_mov.move(self.ref_body_vel[0:3], self.ref_body_vel[3:6], self.ef_ref_pos)

    def ikine_check_mode(self):
        for i in range(1, 5):
            self.ef_ref_pos[i]["x"] += self.ef_ref_vel[i]["x"]
            self.ef_ref_pos[i]["y"] += self.ef_ref_vel[i]["y"]
            self.ef_ref_pos[i]["z"] += self.ef_ref_vel[i]["z"]

    def cute_action_mode(self):
        pass

    def go_sleep(self):
        print("State: Go Sleep")
        time.sleep(5)
        

    def step(self):
        # get ll commands signals
        self.mode, self.start, self.theta_ref_vel, self.ef_ref_vel, self.ref_body_vel, self.robot_params, self.cute_action_num = self.lcm_exch.get_ll_com()

        # ---------------------------
        # state machine
        # cur_state:
        #       1 - relax
        #       2 - get_up
        #       3 - control
        #       4 - go_sleep
        self.u1 = self.start
        self.u3 = self.start

        if self.cur_state == 0 and self.u1:
            self.cur_state = 1
            self.u1 = 0
        elif self.cur_state == 1 and self.u2:
            self.cur_state = 2
            self.u2 = 0
        elif self.cur_state == 2 and self.u3:
            self.cur_state = 3
            self.u3 = 0
        elif self.cur_state == 3 and self.u4:
            self.cur_state = 0
            self.u4 = 0

        if self.cur_state == 0:
            self.do_nothing()
        elif self.cur_state == 1:
            self.get_up()
            self.u2 = 1
        elif self.cur_state == 2:
            self.control()
        elif self.cur_state == 3:
            self.go_sleep()
            self.u4 = 1
        # ----------------------------
        # get theta cur and calculate fkine
        self.theta_cur = self.get_theta_cur()
        self.ef_cur_pos = self.fk.calculate(self.theta_cur)
        self.lcm_exch.send_ll_state(self.ef_cur_pos)
        # ----------------------------

def main():
    loc_ctrl = Locomotion_Control()
    #loc_ctrl.get_up()
    step_period = loc_ctrl.get_ll_hl_period()
    while 1:
        start_time = time.time()
        loc_ctrl.step()
        elapced_time = time.time() - start_time
        wait_time = step_period - elapced_time
        if wait_time > 0:
            time.sleep(wait_time)
        # print(1/elapced_time)


if __name__ == '__main__':
    main()
