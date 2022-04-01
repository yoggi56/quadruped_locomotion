import lcm
from msgs import hl_command_msg
from msgs import velocity
from msgs import measurment
from msgs import ef_point
from msgs import robot_ref_com


hl_state_channel = "HL_STATE"
hl_cmd_channel = "HL_COMMAND"
ll_cmd_channel = "LL_COMMAND"
ll_state_channel = "LL_STATE"

act_mode = 0
ef_mode=1
body_mode=2
robot_mode=3

vx = 0
vy = 1
vz = 2
wx = 3
wy = 4
wz = 5

DX = 0
DY = 1
DZ = 2
STEP_FREQ = 3
STEP_LENGHT = 4
STEP_HEIGHT = 5
GAIT_TYPE = 6

class LCM_Data_Exchange(object):
    def __init__(self) -> None:

        self.mode = 0
        self.start = False
        self.ref_angle_vel = [0]*12
        self.ef_refs = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                    2: {"x": 0.0, "y": 0.0, "z": 0.0},
                    3: {"x": 0.0, "y": 0.0, "z": 0.0},
                    4: {"x": 0.0, "y": 0.0, "z": 0.0},
                    }
        self.ref_body_vel = [0]*6
        self.robot_params = [0, 0, 0, 0, 0.0, 0.0, 0]
        self.cute_action = 0

        #self.ref_vel = [0]*3
        self.msg = hl_command_msg()
        self.ef_cur_msg = ef_point()
        self.lc_hl_cmd = lcm.LCM()
        self.lc_ll_state = lcm.LCM()
        self.theta_cur = [0]*12

    def ll_cmd_handler(self, channel, data):
        cmd_msg = robot_ref_com.decode(data)
        self.mode = cmd_msg.mode
        self.start = cmd_msg.start
        
        for i in range(12):
            self.ref_angle_vel[i] = cmd_msg.srv_check.ref_angle[i]

        for i in range(4):
            self.ef_refs[i+1]["x"] = cmd_msg.ref_ef_pt.x[i]
            self.ef_refs[i+1]["y"] = cmd_msg.ref_ef_pt.y[i]
            self.ef_refs[i+1]["z"] = cmd_msg.ref_ef_pt.z[i]

        self.ref_body_vel[0] = cmd_msg.body_vel.vx
        self.ref_body_vel[1] = cmd_msg.body_vel.vy
        self.ref_body_vel[2] = cmd_msg.body_vel.vz
        self.ref_body_vel[3] = cmd_msg.body_vel.wx
        self.ref_body_vel[4] = cmd_msg.body_vel.wy
        self.ref_body_vel[5] = cmd_msg.body_vel.wz

        self.robot_params[DX] = cmd_msg.robot_prms.robot_dx
        self.robot_params[DY] = cmd_msg.robot_prms.robot_dy
        self.robot_params[DZ] = cmd_msg.robot_prms.robot_dz
        self.robot_params[STEP_FREQ] = cmd_msg.robot_prms.robot_step_freq
        self.robot_params[STEP_HEIGHT] = cmd_msg.robot_prms.robot_step_height
        self.robot_params[STEP_LENGHT] = cmd_msg.robot_prms.robot_step_lenght
        self.robot_params[GAIT_TYPE] = cmd_msg.robot_prms.robot_gait_type

        self.cute_action = cmd_msg.cute_action_num

        #print(self.robot_params)
    
    def get_ef_refs(self):
        return self.ef_refs
    
    def get_ll_com(self):
        return self.mode, self.start, \
            self.ref_angle_vel, \
            self.ef_refs, self.ref_body_vel, \
            self.robot_params, self.cute_action

    # def ll_cmd_handler(self, channel, data):
    #     ll_cmd_msg = velocity.decode(data)
    #     self.ref_vel[0] = ll_cmd_msg.vx
    #     self.ref_vel[1] = ll_cmd_msg.vy
    #     self.ref_vel[2] = ll_cmd_msg.wz

    def hl_state_handler(self, channel, data):
        hl_state_msg = measurment.decode(data)
        for i in range(12):
            self.theta_cur[i] = hl_state_msg.act[i].position

    def get_theta_cur(self):
        return self.theta_cur

    def ll_cmd_thread(self):
        print("Thread th_ll_command started")
        lc = lcm.LCM()
        subscription = lc.subscribe(ll_cmd_channel, self.ll_cmd_handler)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass

    def hl_state_thread(self):
        print("Thread th_hl_meas started")
        lc = lcm.LCM()
        subscription = lc.subscribe(hl_state_channel, self.hl_state_handler)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass

    def send_hl_cmd(self, data):
        for i in range(12):
            self.msg.act[i].position = data[i][0]
            self.msg.act[i].velocity = data[i][1]
            self.msg.act[i].torque = data[i][2]
            self.msg.act[i].kp = data[i][3]
            self.msg.act[i].kd = data[i][4]
            #print(self.msg.act[i].kp)

        self.lc_hl_cmd.publish(hl_cmd_channel, self.msg.encode())
        #print(self.msg.act[1].position)

    def send_ll_state(self, data):
        for i in range(4):
            self.ef_cur_msg.x[i] = data[i+1]["x"]
            self.ef_cur_msg.y[i] = data[i+1]["y"]
            self.ef_cur_msg.z[i] = data[i+1]["z"]
        #print(self.ef_cur_msg.x, self.ef_cur_msg.y, self.ef_cur_msg.z)
        self.lc_ll_state.publish(ll_state_channel, self.ef_cur_msg.encode())