import lcm
from msgs import measurment
from msgs import ef_point
from threading import Thread
#import rospy

hl_state_channel = "HL_STATE"
ll_state_channel = "LL_STATE"

class LCM_Msg_Parse(object):
    def __init__(self):
        self.cur_angle = [0]*12
        self.cur_vel = [0]*12
        self.cur_torq = [0]*12
        self.cur_euler = [0]*3
        self.ef_curs = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                2: {"x": 0.0, "y": 0.0, "z": 0.0},
                3: {"x": 0.0, "y": 0.0, "z": 0.0},
                4: {"x": 0.0, "y": 0.0, "z": 0.0},
                }

        self.lc_hl_state = lcm.LCM()
        subscription = self.lc_hl_state.subscribe(hl_state_channel, self.hl_state_callback)

        th1 = Thread(target=self.check_hl_state_msg, args=())
        th1.start()

        self.lc_ll_state = lcm.LCM()
        subscription = self.lc_ll_state.subscribe(ll_state_channel, self.ll_state_callback)

        th2 = Thread(target=self.check_ll_state_msg, args=())
        th2.start()

    def check_ll_state_msg(self):
        while True:
            self.lc_ll_state.handle()

    def ll_state_callback(self, channel, data):
        msg = ef_point.decode(data)
        for i in range(4):
            self.ef_curs[i+1]["x"] = msg.x[i]
            self.ef_curs[i+1]["y"] = msg.y[i]
            self.ef_curs[i+1]["z"] = msg.z[i]

    def get_ll_state(self):
        return self.ef_curs

    def check_hl_state_msg(self):
        # rospy.loginfo("Yo!")
        while True:
            self.lc_hl_state.handle()

    def hl_state_callback(self, channel, data):
        msg = measurment.decode(data)
        for i in range(12):
            self.cur_angle[i] = msg.act[i].position
            self.cur_vel[i] = msg.act[i].velocity
            self.cur_torq[i] = msg.act[i].torque
        self.cur_euler[0] = msg.imu.euler.x
        self.cur_euler[1] = msg.imu.euler.y
        self.cur_euler[2] = msg.imu.euler.z

    def get_hl_state(self):
        return self.cur_angle, self.cur_vel, self.cur_torq, self.cur_euler
