from cgitb import reset
from sys import flags
import pybullet as p
import time
from threading import Thread
import pybullet_data
import lcm
from msgs import hl_command_msg
from msgs import measurment
import configparser



class Hardware_Level_Sim(object):
    def __init__(self):
        self.ref_pos = [0]*12
        self.init_ref_pos = [0]*12
        self.ref_vel = [0]*12
        self.ref_torq = [0]*12
        self.kp = [0]*12
        self.kd = [0]*12
        self.joint_dir = [-1]*12
        self.joint_offset = [0]*12
        
        self.cmd_channel = "HL_COMMAND"
        self.hl_state_channel = "HL_STATE"

        self.read_config()

        # init LCM thread
        self.cmd_th = Thread(target=self.get_cmd, args=())
        self.cmd_th.start()

        # init LCM for states
        self.state_msg = measurment()
        self.lc_state = lcm.LCM()

        # init pybullet
        # or p.DIRECT for non-graphical version
        self.physicsClient = p.connect(p.GUI)
        self.init_simulation()
        self.sim_it = 0

        # debug parameters
        self.reset_id = p.addUserDebugParameter("Reset",1,0,1)
        self.reset_clicked_prev = p.readUserDebugParameter(self.reset_id)
        self.reset_clicked = p.readUserDebugParameter(self.reset_id)
    def set_start_posisition(self):
        # self.init_ref_pos[0] = -0.0
        # self.init_ref_pos[1] = -0.0
        # self.init_ref_pos[2] = 0.0
        # self.init_ref_pos[3] = 0.
        # self.init_ref_pos[4] = 0.0
        # self.init_ref_pos[5] = -0.0
        # self.init_ref_pos[6] = 0.0
        # self.init_ref_pos[7] = 0.0
        # self.init_ref_pos[8] = 0.0
        # self.init_ref_pos[9] = -0.0
        # self.init_ref_pos[10] = -0.0
        # self.init_ref_pos[11] = -0.0
        self.init_ref_pos[0] = -0.97
        self.init_ref_pos[1] = -2
        self.init_ref_pos[2] = 1.53
        self.init_ref_pos[3] = 0.97
        self.init_ref_pos[4] = 2
        self.init_ref_pos[5] = -1.53
        self.init_ref_pos[6] = 0.97
        self.init_ref_pos[7] = 0.97
        self.init_ref_pos[8] = 1.53
        self.init_ref_pos[9] = -0.97
        self.init_ref_pos[10] = -0.97
        self.init_ref_pos[11] = -1.53
    def init_simulation(self):
        self.jointIds = []
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.sim_period)
        p.setRealTimeSimulation(0)
        p.setPhysicsEngineParameter(enableConeFriction=0)
        self.planeId = p.loadURDF("plane.urdf")
        self.startPos = [0, 0, 0.25]
        self.startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = p.loadURDF("urdf/quadrobot.urdf", self.startPos, self.startOrientation, useFixedBase=False,
                                flags=p.URDF_USE_INERTIA_FROM_FILE+p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

        for j in range(p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, j, linearDamping=1.0, angularDamping=1.0)
            info = p.getJointInfo(self.robot, j)
            # print(info)
            #jointName = info[1]
            jointType = info[2]
            if (jointType == p.JOINT_REVOLUTE):
                self.jointIds.append(j)

        # set start position
        self.set_start_posisition()
        # self.init_ref_pos[0] = -0.17
        # self.init_ref_pos[1] = -3
        # self.init_ref_pos[2] = 2.53
        # self.init_ref_pos[3] = 0.17
        # self.init_ref_pos[4] = 3
        # self.init_ref_pos[5] = -2.53
        # self.init_ref_pos[6] = 0.17
        # self.init_ref_pos[7] = 0.14
        # self.init_ref_pos[8] = 2.53
        # self.init_ref_pos[9] = -0.17
        # self.init_ref_pos[10] = -0.14
        # self.init_ref_pos[11] = -2.53
        # for i in range(len(self.jointIds)):
        #     p.setJointMotorControl2(self.robot, self.jointIds[i], p.POSITION_CONTROL,
        #                             targetPosition=self.joint_dir[i]*self.ref_pos[i]-self.joint_offset[i],
        #                             maxVelocity=150.0, force=18)


    def loop(self):
        self.sim_it += 1
        self.reset_clicked = p.readUserDebugParameter(self.reset_id)
        if self.reset_clicked != self.reset_clicked_prev:
            p.resetSimulation()
            self.init_simulation()
            self.sim_it = 0

        if self.sim_it < 90:
            for i in range(len(self.jointIds)):
                p.setJointMotorControl2(self.robot, self.jointIds[i], p.POSITION_CONTROL,
                                        targetPosition=self.joint_dir[i]*self.init_ref_pos[i]-self.joint_offset[i],
                                        maxVelocity=15.0, force=18)
        else:
            #print(self.kp[0])
            if self.kp[0] != 0:
                force = 18
            else:
                force = 0
            for i in range(len(self.jointIds)):
                p.setJointMotorControl2(self.robot, self.jointIds[i], p.POSITION_CONTROL,
                                        targetPosition=self.joint_dir[i]*self.ref_pos[i]-self.joint_offset[i],
                                        maxVelocity=15.0, force=force)#, positionGain=self.kp[i], velocityGain=self.kd[i])#
        self.get_state()
        self.lc_state.publish(self.hl_state_channel, self.state_msg.encode())
        # print(self.state_msg.act[0].position)

        basePos, baseOrn = p.getBasePositionAndOrientation(self.robot) # Get model position
	    
        p.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=20, cameraPitch=-40, cameraTargetPosition=basePos) # fix camera onto model

        p.stepSimulation()
        self.reset_clicked_prev = self.reset_clicked
        #time.sleep(1./240.)


    def read_config(self):
        # reading config file for interaction with hardware level
        ll_hl_config = configparser.ConfigParser()
        ll_hl_config.sections()
        ll_hl_config.read("./configs/LL_HL_Config.ini")
        sim_freq = float(ll_hl_config['DATA_TRANSFER']['Frequency'])
        self.sim_period = 1.0/sim_freq

        self.joint_dir[0] = -float(ll_hl_config['ACTUATORS']['RotDir1'])*2+1
        self.joint_dir[1] = -float(ll_hl_config['ACTUATORS']['RotDir2'])*2+1
        self.joint_dir[2] = -float(ll_hl_config['ACTUATORS']['RotDir3'])*2+1
        self.joint_dir[3] = -float(ll_hl_config['ACTUATORS']['RotDir4'])*2+1
        self.joint_dir[4] = -float(ll_hl_config['ACTUATORS']['RotDir5'])*2+1
        self.joint_dir[5] = -float(ll_hl_config['ACTUATORS']['RotDir6'])*2+1
        self.joint_dir[6] = -float(ll_hl_config['ACTUATORS']['RotDir7'])*2+1
        self.joint_dir[7] = -float(ll_hl_config['ACTUATORS']['RotDir8'])*2+1
        self.joint_dir[8] = -float(ll_hl_config['ACTUATORS']['RotDir9'])*2+1
        self.joint_dir[9] = -float(ll_hl_config['ACTUATORS']['RotDir10'])*2+1
        self.joint_dir[10] = -float(ll_hl_config['ACTUATORS']['RotDir11'])*2+1
        self.joint_dir[11] = -float(ll_hl_config['ACTUATORS']['RotDir11'])*2+1

        self.joint_offset[0] = float(ll_hl_config['ACTUATORS']['AngleOffset1'])
        self.joint_offset[1] = float(ll_hl_config['ACTUATORS']['AngleOffset2'])+1.57
        self.joint_offset[2] = float(ll_hl_config['ACTUATORS']['AngleOffset3'])
        self.joint_offset[3] = float(ll_hl_config['ACTUATORS']['AngleOffset4'])
        self.joint_offset[4] = float(ll_hl_config['ACTUATORS']['AngleOffset5'])-1.57
        self.joint_offset[5] = float(ll_hl_config['ACTUATORS']['AngleOffset6'])
        self.joint_offset[6] = float(ll_hl_config['ACTUATORS']['AngleOffset7'])
        self.joint_offset[7] = float(ll_hl_config['ACTUATORS']['AngleOffset8'])-1.57
        self.joint_offset[8] = float(ll_hl_config['ACTUATORS']['AngleOffset9'])
        self.joint_offset[9] = float(ll_hl_config['ACTUATORS']['AngleOffset10'])
        self.joint_offset[10] = float(ll_hl_config['ACTUATORS']['AngleOffset11'])+1.57
        self.joint_offset[11] = float(ll_hl_config['ACTUATORS']['AngleOffset12'])


    def get_cmd(self):
        # init LCM
        lc = lcm.LCM()
        subscription = lc.subscribe(self.cmd_channel, self.cmd_handler)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass


    def cmd_handler(self, channel, data):
        msg = hl_command_msg.decode(data)
        # print("Received message:")
        for i in range(12):
            self.ref_pos[i] = msg.act[i].position
            self.ref_vel[i] = msg.act[i].velocity
            self.ref_torq[i] = msg.act[i].torque
            self.kp[i] = msg.act[i].kp
            self.kd[i] = msg.act[i].kd


    def map_ref_pos(got_ref_pos):
        pb_ref_pos = [0]*16
        pb_ref_pos[0] = 0.0
        pb_ref_pos[1] = got_ref_pos[0]
        pb_ref_pos[2] = got_ref_pos[1]
        pb_ref_pos[3] = got_ref_pos[2]

        pb_ref_pos[4] = 0.0
        pb_ref_pos[5] = got_ref_pos[3]
        pb_ref_pos[6] = got_ref_pos[4]
        pb_ref_pos[7] = got_ref_pos[5]

        pb_ref_pos[8] = 0.0
        pb_ref_pos[9] = got_ref_pos[6]
        pb_ref_pos[10] = got_ref_pos[7]
        pb_ref_pos[11] = got_ref_pos[8]

        pb_ref_pos[12] = 0.0
        pb_ref_pos[13] = got_ref_pos[9]
        pb_ref_pos[14] = got_ref_pos[10]
        pb_ref_pos[15] = got_ref_pos[11]

        return pb_ref_pos


    def map_state_msg(self, got_cur_pos, got_cur_vel, got_cur_torq):
        msg_cur_pos = [0]*12
        msg_cur_vel = [0]*12
        msg_cur_torq = [0]*12
        # --
        msg_cur_pos[0] = self.joint_dir[0]*got_cur_pos[1]-self.joint_offset[0]
        msg_cur_pos[1] = self.joint_dir[1]*got_cur_pos[2]-self.joint_offset[1]
        msg_cur_pos[2] = self.joint_dir[2]*got_cur_pos[3]-self.joint_offset[2]

        msg_cur_pos[3] = self.joint_dir[3]*got_cur_pos[5]-self.joint_offset[3]
        msg_cur_pos[4] = self.joint_dir[4]*got_cur_pos[6]-self.joint_offset[4]
        msg_cur_pos[5] = self.joint_dir[5]*got_cur_pos[7]-self.joint_offset[5]

        msg_cur_pos[6] = self.joint_dir[6]*got_cur_pos[9]-self.joint_offset[6]
        msg_cur_pos[7] = self.joint_dir[7]*got_cur_pos[10]-self.joint_offset[7]
        msg_cur_pos[8] = self.joint_dir[8]*got_cur_pos[11]-self.joint_offset[8]

        msg_cur_pos[9] = self.joint_dir[9]*got_cur_pos[13]-self.joint_offset[9]
        msg_cur_pos[10] = self.joint_dir[10]*got_cur_pos[14]-self.joint_offset[10]
        msg_cur_pos[11] = self.joint_dir[11]*got_cur_pos[15]-self.joint_offset[11]
        # --
        msg_cur_vel[0] = self.joint_dir[0]*got_cur_vel[1]
        msg_cur_vel[1] = self.joint_dir[1]*got_cur_vel[2]
        msg_cur_vel[2] = self.joint_dir[2]*got_cur_vel[3]

        msg_cur_vel[3] = self.joint_dir[3]*got_cur_vel[5]
        msg_cur_vel[4] = self.joint_dir[4]*got_cur_vel[6]
        msg_cur_vel[5] = self.joint_dir[5]*got_cur_vel[7]

        msg_cur_vel[6] = self.joint_dir[6]*got_cur_vel[9]
        msg_cur_vel[7] = self.joint_dir[7]*got_cur_vel[10]
        msg_cur_vel[8] = self.joint_dir[8]*got_cur_vel[11]

        msg_cur_vel[9] = self.joint_dir[9]*got_cur_vel[13]
        msg_cur_vel[10] = self.joint_dir[10]*got_cur_vel[14]
        msg_cur_vel[11] = self.joint_dir[11]*got_cur_vel[15]
        # --
        msg_cur_torq[0] = self.joint_dir[0]*got_cur_torq[1]
        msg_cur_torq[1] = self.joint_dir[1]*got_cur_torq[2]
        msg_cur_torq[2] = self.joint_dir[2]*got_cur_torq[3]

        msg_cur_torq[3] = self.joint_dir[3]*got_cur_torq[5]
        msg_cur_torq[4] = self.joint_dir[4]*got_cur_torq[6]
        msg_cur_torq[5] = self.joint_dir[5]*got_cur_torq[7]

        msg_cur_torq[6] = self.joint_dir[6]*got_cur_torq[9]
        msg_cur_torq[7] = self.joint_dir[7]*got_cur_torq[10]
        msg_cur_torq[8] = self.joint_dir[8]*got_cur_torq[11]

        msg_cur_torq[9] = self.joint_dir[9]*got_cur_torq[13]
        msg_cur_torq[10] = self.joint_dir[10]*got_cur_torq[14]
        msg_cur_torq[11] = self.joint_dir[11]*got_cur_torq[15]

        for i in range(12):
            self.state_msg.act[i].position = msg_cur_pos[i]
            self.state_msg.act[i].velocity = msg_cur_vel[i]
            self.state_msg.act[i].torque = msg_cur_torq[i]


    def get_state(self):
        states = p.getJointStates(self.robot, range(16))
        cur_pos = ([x[0] for x in states])
        cur_vel = ([x[1] for x in states])
        cur_torque = ([x[3] for x in states])

        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robot)
        euler = p.getEulerFromQuaternion(cubeOrn)
        self.state_msg.imu.euler.x = euler[0]
        self.state_msg.imu.euler.y = euler[1]
        self.state_msg.imu.euler.z = euler[2]

        self.map_state_msg(cur_pos, cur_vel, cur_torque)

    def get_sim_period(self):
        return self.sim_period


def main():
    hw_level = Hardware_Level_Sim()
    sim_period = hw_level.get_sim_period()
    while True:
        start_time = time.time()
        hw_level.loop()
        elapced_time = time.time() - start_time
        wait_time = sim_period - elapced_time
        if wait_time > 0:
            time.sleep(wait_time)
        #print(wait_time)


if __name__ == '__main__':
    main()
