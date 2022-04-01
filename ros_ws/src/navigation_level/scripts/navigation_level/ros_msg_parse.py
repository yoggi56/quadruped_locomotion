#!/usr/bin/env python

import rospy
from navigation_level.msg import ref_angles
from navigation_level.msg import ef_points
from navigation_level.msg import ll_command

DX = 0
DY = 1
DZ = 2
STEP_FREQ = 3
STEP_LENGHT = 4
STEP_HEIGHT = 5
GAIT_TYPE = 6

class ROS_Msg_Parse(object):
    def __init__(self):
        pass

    def ref_angles_init(self):
        self.theta_refs = [0.0]*12
        rospy.Subscriber('LL_COMMAND', ref_angles, self.ref_angles_callback, queue_size=1)

    def ref_angles_callback(self, msg):
        self.theta_refs = msg.theta
        #rospy.loginfo("I got: {0}".format(msg.theta))

    def get_ref_angles(self):
        return self.theta_refs

    def ef_ref_init(self):
        self.ef_refs = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                2: {"x": 0.0, "y": 0.0, "z": 0.0},
                3: {"x": 0.0, "y": 0.0, "z": 0.0},
                4: {"x": 0.0, "y": 0.0, "z": 0.0},
                }
        rospy.Subscriber('LL_COMMAND', ef_points, self.ef_ref_callback, queue_size=1)

    def ef_ref_callback(self, msg):
        for i in range(4):
            self.ef_refs[i+1]["x"] = msg.x[i]
            self.ef_refs[i+1]["y"] = msg.y[i]
            self.ef_refs[i+1]["z"] = msg.z[i]
        #rospy.loginfo(msg)

    def get_ef_refs(self):
        return self.ef_refs

    def ll_com_init(self):
        self.mode = 0
        self.ef_refs = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                2: {"x": 0.0, "y": 0.0, "z": 0.0},
                3: {"x": 0.0, "y": 0.0, "z": 0.0},
                4: {"x": 0.0, "y": 0.0, "z": 0.0},
                }
        self.ref_body_vel = [0]*6
        self.robot_params = [0, 0, 0, 0, 0.12, 0.08, 0]
        self.ref_angle_vel = [0]*12
        self.cute_action = 0
        self.start = False
        rospy.Subscriber('LL_COMMAND', ll_command, self.ll_com_callback, queue_size=1)

    def ll_com_callback(self, msg):
        self.mode = msg.mode
        self.start = msg.start

        for i in range(12):
            self.ref_angle_vel[i] = msg.ref_angle_vel[i]

        self.cute_action = msg.cute_action

        for i in range(4):
            self.ef_refs[i+1]["x"] = msg.ef_vx[i]
            self.ef_refs[i+1]["y"] = msg.ef_vy[i]
            self.ef_refs[i+1]["z"] = msg.ef_vz[i]

        self.ref_body_vel[0] = msg.body_vx
        self.ref_body_vel[1] = msg.body_vy
        self.ref_body_vel[2] = msg.body_vz
        self.ref_body_vel[3] = msg.body_wx
        self.ref_body_vel[4] = msg.body_wy
        self.ref_body_vel[5] = msg.body_wz

        self.robot_params[DX] = msg.robot_dx
        self.robot_params[DY] = msg.robot_dy
        self.robot_params[DZ] = msg.robot_dz
        self.robot_params[STEP_FREQ] = msg.robot_step_freq
        self.robot_params[STEP_LENGHT] = msg.robot_step_lenght
        self.robot_params[STEP_HEIGHT] = msg.robot_step_height
        self.robot_params[GAIT_TYPE] = msg.robot_gait_type
        

    def get_ll_com(self):
        return self.mode, self.start, \
                self.ref_angle_vel, self.ef_refs, \
                self.ref_body_vel, self.robot_params, \
                self.cute_action