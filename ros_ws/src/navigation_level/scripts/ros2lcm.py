#!/usr/bin/env python

import rospy
import lcm
from navigation_level.msg import ref_angles
from navigation_level.ros_msg_parse import *
from msgs import ef_point
from msgs import robot_ref_com

ll_cmd_channel = "LL_COMMAND"

if __name__ == '__main__':
    # ros things
    rospy.init_node("ros2lcm")
    
    msg_parser = ROS_Msg_Parse()
    msg_parser.ll_com_init()

    rate = rospy.Rate(40)

    # data to be translated
    mode = 0
    go_idle = False
    go_sleep = False
    ef_refs = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                2: {"x": 0.0, "y": 0.0, "z": 0.0},
                3: {"x": 0.0, "y": 0.0, "z": 0.0},
                4: {"x": 0.0, "y": 0.0, "z": 0.0},
                }
    ref_body_vel = [0]*6
    robot_params = [0, 0, 0, 0, 0.12, 0.08, 0]
    ref_angle_vel = [0]*12
    cute_action = 0
    start = False

    # lcm things
    lc = lcm.LCM()
    lcm_msg = robot_ref_com()

    while not rospy.is_shutdown():
        mode, start, ref_angle_vel, ef_refs, ref_body_vel, robot_params, cute_action = msg_parser.get_ll_com()
        #rospy.loginfo("I got: {0}".format(theta_refs))

        lcm_msg.mode = mode
        lcm_msg.start = start

        for i in range(12):
            lcm_msg.srv_check.ref_angle[i] = ref_angle_vel[i]

        for i in range(4):
            lcm_msg.ref_ef_pt.x[i] = ef_refs[i+1]["x"]
            lcm_msg.ref_ef_pt.y[i] = ef_refs[i+1]["y"]
            lcm_msg.ref_ef_pt.z[i] = ef_refs[i+1]["z"]
        lcm_msg.body_vel.vx = ref_body_vel[0]
        lcm_msg.body_vel.vy = ref_body_vel[1]
        lcm_msg.body_vel.vz = ref_body_vel[2]
        lcm_msg.body_vel.wx = ref_body_vel[3]
        lcm_msg.body_vel.wy = ref_body_vel[4]
        lcm_msg.body_vel.wz = ref_body_vel[5]
        lcm_msg.robot_prms.robot_dx = robot_params[DX]
        lcm_msg.robot_prms.robot_dy = robot_params[DY]
        lcm_msg.robot_prms.robot_dz = robot_params[DZ]
        lcm_msg.robot_prms.robot_step_freq = robot_params[STEP_FREQ]
        lcm_msg.robot_prms.robot_step_lenght = robot_params[STEP_LENGHT]
        lcm_msg.robot_prms.robot_step_height = robot_params[STEP_HEIGHT]
        lcm_msg.robot_prms.robot_gait_type = robot_params[GAIT_TYPE]

        lcm_msg.cute_action_num = cute_action

        # rospy.loginfo("{0}, {1}, {2}, {3}, {4}, {5}, {6}".format(lcm_msg.robot_prms.robot_dx,
        #                         lcm_msg.robot_prms.robot_dy,
        #                         lcm_msg.robot_prms.robot_dz,
        #                         lcm_msg.robot_prms.robot_step_freq,
        #                         lcm_msg.robot_prms.robot_step_lenght,
        #                         lcm_msg.robot_prms.robot_step_height,
        #                         lcm_msg.robot_prms.robot_gait_type))

        # rospy.loginfo("{0}: {1}, {2}, {3}".format(lcm_msg.mode, 
        #                 lcm_msg.ref_ef_pt.x,
        #                 lcm_msg.ref_ef_pt.y,
        #                 lcm_msg.ref_ef_pt.z))

        lc.publish(ll_cmd_channel, lcm_msg.encode())

        #rospy.loginfo("{0} | {1}".format(lcm_msg.y, lcm_msg.z))
        #rospy.loginfo("I sent: {0}".format(lcm_msg.angle))
        rate.sleep()