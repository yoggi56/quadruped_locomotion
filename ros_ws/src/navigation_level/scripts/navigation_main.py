#!/usr/bin/env python

#from navigation_level.Joy_Control import Joy_Control
from navigation_level.DS4_Control import *
import rospy
from navigation_level.msg import ll_command
from navigation_level.suppl import read_config


if __name__ == '__main__':
    rospy.init_node("nav_ctrl")
    freq = read_config()
    pub = rospy.Publisher('LL_COMMAND', ll_command, queue_size=10)
    ds4 = DS4_Control(read_freq=freq)
    # joy = Joy_Control(read_freq=freq)
    rate = rospy.Rate(freq)
    # ef_refs = {1: {"x": 0.3063, "y": -0.1835, "z": -0.3450},
    #             2: {"x": 0.3063, "y": 0.1835, "z": -0.3450},
    #             3: {"x": -0.2568, "y": -0.1835, "z": -0.3485},
    #             4: {"x": -0.2568, "y": 0.1835, "z": -0.3485},
    #             }
    ef_vel = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                2: {"x": 0.0, "y": 0.0, "z": 0.0},
                3: {"x": 0.0, "y": 0.0, "z": 0.0},
                4: {"x": 0.0, "y": 0.0, "z": 0.0},
                }
    init_ef_pos = [0.3063, -0.1835, -0.3450, 
                   0.3063,  0.1835, -0.3450, 
                  -0.2568, -0.1835, -0.3485, 
                  -0.2568,  0.1835, -0.3485]
    mode = 0
    ref_body_vel = [0]*6
    robot_params = [0, 0, 0, 0, 0, 0.08, 0]
    ref_angle_vel = [0]*12
    cute_action = 0
    start = False
    msg = ll_command()
    while not rospy.is_shutdown():
        mode, start, ref_angle_vel, ef_vel, ref_body_vel, robot_params, cute_action = ds4.get_data()
        
        msg.mode = mode
        msg.start = start

        for i in range(12):
            msg.ref_angle_vel[i] = ref_angle_vel[i]
        
        for i in range(1,5):
            msg.ef_vx[i-1] = ef_vel[i]["x"]
            msg.ef_vy[i-1] = ef_vel[i]["y"]
            msg.ef_vz[i-1] = ef_vel[i]["z"]

        msg.body_vx = ref_body_vel[0]
        msg.body_vy = ref_body_vel[1]
        msg.body_vz = ref_body_vel[2]
        msg.body_wx = ref_body_vel[3]
        msg.body_wy = ref_body_vel[4]
        msg.body_wz = ref_body_vel[5]

        msg.robot_dx = robot_params[DX]
        msg.robot_dy = robot_params[DY]
        msg.robot_dz = robot_params[DZ]
        msg.robot_step_freq = robot_params[STEP_FREQ]
        msg.robot_step_height = robot_params[STEP_HEIGHT]
        msg.robot_step_lenght = robot_params[STEP_LENGHT]
        msg.robot_gait_type = robot_params[GAIT_TYPE]

        msg.cute_action = cute_action
        
        #rospy.loginfo("{0} | {1}".format(mode, ef_vel))

        pub.publish(msg)
        rate.sleep()
