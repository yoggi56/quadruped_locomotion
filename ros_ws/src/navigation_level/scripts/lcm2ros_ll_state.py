#!/usr/bin/env python

import rospy
import lcm
from navigation_level.msg import ef_points
from navigation_level.lcm_msg_parse import LCM_Msg_Parse
from navigation_level.suppl import read_config


cur_angle = [0]*12
cur_vel = [0]*12
cur_torq = [0]*12
cur_euler = [0]*3


if __name__ == '__main__':
    rospy.init_node("lcm2ros_ll_state")
    freq = read_config()
    rate = rospy.Rate(freq)
    pub = rospy.Publisher('LL_STATE', ef_points, queue_size=10)
    ros_msg = ef_points()

    lcm_msg_parser = LCM_Msg_Parse()

    ef_curs = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                2: {"x": 0.0, "y": 0.0, "z": 0.0},
                3: {"x": 0.0, "y": 0.0, "z": 0.0},
                4: {"x": 0.0, "y": 0.0, "z": 0.0},
                }

    while not rospy.is_shutdown():
        ef_curs = lcm_msg_parser.get_ll_state()
        for i in range(4):
            ros_msg.x[i] = ef_curs[i+1]["x"]
            ros_msg.y[i] = ef_curs[i+1]["y"]
            ros_msg.z[i] = ef_curs[i+1]["z"]

        #rospy.loginfo(ros_msg.cur_angle[0])
        pub.publish(ros_msg)
        
        rate.sleep()