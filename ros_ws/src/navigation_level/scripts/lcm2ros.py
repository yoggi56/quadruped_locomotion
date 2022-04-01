#!/usr/bin/env python

import rospy
import lcm
from navigation_level.msg import hl_state
from navigation_level.lcm_msg_parse import LCM_Msg_Parse
from navigation_level.suppl import read_config


cur_angle = [0]*12
cur_vel = [0]*12
cur_torq = [0]*12
cur_euler = [0]*3


if __name__ == '__main__':
    rospy.init_node("lcm2ros")
    freq = read_config()
    rate = rospy.Rate(freq)
    pub = rospy.Publisher('HL_STATE', hl_state, queue_size=10)
    ros_msg = hl_state()

    lcm_msg_parser = LCM_Msg_Parse()

    while not rospy.is_shutdown():
        cur_angle, cur_vel, cur_torq, cur_euler = lcm_msg_parser.get_hl_state()
        ros_msg.cur_angle = cur_angle
        ros_msg.cur_vel = cur_vel
        ros_msg.cur_torque = cur_torq
        ros_msg.cur_euler_x = cur_euler[0]
        ros_msg.cur_euler_y = cur_euler[1]
        ros_msg.cur_euler_z = cur_euler[2]

        #rospy.loginfo(ros_msg.cur_angle[0])
        pub.publish(ros_msg)
        
        rate.sleep()



