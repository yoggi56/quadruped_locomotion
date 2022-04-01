#!/usr/bin/env python

from statistics import mode
import rospy
from ds4_driver.msg import Feedback, Status
from math import *

ROBOT_MODE  = 0
EF_MODE     = 1
BODY_MODE   = 2
SERVO_MODE  = 3
CUTE_MODE   = 4

VX = 0
VY = 1
VZ = 2
WX = 3
WY = 4
WZ = 5

DX = 0
DY = 1
DZ = 2
STEP_FREQ   = 3
STEP_LENGHT = 4
STEP_HEIGHT = 5
GAIT_TYPE   = 6

class DS4_Control(object):
    def __init__(self, status_topic="status", feedback_topic="set_feedback",
                    read_freq=40):
        self._min_interval = 1.0/read_freq
        self._last_pub_time = rospy.Time()
        self._prev = Status()
        self._led = {
            "r": 0,
            "g": 0.5,
            "b": 0,
        }
        self.clr_it = 1
        self.colors = {0: {"r": 0.0, "g": 0.5, "b": 0.0},
                        1: {"r": 0.0, "g": 0.0, "b": 0.5},
                        2: {"r": 0.5, "g": 0.1, "b": 0.0},
                        3: {"r": 0.5, "g": 0.5, "b": 0.0},
                        4: {"r": 0.5, "g": 0.0, "b": 0.5},
                        5: {"r": 0.5, "g": 0.5, "b": 0.5},
                        6: {"r": 0.5, "g": 0.0, "b": 0.0},
                      }
        self.ref_ef_vel = {1: {"x": 0.0, "y": 0.0, "z": 0.0},
                            2: {"x": 0.0, "y": 0.0, "z": 0.0},
                            3: {"x": 0.0, "y": 0.0, "z": 0.0},
                            4: {"x": 0.0, "y": 0.0, "z": 0.0},
                        }
        self.ref_body_vel = [0]*6
        self.ref_angle_vel = [0]*12
        self.cute_action = 0
        self.servo_num = 0
        self.robot_params = [0, 0, 0, 0, 0.05, 0.04, 1]
        self.robot_params_tr = [0, 0, 0, 0, 0.05, 0.04, 1]
        self.robot_start_freq = 11
        self.mode = 0
        self.start = 0
        self._pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
        rospy.Subscriber(status_topic, Status, self.callback, queue_size=1)
        rospy.loginfo("Mode: {0}".format(self.mode))
        
    
    def callback(self, msg):
        """ msg type: Status """
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval:
            return

        feedback = Feedback()
        feedback.set_led = True
        feedback.set_rumble = True
        

        #switching between modes
        if not self._prev.button_dpad_up and msg.button_dpad_up:
            if self.mode < 4:
                self.mode += 1
            else:
                self.mode = 0
            self._led = self.colors[self.mode]
            self.clr_it = 1
            self.servo_num = 0
            rospy.loginfo("Mode: {0}".format(self.mode))


        if not self._prev.button_dpad_down and msg.button_dpad_down:
            if self.mode > 0:
                self.mode -= 1
            else:
                self.mode = 4
            self._led = self.colors[self.mode]
            self.clr_it = 1
            self.servo_num = 0
            rospy.loginfo("Mode: {0}".format(self.mode))
            
        # checking if start is pressed
        self.start = False
        if not self._prev.button_options and msg.button_options:
            self.start = True
            rospy.loginfo("Start is pressed")
            

        # execute modes
        if self.mode == EF_MODE:
            feedback.rumble_small = (abs(msg.axis_right_y) + abs(msg.axis_left_x) + abs(msg.axis_left_y))/2
            # switching between legs
            if not self._prev.button_dpad_right and msg.button_dpad_right:
                if self.clr_it < 4:
                    self.clr_it += 1
                else:
                    self.clr_it = 1
                #self._led = self.colors[self.clr_it]
                rospy.loginfo("Right: {0}.".format(self.clr_it))


            if not self._prev.button_dpad_left and msg.button_dpad_left:
                if self.clr_it > 1:
                    self.clr_it -= 1
                else:
                    self.clr_it = 4
                #self._led = self.colors[self.clr_it]
                rospy.loginfo("Left: {0}".format(self.clr_it))

            self.ref_ef_vel[self.clr_it]["x"] = msg.axis_left_y/400
            self.ref_ef_vel[self.clr_it]["y"] = -msg.axis_left_x/400
            self.ref_ef_vel[self.clr_it]["z"] = msg.axis_right_y/400
            #rospy.loginfo(self.ref_vel)
        elif self.mode == BODY_MODE:
            self.ref_body_vel[VX] = msg.axis_left_y/800
            self.ref_body_vel[VY] = -msg.axis_left_x/800
            self.ref_body_vel[VZ] = msg.axis_right_y/800
            if not msg.button_l1 and not msg.button_r1:
                self.ref_body_vel[WZ] = (msg.axis_r2 - msg.axis_l2)/800
            elif msg.button_l1 and not msg.button_r1:
                self.ref_body_vel[WX] = (msg.axis_r2 - msg.axis_l2)/800
            elif not msg.button_l1 and msg.button_r1:
                self.ref_body_vel[WY] = (msg.axis_r2 - msg.axis_l2)/800
            
        elif self.mode == ROBOT_MODE:
            self.robot_params[DX] = msg.axis_right_y
            self.robot_params[DY] = msg.axis_right_x
            self.robot_params[DZ] = msg.axis_r2 - msg.axis_l2
            self.robot_params[STEP_FREQ] = self.robot_start_freq + 6*sqrt(self.robot_params[DX]**2 + self.robot_params[DY]**2)
            if self.robot_params[STEP_FREQ] == self.robot_start_freq:
                self.robot_params[STEP_FREQ] = 0


            if not msg.button_l1 and not msg.button_r1:
                pass
                # if not self._prev.button_dpad_right and msg.button_dpad_right:
                #     if self.robot_params[STEP_FREQ] == 0:
                #         self.robot_params[STEP_FREQ] = 9.25
                #     else:
                #         self.robot_params[STEP_FREQ] += (0.25*pi)
                #     rospy.loginfo("Step frequency: {0}".format(self.robot_params[STEP_FREQ]))
                # elif not self._prev.button_dpad_left and msg.button_dpad_left:
                #     if self.robot_params[STEP_FREQ] > 9.25:
                #         self.robot_params[STEP_FREQ] -= (0.25*pi)
                #         rospy.loginfo("Step frequency: {0}".format(self.robot_params[STEP_FREQ]))
                #     else:
                #         self.robot_params[STEP_FREQ] = 0
                #         rospy.loginfo("Step frequency: {0}".format(self.robot_params[STEP_FREQ]))
            elif not msg.button_l1 and msg.button_r1:
                if msg.button_dpad_right: #not self._prev.button_dpad_right and 
                    if self.robot_params_tr[STEP_LENGHT] == 0:
                        self.robot_params_tr[STEP_LENGHT] = 0.05
                    else:
                        self.robot_params_tr[STEP_LENGHT] += 0.0005
                    #rospy.loginfo("Step length: {0}".format(self.robot_params[STEP_LENGHT]))
                elif msg.button_dpad_left: #not self._prev.button_dpad_left and 
                    if self.robot_params_tr[STEP_LENGHT] >= 0.05:
                        self.robot_params_tr[STEP_LENGHT] -= 0.0005
                        #rospy.loginfo("Step length: {0}".format(self.robot_params[STEP_LENGHT]))
                    # else:
                    #     self.robot_params_tr[STEP_LENGHT] = 0
                        #rospy.loginfo("Step length: {0}".format(self.robot_params[STEP_LENGHT]))
            elif msg.button_l1 and not msg.button_r1:
                if msg.button_dpad_right: #not self._prev.button_dpad_right and 
                    if self.robot_params_tr[STEP_HEIGHT] == 0:
                        self.robot_params_tr[STEP_HEIGHT] = 0.04
                    else:
                        self.robot_params_tr[STEP_HEIGHT] += 0.0005
                        #rospy.loginfo("Step height: {0}".format(self.robot_params[STEP_HEIGHT]))
                elif msg.button_dpad_left: #not self._prev.button_dpad_left and 
                    if self.robot_params_tr[STEP_HEIGHT] >= 0.04:
                        self.robot_params_tr[STEP_HEIGHT] -= 0.0005
                        #rospy.loginfo("Step height: {0}".format(self.robot_params[STEP_HEIGHT]))
                    # else:
                    #     self.robot_params_tr[STEP_HEIGHT] = 0
                        #rospy.loginfo("Step height: {0}".format(self.robot_params[STEP_HEIGHT]))

            if msg.button_cross:
                self.robot_params[GAIT_TYPE] = 0
                rospy.loginfo("Gait type: Walk")
            if msg.button_square:
                self.robot_params[GAIT_TYPE] = 1
                rospy.loginfo("Gait type: Trot")
            if msg.button_circle:
                self.robot_params[GAIT_TYPE] = 2
                rospy.loginfo("Gait type: Pace")
            if msg.button_triangle:
                self.robot_params[GAIT_TYPE] = 3
                rospy.loginfo("Gait type: Gallop")

            if self.robot_params[STEP_FREQ] == 0:
                self.robot_params[STEP_HEIGHT] = 0
                self.robot_params[STEP_LENGHT] = 0
            else:
                self.robot_params[STEP_HEIGHT] = self.robot_params_tr[STEP_HEIGHT]
                self.robot_params[STEP_LENGHT] = self.robot_params_tr[STEP_LENGHT]

        elif self.mode == SERVO_MODE:
            if not self._prev.button_dpad_right and msg.button_dpad_right:
                if self.servo_num < 11:
                    self.servo_num += 1
                else:
                    self.servo_num = 0

            if not self._prev.button_dpad_left and msg.button_dpad_left:
                if self.servo_num > 0:
                    self.servo_num -= 1
                else:
                    self.servo_num = 11
            
            self.ref_angle_vel[self.servo_num] = msg.axis_right_y
        elif self.mode == CUTE_MODE:
            self.cute_action = 0
            if not self._prev.button_cross and msg.button_cross:
                self.cute_action = 1
                rospy.loginfo("Cute action: 1")
            if msg.button_square and not self._prev.button_square:
                self.cute_action = 2
                rospy.loginfo("Cute action: 2")
            if msg.button_circle and not self._prev.button_circle:
                self.cute_action = 3
                rospy.loginfo("Cute action: 3")
            if msg.button_triangle and not self._prev.button_triangle:
                self.cute_action = 4
                rospy.loginfo("Cute action: 4")


        feedback.led_r = self._led["r"]
        feedback.led_g = self._led["g"]
        feedback.led_b = self._led["b"]

        self._pub_feedback.publish(feedback)
        
        self._prev = msg
        self._last_pub_time = now

    def get_data(self):
        return self.mode, self.start, \
                self.ref_angle_vel, \
                self.ref_ef_vel, self.ref_body_vel, \
                self.robot_params, self.cute_action
        

# def ds4_callback(data):
#     rospy.loginfo(" I heard %s", data.button_dpad_up)

# def listener():
#     #rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("status", Status, ds4_callback, queue_size=1)
#     #rospy.spin()

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(1) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     rospy.init_node("ds4_translator")
#     DS4_Control()
#     rospy.spin()