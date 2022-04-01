#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

RIGHT = 0
LEFT = 1
AXIS_RIGHT_Y = 2
BTN_2 = 3

class Joy_Control(object):
    def __init__(self, status_topic="joy",
                    read_freq=40):
        self._min_interval = 1.0/read_freq
        self._last_pub_time = rospy.Time()
        self.ref_vel = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.gamepad = [0]*4
        #self.gamepad_prev = [0]*3
        self.clr_it = 1
        rospy.Subscriber(status_topic, Joy, self.callback, queue_size=1)
    
    def callback(self, msg):
        """ msg type: Status """
        # we don't read messages if they come more frequent than base frequency of the node
        # now = rospy.Time.now()
        # if (now - self._last_pub_time).to_sec() < self._min_interval:
        #     return
        # read data from gamepad
        if msg.axes[4] == -1.0:
            self.gamepad[RIGHT] = 1
        if msg.axes[4] != -1.0:
            self.gamepad[RIGHT] = 0
        if msg.axes[4] == 1.0:
            self.gamepad[LEFT] = 1
        if msg.axes[4] != 1.0:
            self.gamepad[LEFT] = 0
        
        self.gamepad[AXIS_RIGHT_Y] = msg.axes[3]
        self.gamepad[BTN_2] = msg.buttons[1]
        # parameter control
        if self.gamepad[RIGHT]: #not self.gamepad_prev[RIGHT] and 
            if self.clr_it < 12:
                self.clr_it += 1
            else:
                self.clr_it = 1
            rospy.loginfo("Right: {0}".format(self.clr_it))
            #rospy.loginfo("Chosen joint: {0}\r\n Desired speed: {1}".format(self.clr_it, self.ref_vel))

        if self.gamepad[LEFT]: #not self.gamepad_prev[LEFT] and 
            if self.clr_it > 1:
                self.clr_it -= 1
            else:
                self.clr_it = 12
            rospy.loginfo("Left: {0}".format(self.clr_it))
        
        if self.gamepad[BTN_2]:
            self.ref_vel[self.clr_it-1] = 0
        
        rospy.loginfo("Chosen joint: {0}\r\n Desired speed: {1}".format(self.clr_it, self.ref_vel))
        self.ref_vel[self.clr_it-1] = self.gamepad[AXIS_RIGHT_Y]
        #rospy.loginfo("Stick: {0}. Cur: {1}. Prev {2}".format(self.ref_vel[self.clr_it-1], self.gamepad[RIGHT], self.gamepad_prev[RIGHT]))

        #self.gamepad_prev = self.gamepad
        # self._last_pub_time = now

    def get_data(self):
        return self.ref_vel

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