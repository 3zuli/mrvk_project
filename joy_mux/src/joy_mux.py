#!/usr/bin/env python
"""
# joy_mux

Subscribes to `source_topic`, publishes a list of topics defined by `published_topics`.
By default, source topic is redirected to the first topic in `published_topics` list.
Pressing the button defined by `button_index` will redirect the source topic to the next topic, wrapping around the end of the list.
"""

import sys, os, time, re, copy
from pprint import pprint
import copy

import rospy
from sensor_msgs.msg import Joy

rospy.init_node('joy_mux_node', anonymous=True)

# Button names for joystick Logitech EXTREME 3D PRO
button_names = {
    0: 'trigger',    # main trigger
    1: 'thumb',      # thumb button on the side of joystick
    2: '3', 3: '4',  # buttons '3'-'6' on top of joystick
    4: '5', 5: '6',
    6: '7', 7: '8',  # buttons '7'-'12' on joystick base
    8: '9', 9: '10', # buttons '8' and '10' DO NOT WORK
    10: '11', 11: '12'
}

class JoyMux:
    def __init__(self):
        self.active_topic = 0
        self.btnflag = 0
        self.t_switch = time.time()

        # Source topic
        self.source_topic = rospy.get_param('~source_topic', 'joy')
        # List of published topics
        self.pub_topic_list = rospy.get_param('~published_topics')
        # Button index in array Joy.buttons[] (default button '7' -> index 6, see above)
        self.button_index = rospy.get_param('~button_index', 6)

        if len(self.pub_topic_list)==0:
            rospy.logerr("No published topics specified, stopping.")
            exit(0)

        self.joy_sub = rospy.Subscriber(self.source_topic, Joy, self.joy_cb)
        rospy.loginfo("joy_mux: Subscribing to %s" % self.source_topic)

        self.joy_publishers = []
        for i, topic in enumerate(self.pub_topic_list):
            rospy.loginfo("joy_mux: Publishing topic %d: %s" % (i,topic))
            self.joy_publishers.append(rospy.Publisher(topic, Joy, queue_size=10))

        rospy.loginfo("joy_mux: Press button '%s' to switch published topic"
                      % button_names[self.button_index])
        rospy.logwarn("joy_mux: Switching to topic %d: %s"
                      % (self.active_topic, self.pub_topic_list[self.active_topic]))

        rospy.spin()

    def joy_cb(self, msg):
        # pprint(msg)

        t = time.time()
        # If button is clicked and it's at least 0.5s after last click,
        # go to next topic
        if msg.buttons[self.button_index] == 1 and self.btnflag == 0\
           and t-self.t_switch>0.5:
            # First, publish a zero command to the current topic
            empty_msg = copy.deepcopy(msg)
            empty_msg.axes = [0.0 for x in empty_msg.axes]
            empty_msg.buttons =  [0 for x in empty_msg.buttons]
            self.joy_publishers[self.active_topic].publish(empty_msg)

            # Switch to next topic
            self.active_topic = (self.active_topic + 1) % len(self.pub_topic_list)
            self.btnflag = 1
            self.t_switch = t
            rospy.logwarn("joy_mux: Switching to topic %d: %s" % (self.active_topic, self.pub_topic_list[self.active_topic]))
        else:
            self.btnflag = 0

        # Publish the message to current topic
        # for i, pub in enumerate(self.joy_publishers):
        #     if i==
        self.joy_publishers[self.active_topic].publish(msg)


if __name__=='__main__':
    joy_mux = JoyMux()