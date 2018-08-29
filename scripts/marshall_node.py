#!/usr/bin/env python
import atexit
import rospy
from mavros_msgs.msg import RCIn, OverrideRCIn, ParamValue, State
from mavros_msgs.srv import ParamGet, ParamSet, ParamSetRequest
from enum import IntEnum

from robot_learning.marshall import *

TRIGGER_CHANNEL = 4

class RC_CHANNELS(IntEnum):
    THROTTLE = 1
    STEER = 3
    USER_TOGGLE = 5


class PX4MarshallNode(MarshallNode):
    def __init__(self, name='px4_marshall'):
        super(PX4MarshallNode, self).__init__(name)
        self.prev_channels = []
        self.name = rospy.get_name()

        rospy.loginfo(
            '[%s] waiting for /mavros/param/set...' % self.name)
        rospy.wait_for_service('/mavros/param/set')
        self.set_param_client = rospy.ServiceProxy(
            '/mavros/param/set', ParamSet)
        self.get_param_client = rospy.ServiceProxy(
            '/mavros/param/get', ParamGet)

        # get defaults
        self.params = dict(
            SYSID_MYGCS=ParamValue(1, 0.0), 
            ARMING_REQUIRE=ParamValue(0, 0.0),
            ARMING_CHECK=ParamValue(0, 0.0))

        self.default_params = {}

        self.get_param('SYSID_MYGCS')
        rospy.loginfo(
            '[%s] waiting for parameters to be downloaded...' % self.name)
        for p in self.params:
            self.default_params[p] = self.get_param(p)

        self.cmd_pub = rospy.Publisher(
            '/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.rc_sub = rospy.Subscriber(
            '/mavros/rc/in', RCIn, self.rc_in_callback)

        if self.FSM in [FSM_STATES.USER, FSM_STATES.USER_PROMPT]:
            self.set_user_mode()
        else:
            self.set_rl_mode()
        self.fsm_pub.publish(str(self.FSM))

        self.rc_in_rl_sub = rospy.Subscriber(
            '/sandbox/rc/override', OverrideRCIn,
            self.process_command)
        
        self.mavros_state_sub = rospy.Subscriber(
            '/mavros/state', State,
            self.process_state)

        rospy.loginfo(
            '%s: initialized into %s mode' % (self.name, self.FSM))

    def set_user_mode(self):
        pass
        # reset params
        #for p in self.params:
        #    req = ParamSetRequest(p, self.default_params[p])
        #    response = self.set_param_client(req)
        #    if not response:
        #        rospy.loginfo("[%s] Failed to set %s" % (self.name, p))

    def set_rl_mode(self):
        # get the old values for the parameters we're going to change
        for p in self.params:
            self.default_params[p] = self.get_param(p)
        
        # set new values
        for p in self.params:
            req = ParamSetRequest(p, self.params[p])
            response = self.set_param_client(req)
            if not response:
                rospy.loginfo("[%s] Failed to set %s" % (self.name, p))

    def rc_in_callback(self, msg):
        if self.FSM == FSM_STATES.USER_PROMPT:
            self.trigger_start_pub.publish()
        if len(msg.channels) >= 4:
            if self.FSM == FSM_STATES.USER_PROMPT:
                self.trigger_start_pub.publish()
                # check if we the user wants to run an experiment
                # this is done by toggling the switch: we should read first a
                # 1520, then a 967.
                if len(self.prev_channels)>= 4:
                    if (self.prev_channels[TRIGGER_CHANNEL] == 967
                        and msg.channels[TRIGGER_CHANNEL] == 1520):
                        rospy.loginfo("[%s] Triggering reset" % (self.name))
                        self.trigger_start_pub.publish()
            elif self.FSM == FSM_STATES.RL:
                # check if the user wants to take over
                #if msg.channels[TRIGGER_CHANNEL] == 967:
                #    rospy.loginfo("[%s] Triggering stop" % (self.name))
                #    self.trigger_stop_pub.publish()
                pass

            
        self.prev_channels = msg.channels

    def process_command(self, msg):
        if self.FSM == FSM_STATES.RL:
            self.cmd_pub.publish(msg)

    def process_state(self, msg):
        pass

    def spin(self):
        rospy.spin()

    def shutdown(self):
        # reset original parameters
        for p in self.default_params:
            req = ParamSetRequest(p, self.default_params[p])
            response = self.set_param_client(req)
            if not response:
                rospy.loginfo("[%s] Failed to set %s" % (self.name, p))

    def get_param(self, param):
        response = self.get_param_client(param)
        while not response.success:
            rospy.sleep(1.0)
            response = self.get_param_client(param)
        return response.value

if __name__ == '__main__':
    try:
        px4 = PX4MarshallNode()
        atexit.register(px4.shutdown)
        px4.spin()
    except rospy.ROSInterruptException:
        pass