#!/usr/bin/env python
import rospy
from mavros_msgs.msg import OverrideRCIn, ParamValue, State
from mavros_msgs.srv import ParamGet, ParamSet, ParamSetRequest
from enum import IntEnum

from robot_learning.marshall import *


class RC_CHANNELS(IntEnum):
    THROTTLE = 1
    STEER = 3
    USER_TOGGLE = 5


class PX4MarshallNode(MarshallNode):
    def __init__(self, name='aqua_marshall'):
        super(PX4MarshallNode, self).__init__(name)

        rospy.loginfo(
            '%s: waiting for /mavros/param/set..' % rospy.get_name())
        rospy.wait_for_service('/mavros/param/set')
        self.set_param_client = rospy.ServiceProxy(
            '/mavros/param/set', ParamSet)
        self.get_param_client = rospy.ServiceProxy(
            '/mavros/param/get', ParamGet)

        # get defaults
        self.SYSID_MYGCS = self.get_param_client('SYSID_MYGCS').value
        self.ARMING_REQUIRE = self.get_param_client('ARMING_REQUIRE').value
        self.ARMING_CHECK = self.get_param_client('ARMING_CHECK').value

        self.cmd_pub = rospy.Publisher(
            '/mavros/rc/override', OverrideRCIn, queue_size=1)

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
            '%s: initialized into %s mode' % (rospy.get_name(), self.FSM))

    def set_user_mode(self):
        # reset params
        req = ParamSetRequest('SYSID_MYGCS', self.SYSID_MYGCS)
        response = self.set_param_client(req)
        req = ParamSetRequest('ARMING_REQUIRE', self.ARMING_REQUIRE)
        response = self.set_param_client(req)
        req = ParamSetRequest('ARMING_CHECK', self.ARMING_CHECK)
        response = self.set_param_client(req)

    def set_rl_mode(self):
        # get the old values for the parameters we're going to change
        self.SYSID_MYGCS = self.get_param_client('SYSID_MYGCS').value
        self.ARMING_REQUIRE = self.get_param_client('ARMING_REQUIRE').value
        self.ARMING_CHECK = self.get_param_client('ARMING_CHECK').value
        
        # set new values
        req = ParamSetRequest('SYSID_MYGCS', ParamValue(1, 0.0))
        response = self.set_param_client(req)
        req = ParamSetRequest('ARMING_REQUIRE', ParamValue(0, 0.0))
        response = self.set_param_client(req)
        req = ParamSetRequest('ARMING_CHECK', ParamValue(0, 0.0))
        response = self.set_param_client(req)

    def process_command(self, msg):
        if self.FSM == FSM_STATES.RL:
            self.cmd_pub.publish(msg)

    def process_state(self, msg):
        pass

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PX4MarshallNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass