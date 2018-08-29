#!/usr/bin/env python
import atexit
import curses
import numpy as np
import rospy
from robot_learning.ros_plant import ROSPlant
from mavros_msgs.srv import ParamGet
from sensor_msgs.msg import Joy

max_throttle = 0.2
cmd_revert = [-1, 1]
cmd_min = [1100, 1100]
cmd_max = [1900, 1900]
cmd_bias = [1500, 1500]
steer = cmd_bias
throttle = cmd_bias
cmd = np.zeros(8)


def handle_joystick(msg):
    steer = msg.axes[2]*cmd_revert[0]
    throttle = msg.axes[1]*cmd_revert[1]
    if msg.buttons[6]:
        if msg.buttons[7]:
            max_throttle = 1.0
        else:
            max_throttle = 0.4
    else:
        max_throttle = 0.2

    steer_scale = cmd_max[0]- cmd_bias[0] if steer > 0 else  cmd_bias[0] - cmd_min[0]
    throttle_scale = cmd_max[1]- cmd_bias[1] if throttle > 0 else  cmd_bias[1] - cmd_min[1]
    throttle_scale *= max_throttle
    cmd[0] = (steer*steer_scale + cmd_bias[0])
    cmd[1] = (throttle*throttle_scale + cmd_bias[1])


def get_param(get_param_srv, param):
    response = get_param_srv(param)
    while not response.success:
        rospy.sleep(0.1)
        response = get_param_srv(param)
    return response.value


if __name__=='__main__':
    rospy.init_node('commands_test', disable_signals=True)
    # setup joypad subscriber
    joy_sub = rospy.Subscriber('/joy', Joy, handle_joystick)

    # get rc trim, deadzone and range
    rospy.wait_for_service('/mavros/param/get')
    get_param_srv = rospy.ServiceProxy(
        '/mavros/param/get', ParamGet)

    cmd_min[0] = get_param(get_param_srv, 'RC1_MIN').integer
    cmd_min[1] = get_param(get_param_srv, 'RC2_MIN').integer
    cmd_max[0] = get_param(get_param_srv, 'RC1_MAX').integer
    cmd_max[1] = get_param(get_param_srv, 'RC2_MAX').integer
    cmd_bias[0] = get_param(get_param_srv, 'RC1_TRIM').integer
    cmd_bias[1] = get_param(get_param_srv, 'RC2_TRIM').integer
    
    # initalize plant
    env = ROSPlant(dt=rospy.get_param('~dt', 0.05))
    env.reset()
    t = rospy.get_time()

    # main loop
    while not rospy.is_shutdown():
        # apply command
        state, reward, info, done = env.step(cmd.astype(np.uint16))
        t = rospy.get_time()

