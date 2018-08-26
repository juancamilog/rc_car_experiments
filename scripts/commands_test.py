#!/usr/bin/env python
import numpy as np
import rospy
from kusanagi.utils import print_with_stamp
from robot_learning.ros_plant import ROSPlant

max_cmd = 1.0
cmd_scale = 400
cmd_bias = 1493

if __name__=='__main__':
    rospy.init_node('commands_test', disable_signals=True)
    env = ROSPlant(dt=0.1)
    env.reset()
    t = rospy.get_time()
    while not rospy.is_shutdown():
        cmd = np.zeros(8)
        cmd[1] = 0.15*max_cmd*cmd_scale*np.sin(0.5*t) + cmd_bias
        cmd = cmd.astype(np.uint16)
        state, reward, info, done = env.step(cmd)
        f = 1.0/(rospy.get_time()-t)
        t = rospy.get_time()
