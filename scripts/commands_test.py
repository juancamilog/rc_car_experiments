#!/usr/bin/env python
import numpy as np
import rospy
from robot_learning.ros_plant import ROSPlant

max_cmd = 1.0
cmd_scale = 2**16

if __name__=='__main__':
    rospy.init_node('commands_test', disable_signals=True)
    env = ROSPlant()
    env.reset()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        cmd = max_cmd*cmd_scale*np.sin(0.1*t)
        state, reward, info, done = env.step(cmd)

