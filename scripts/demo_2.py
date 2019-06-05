#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage.filters import gaussian_filter1d
from pydmps import DMPs_discrete
import ipdb

csv_path = '/home/ye/ros_ws/src/baxter_pydmps/data/baxter_joint_input_data.csv'
data_csv = pd.read_csv(csv_path)
raw_data = data_csv.values[:, 1:8].astype(float)
filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T

dmp = DMPs_discrete(n_dmps=7, n_bfs=200)
dmp.imitate_path(y_des=filtered_data.T)
y_track_d, dy_track_d, ddy_track_d = dmp.rollout()

#moveit
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_move_test', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('left_arm')

print "============ Robot Groups:"
print robot.get_group_names()


# print "============ Printing robot state"
# print robot.get_current_state()
# print "============"
joint_position = [0.91693702, -0.60515542, -0.13345633, 0.99900499, 0.05867477, 1.20033997, -0.05752428]
group.set_joint_value_target(joint_position)
group.go()
# plan = group.plan()
# group.execute(plan, wait=True)
rospy.sleep(2)

home_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
group.set_joint_value_target(home_joint_position)
group.go()
# plan = group.plan()
# group.execute(plan, wait=True)
rospy.sleep(2)

# for value in y_track_d:
# 	group.set_joint_value_target(value)
# 	plan = group.plan()
# 	group.execute(plan, wait=True)
# 	#print value


moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

