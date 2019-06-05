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
from baxter_pydmps.pydmps import DMPs_discrete
import ipdb

csv_path = '/home/ye/ros_ws/src/baxter_pydmps/data/hand_marker.csv'
data_csv = pd.read_csv(csv_path)
raw_data = data_csv.values[:, 24:27].astype(float)
filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T

dmp = DMPs_discrete(n_dmps=3, n_bfs=200)
dmp.imitate_path(y_des=filtered_data.T)
y_track_d, dy_track_d, ddy_track_d = dmp.rollout()

#plot raw data
fig = plt.figure(1, figsize=(10, 8))
ax = fig.gca(projection='3d')
ax.plot(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], 'b', linewidth=3, linestyle='-')

#plot filtered data
fig = plt.figure(2, figsize=(10, 8))
ax = fig.gca(projection='3d')
ax.scatter(filtered_data[0, 0], filtered_data[0, 1], filtered_data[0, 2], color="r", s=100)
ax.scatter(filtered_data[-1, 0], filtered_data[-1, 1], filtered_data[-1, 2], color="b", s=100)
ax.plot(filtered_data[:, 0], filtered_data[:, 1], filtered_data[:, 2], linewidth=3, linestyle='-')

#plot dmp_discrete data
fig = plt.figure(3, figsize=(10, 8))
ax = fig.gca(projection='3d')
ax.plot(y_track_d[:, 0], y_track_d[:, 1], y_track_d[:, 2], linewidth=3, linestyle='--')

plt.show()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_move_test', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('right_arm')

waypoints = []
waypoints.append(group.get_current_pose().pose)
for idx in xrange(y_track_d.shape[0]):
	wpose = Pose()
	wpose.position.x = y_track_d[idx, 0]
	wpose.position.y = y_track_d[idx, 1]
	wpose.position.z = y_track_d[idx, 2]
	wpose.orientation.x = 0
	wpose.orientation.y = 0
	wpose.orientation.z = 0
	wpose.orientation.w = 1
	waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

group.execute(plan, wait=True)

moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)
