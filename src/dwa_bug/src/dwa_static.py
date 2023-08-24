#!/usr/bin/env python

# Author: Connor McGuile
# Feel free to use in any way.

# A custom Dynamic Window Approach implementation for use with Turtlebot.
# Obstacles are registered by a front-mounted laser and stored in a set.
# If, for testing purposes or otherwise, you do not want the laser to be used,
# disable the laserscan subscriber and create your own obstacle set in main(),
# before beginning the loop. If you do not want obstacles, create an empty set.
# Implentation based off Fox et al.'s paper, The Dynamic Window Approach to
# Collision Avoidance (1997).

import cv2
import rospy
import math
import numpy as np
import os
import datetime
import time
import matplotlib as mpl
import sympy
import mpmath as mp
import random
import argparse


from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
from scipy.optimize import curve_fit
from scipy.interpolate import splprep, splev
from scipy.integrate import quad

from scipy.interpolate import interp1d, PchipInterpolator

from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap, LinearSegmentedColormap

from matplotlib.ticker import MultipleLocator
# from sympy import *
from pathsolver import solver
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from mpl_toolkits.axes_grid1 import make_axes_locatable
from mpl_toolkits.axes_grid1.inset_locator import mark_inset, inset_axes
from pyinstrument import Profiler
from colorama import Fore, Style
import PIL.Image as Image


class Config():
	# simulation parameters
	def __init__(self):
		self.dt = 0.1  # [s]
		self.predict_time = 2.0

		# robot parameter
		self.max_linear_velocity = 2.0  # [m/s]
		self.min_linear_velocity = 0.0

		self.max_angular_velocity = 6.28  # [rad/s]360 degree
		self.min_angular_velocity = -6.28

		self.max_linear_acceleration = 1.0  # 2.0  # [m/ss]
		self.max_angular_acceleration = 5.24  # [rad/ss]300 degree

		self.max_linear_jerk = 0.5  # [m/sss]
		self.max_angular_jerk = 1.31  # [m/sss]

		self.linear_velocity_resolution = self.max_linear_acceleration * self.dt * 0.5
		self.angular_velocity_resolution = self.max_angular_acceleration * self.dt * 0.5

		self.linear_acceleration_resolution = self.max_linear_acceleration * 0.5
		self.angular_acceleration_resolution = self.max_angular_acceleration * 0.5

		self.linear_jerk_resolution = self.max_linear_jerk * 0.5
		self.angular_jerk_resolution = self.max_angular_jerk * 0.5

		self.robot_radius = 0.2  # [m]
		self.sensor_range = 5.0

		self.goalX = 18.0
		self.goalY = 0.0
		self.startX = -18.0
		self.startY = 0.0
		self.x = self.startX
		self.y = self.startY
		self.th = 0.0
		self.localX = self.goalX
		self.localY = self.goalY

		self.local_goal_index = 0
		self.motion_model = "tangent" #tangent or secant

		self.current_v = 0.0
		self.current_w = 0.0
		self.r = rospy.Rate(10)

		self.error = []

		self.linear_velocity_list = [1.0]
		self.angular_velocity_list = [0.0]
		self.linear_acceleration_list = [0.0]
		self.angular_acceleration_list = [0.0]
		self.linear_jerk_list = [0.0]
		self.angular_jerk_list = [0.0]

		self.odom_x_list = [self.startX]
		self.odom_y_list = [self.startY]
		self.odom_th_list = []

		self.cal_time1 = []
		self.cal_time2 = []
		self.cal_time3 = []

		self.ob_nums = []
		self.path = []
		self.remain_path = self.path

		self.stop_dist = 0.5

		self.v_weight = 0.1
		self.j_weight = 0.1

		self.result_graph_type = "gray"  # gray or color
		self.show_path_figure = False
		self.show_pyinstrument = False
		self.show_result_figure = False

		self.save_result_figure = True
		self.save_process_time = True

		self.result_path = '../results/{}/'.format(time.strftime("%Y%m%d", time.localtime()))

		self.type = "a"  # dwa a a2 j
		# self.laser_reso = []
		self.map = '../world/1181.png'

	def assignOdomCoords(self, msg):
		# X- and Y- coords and pose of robot fed back into the robot config
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		r = R.from_quat([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		(roll, pitch, theta) = r.as_euler('xyz', degrees=False)
		self.th = theta

	# Callback for cmd_vel
	def callcmd(self, msg):
		self.current_v = msg.twist.twist.linear.x
		self.current_w = msg.twist.twist.angular.z


def motion(config, x, u):
	# x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
	x = [x[0], x[1], x[2], u[0], u[1]]
	if config.motion_model == "tangent":
		model = np.array([[1, 0, 0, config.dt * math.cos(x[2]), 0],
						[0, 1, 0, config.dt * math.sin(x[2]), 0],
						[0, 0, 1, 0, config.dt],
						[0, 0, 0, 1, 0],
						[0, 0, 0, 0, 1]])
	elif config.motion_model == "secant":
		model = np.array([[1, 0, 0, config.dt * math.cos(x[2]+u[1]*config.dt*0.5), 0],
						[0, 1, 0, config.dt * math.sin(x[2]+u[1]*config.dt*0.5), 0],
						[0, 0, 1, 0, config.dt],
						[0, 0, 0, 1, 0],
						[0, 0, 0, 0, 1]])

	x = np.dot(model, x)
	return x


def accelerationToVelocity(config, a, mode):
	predict_velocity_list = []
	if mode == 'av':
		predict_velocity_list.append(config.linear_velocity_list[-1])
		for i in range(int(config.predict_time/config.dt)):
			if predict_velocity_list[-1] + a * config.dt >= 0:
				predict_velocity_list.append(min(config.max_linear_velocity, predict_velocity_list[-1] + a * config.dt))
			else:
				predict_velocity_list.append(max(0, predict_velocity_list[-1] + a * config.dt))
	elif mode == 'aw':
		predict_velocity_list.append(config.angular_velocity_list[-1])
		for i in range(int(config.predict_time/config.dt)):
			if predict_velocity_list[-1] + a * config.dt >= 0:
				predict_velocity_list.append(min(config.max_angular_velocity, w_predict_list[-1] + aw * config.dt))
			else:
				predict_velocity_list.append(max(-config.max_angular_velocity, w_predict_list[-1] + aw * config.dt))
	del (predict_velocity_list[0])
	return predict_velocity_list


def jerkToVelocity(config, j, mode):
	time = 0
	predict_acceleration_list = []
	predict_velocity_list = []
	if mode == "jv":
		predict_acceleration_list.append(config.linear_acceleration_list[-1])
		predict_velocity_list.append(config.linear_velocity_list[-1])
		while time <= config.predict_time:
			if predict_acceleration_list[-1] + j * config.dt >= 0:
				predict_acceleration_list.append(
					min(config.max_linear_acceleration, predict_acceleration_list[-1] + j * config.dt))
			else:
				predict_acceleration_list.append(
					max(-config.max_linear_acceleration, predict_acceleration_list[-1] + j * config.dt))
			time = time + config.dt
		del (predict_acceleration_list[0])

		for i in range(len(predict_acceleration_list)):
			if predict_velocity_list[-1] + predict_acceleration_list[i] * config.dt >= 0:
				predict_velocity_list.append(min(config.max_linear_velocity,
												 predict_velocity_list[-1] + predict_acceleration_list[i] * config.dt))
			else:
				predict_velocity_list.append(
					max(0, predict_velocity_list[-1] + predict_acceleration_list[i] * config.dt))
		del (predict_velocity_list[0])

	return predict_velocity_list, predict_acceleration_list


# Determine the dynamic window from robot configurations
def calcDynamicWindow(config, x):
	v1 = x[3] - config.max_linear_acceleration * config.dt
	v2 = x[3] + config.max_linear_acceleration * config.dt
	# Dynamic window from robot specification
	Vs = [config.min_linear_velocity, config.max_linear_velocity,
		  -config.max_angular_velocity, config.max_angular_velocity]
	# Dynamic window from motion model
	Vd = [v1,
		  v2,
		  x[4] - config.max_angular_acceleration * config.dt,
		  x[4] + config.max_angular_acceleration * config.dt]

	#  [vmin, vmax, yawrate min, yawrate max]
	dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
	return dw


# Calculate a trajectory sampled across a prediction time
def calcTrajectory(config, xinit, v_predict_list, w_predict_list, ):
	x = np.array(xinit)
	traj = np.array(xinit)  # many motion models stored per trajectory
	#print(v_predict_list,w_predict_list)
	for i in range(len(v_predict_list)):
		x = motion(config, x, [v_predict_list[i], w_predict_list[i]])
		traj = np.vstack((traj, x))
	traj = np.delete(traj, 0, 0)
	return traj


# Calculate obstacle cost inf: collision, 0:free
def calcObstacleCost(config, traj, obst):
	minr = 1000
	# Loop through every obstacle in set and calc Pythagorean distance
	# Use robot radius to determine if collision
	path = traj[:, 0:2]
	obst = list(obst)
	if len(obst) == 0:
		minr = 1000
	else:
		obst = np.array(obst)
		config.ob_nums.append(obst.shape[0])
		dist_matrix = distance.cdist(obst, path, metric="euclidean")
		# dist_matrix = np.sqrt(np.sum((obst[:,np.newaxis]-path[np.newaxis,:])**2,axis=2))
		minr = np.min(dist_matrix)
	if minr <= config.robot_radius:
		cost = -10000  # collision
	elif config.robot_radius < minr <= 1.2:
		cost = minr - 0.2
	else:
		cost = 1
	return cost, minr


def calcObstacleCost2(config, obst_rc, av, v_predict_list, w):
	minr = 1000
	vn = v_predict_list[-1]
	v0 = config.linear_velocity_list[-1]
	w0 = w

	def ff_(t):
		if av > 0:
			acct = (config.max_linear_velocity - v0) / av
			if 0 <= t <= acct:
				ff_ = (v0 + av * t) * math.cos(w0 * t)
			elif acct < t <= config.predict_time:
				ff_ = config.max_linear_velocity * math.cos(w0 * t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0 / av)
			if 0 <= t <= acct:
				ff_ = (v0 + av * t) * math.cos(w0 * t)
			elif acct < t <= config.predict_time:
				ff_ = 0
			else:
				print('out of range')
		else:
			ff_ = v0 * math.cos(w0 * t)
		return ff_

	def gg_(t):
		if av > 0:
			acct = (config.max_linear_velocity - v0) / av
			if 0 <= t <= acct:
				gg_ = (v0 + av * t) * math.sin(w0 * t)
			elif acct < t <= config.predict_time:
				gg_ = config.max_linear_velocity * math.sin(w0 * t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0 / av)
			if 0 <= t <= acct:
				gg_ = (v0 + av * t) * math.sin(w0 * t)
			elif acct < t <= config.predict_time:
				gg_ = 0
			else:
				print('out of range')
		else:
			gg_ = v0 * math.sin(w0 * t)
		return gg_

	def ff(t):
		# print(t)
		if av > 0:
			acct = (config.max_linear_velocity - v0) / av
			x0 = (av * math.cos(w0 * 0) + w0 * (v0 + av * 0) * math.sin(w0 * 0)) / (w0 ** 2)
			dx1 = x0 - 0
			if 0 <= t <= acct:
				f = (av * math.cos(w0 * t) + w0 * (v0 + av * t) * math.sin(w0 * t)) / (w0 ** 2) - dx1
			elif acct < t <= config.predict_time:
				x1 = (av * math.cos(w0 * acct) + w0 * (v0 + av * acct) * math.sin(w0 * acct)) / (w0 ** 2) - dx1
				x2 = config.max_linear_velocity * math.sin(w0 * acct) / w0
				dx2 = x2 - x1
				f = config.max_linear_velocity * math.sin(w0 * t) / w0 - dx2
			else:
				print('av > 0')
				print('out of range')
		elif av < 0:
			acct = abs(v0 / av)
			x0 = (av * math.cos(w0 * 0) + w0 * (v0 + av * 0) * math.sin(w0 * 0)) / (w0 ** 2)
			dx1 = x0 - 0
			if 0 <= t <= acct:
				f = (av * math.cos(w0 * t) + w0 * (v0 + av * t) * math.sin(w0 * t)) / (w0 ** 2) - dx1
			elif acct < t <= config.predict_time:
				f = (av * math.cos(w0 * acct) + w0 * (v0 + av * acct) * math.sin(w0 * acct)) / (w0 ** 2) - dx1
			else:
				print(v0, vn, av)
				print('t=', t)
				print('av < 0')
				print('out of range')
				print('==============')
		else:
			x0 = (v0 * math.sin(w0 * 0) / w0)
			dx1 = x0 - 0
			f = (v0 * math.sin(w0 * t) / w0) - dx1
		return f

	def gg(t):
		if av > 0:
			acct = (config.max_linear_velocity - v0) / av
			y0 = (av * math.sin(w0 * 0) - w0 * (v0 + av * 0) * math.cos(w0 * 0)) / (w0 ** 2)
			dy1 = y0 - 0
			if 0 <= t <= acct:
				g = (av * math.sin(w0 * t) - w0 * (v0 + av * t) * math.cos(w0 * t)) / (w0 ** 2) - dy1
			elif acct < t <= config.predict_time:
				y1 = (av * math.sin(w0 * acct) - w0 * (v0 + av * acct) * math.cos(w0 * acct)) / (w0 ** 2) - dy1
				y2 = -config.max_linear_velocity * math.cos(w0 * acct) / w0
				dy2 = y2 - y1
				g = -config.max_linear_velocity * math.cos(w0 * t) / w0 - dy2
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0 / av)
			y0 = (av * math.sin(w0 * 0) - w0 * (v0 + av * 0) * math.cos(w0 * 0)) / (w0 ** 2)
			dy1 = y0 - 0
			if 0 <= t <= acct:
				g = (av * math.sin(w0 * t) - w0 * (v0 + av * t) * math.cos(w0 * t)) / (w0 ** 2) - dy1
			elif acct < t <= config.predict_time:
				g = (av * math.sin(w0 * acct) - w0 * (v0 + av * acct) * math.cos(w0 * acct)) / (w0 ** 2) - dy1
			else:
				print('out of range')
		else:
			y0 = -(v0 * math.cos(w0 * 0) / w0)
			dy1 = y0 - 0
			g = -(v0 * math.cos(w0 * t) / w0) - dy1
		return g

	if w0 != 0:
		xend = ff(config.predict_time)
		yend = gg(config.predict_time)
		if av != 0:
			midtime = abs((math.sqrt((v0 ** 2 + vn ** 2) * 0.5) - v0) / av)
		else:
			midtime = config.predict_time * 0.5

		xmid = ff(midtime)
		ymid = gg(midtime)
		dx21 = (v0 / w0) * math.sin(w0 * config.predict_time) - xend
		dy21 = (v0 / w0) * (1 - math.cos(w0 * config.predict_time)) - yend
		dx31 = (vn / w0) * math.sin(w0 * config.predict_time) - xend
		dy31 = (vn / w0) * (1 - math.cos(w0 * config.predict_time)) - yend

		dx41 = (v0 / w0) * math.sin(w0 * midtime) - xmid
		dy41 = (v0 / w0) * (1 - math.cos(w0 * midtime)) - ymid
		dx51 = (vn / w0) * math.sin(w0 * midtime) - xmid
		dy51 = (vn / w0) * (1 - math.cos(w0 * midtime)) - ymid

		inside_circle_center = np.array([[0, v0 / w0],  # 2,4,6
										 [0 - dx21, v0 / w0 - dy21],
										 [0 - dx41, v0 / w0 - dy41]])
		outside_circle_center = np.array([[0, vn / w0],  # 3,5,7
										  [0 - dx31, vn / w0 - dy31],
										  [0 - dx51, vn / w0 - dy51]])

		if ff_(config.predict_time) != 0:
			k = gg_(config.predict_time) / ff_(config.predict_time)
			k = -1 / k
		else:
			k = 1
		b = yend - k * xend
		anglemax = math.atan2(yend - b, xend)

		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
			obst_inrange = np.array([0])
			obst_outrange = np.array([0])
		else:
			obst_rc_array = np.array(obst_rc_list)
			angel_matrix = np.arctan2(obst_rc_array[:, 1] - b, obst_rc_array[:, 0])
			angel_matrix2 = angel_matrix.reshape(-1, 1)
			obst_rc_array = np.hstack((obst_rc_array, angel_matrix2))
			if w0 > 0:
				inrange_condition = (-0.5 * math.pi < obst_rc_array[:, 2]) & (obst_rc_array[:, 2] < anglemax)
			else:
				inrange_condition = (anglemax < obst_rc_array[:, 2]) & (obst_rc_array[:, 2] < 0.5 * math.pi)

			obst_inrange = obst_rc_array[inrange_condition]
			obst_outrange = obst_rc_array[~inrange_condition]
			# per_pair1.append(obst_inrange.shape[0])
			# per_pair2.append(obst_outrange.shape[0])

			dist_to_outside_center = distance.cdist(outside_circle_center, obst_inrange[:, 0:2], metric="euclidean")
			dist_to_inside_center = distance.cdist(inside_circle_center, obst_inrange[:, 0:2], metric="euclidean")
			if np.any(obst_inrange):
				mt1 = np.absolute(dist_to_outside_center - (vn / abs(w0)))
				mt2 = np.absolute(dist_to_inside_center - (v0 / abs(w0)))
				c1 = mt1.min(0) > 0
				c2 = mt2.max(0) < 0
				if av > 0:
					mt = np.where(c1, 0.5 * mt1.max(0) + 0.5 * mt2.min(0),
								  np.where(c2, 0.5 * mt1.min(0) + 0.5 * mt2.max(0), 0))
				else:
					mt = np.where(c1, 0.5 * mt1.min(0) + 0.5 * mt2.max(0),
								  np.where(c2, 0.5 * mt1.max(0) + 0.5 * mt2.min(0), 0))
				dist_a = np.min(mt)
			else:
				dist_a = 1000

			if np.any(obst_outrange):
				dist_b1 = np.min(distance.cdist(np.array([[xend, yend]]), obst_outrange[:, 0:2], metric="euclidean"))
				dist_b2 = np.min(distance.cdist(np.array([[0, 0]]), obst_outrange[:, 0:2], metric="euclidean"))
				dist_b = min(dist_b1, dist_b2)
			else:
				dist_b = 1000
			dist = min(dist_a, dist_b)

	else:
		if av > 0:
			acct = (config.max_linear_velocity - v0) / av
			xend = (v0 * acct + 0.5 * av * acct ** 2) + config.max_linear_velocity * (config.predict_time - acct)
		elif av < 0:
			acct = abs(v0 / av)
			if acct > config.predict_time:
				xend = (v0 * config.predict_time + 0.5 * av * config.predict_time ** 2)
			else:
				xend = (v0 * acct + 0.5 * av * acct ** 2)
		else:
			xend = v0 * config.predict_time
		yend = 0
		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
			obst_inrange = np.array([0])
			obst_outrange = np.array([0])

		else:
			obst_rc_array = np.array(obst_rc_list)
			inrange_condition = (0 < obst_rc_array[:, 0]) & (obst_rc_array[:, 0] < xend)
			obst_inrange = obst_rc_array[inrange_condition]
			obst_outrange = obst_rc_array[~inrange_condition]
			if np.any(obst_inrange):
				dist_a = np.min(np.absolute(obst_inrange[:, 1]))
			else:
				dist_a = 1000
			if np.any(obst_outrange):
				dist_b1 = np.min(distance.cdist(np.array([[0, 0]]), obst_outrange, metric="euclidean"))
				dist_b2 = np.min(distance.cdist(np.array([[xend, yend]]), obst_outrange, metric="euclidean"))
				dist_b = min(dist_b1, dist_b2)
			else:
				dist_b = 1000
			dist = min(dist_a, dist_b)
	if dist <= minr:
		minr = dist
	if minr <= config.robot_radius:
		cost = -10000  # collision
	elif config.robot_radius < minr <= 1.2:
		cost = minr - 0.2
	else:
		cost = 1

	return cost, xend, yend, obst_inrange.shape[0], obst_outrange.shape[0]


def result_graph(config, mode, element1, element2=None, element3=None):
	plt.rcParams["axes.labelsize"] = 14
	plt.rcParams["xtick.labelsize"] = 14
	plt.rcParams["ytick.labelsize"] = 14

	fig = plt.figure(figsize=(8, 3), tight_layout=False)
	ax_element1 = HostAxes(fig, [0.1, 0.15, 0.65, 0.8])
	ax_element2 = ParasiteAxes(ax_element1, sharex=ax_element1)
	ax_element3 = ParasiteAxes(ax_element1, sharex=ax_element1)

	ax_element1.parasites.append(ax_element2)
	ax_element1.parasites.append(ax_element3)

	ax_element1.axis['right'].set_visible(False)
	ax_element1.axis['top'].set_visible(False)
	ax_element2.axis['right'].set_visible(True)
	ax_element2.axis['right'].major_ticklabels.set_visible(True)
	ax_element2.axis['right'].label.set_visible(True)

	element3_axisline = ax_element3.get_grid_helper().new_fixed_axis
	ax_element3.axis['right2'] = element3_axisline(loc='right', axes=ax_element3, offset=(60, 0))

	fig.add_axes(ax_element1)

	ax_element2.axis['right'].label.set_color('black')
	ax_element3.axis['right2'].label.set_color('black')

	ax_element2.axis['right'].major_ticks.set_color('black')
	ax_element3.axis['right2'].major_ticks.set_color('black')

	ax_element2.axis['right'].major_ticklabels.set_color('black')
	ax_element3.axis['right2'].major_ticklabels.set_color('black')

	ax_element2.axis['right'].line.set_color('black')
	ax_element3.axis['right2'].line.set_color('black')
	if mode == 'vaj':
		ax_element1.set_ylabel('Velocity [m/s]')
		ax_element1.set_xlabel('Time [s]')
		ax_element2.set_ylabel('Acceleration [' + 'm/$ \mathit{s}^{2}$' + ']')
		ax_element3.set_ylabel('Jerk [' + 'm/$ \mathit{s}^{3}$' + ']')
		ax_element1.set_ylim(0.0, config.max_linear_velocity + 0.5)
		ax_element2.set_ylim(-config.max_linear_acceleration * 2, config.max_linear_acceleration * 2)
		ax_element3.set_ylim(-config.max_linear_acceleration * 20, config.max_linear_acceleration * 20)
		if config.result_graph_type == "gray":
			xl = np.arange(0, len(element1)) * 0.1
			curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color='black', linewidth=1,
											   linestyle='dashed')
			if element2 is not None:
				xl2 = np.arange(0, len(element2)) * 0.1
				curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color='black', linewidth=1,
												   linestyle='dotted')
			if element3 is not None:
				xl3 = np.arange(0, len(element3)) * 0.1
				curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color='black', linewidth=1)
		elif config.result_graph_type == "color":
			xl = np.arange(0, len(element1)) * 0.1
			curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color='black', linewidth=1)

			if element2 is not None:
				xl2 = np.arange(0, len(element2)) * 0.1
				curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color='blue', linewidth=1)
			if element3 is not None:
				xl3 = np.arange(0, len(element3)) * 0.1
				curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color='red', linewidth=1)
	ax_element1.legend()
	ax_element1.legend(loc='upper right')
	if config.show_result_figure is True:
		plt.show()
	if config.save_result_figure is True:
		plt.savefig(
			config.result_path + "vaj-" + str(config.type) + '-time{}.png'.format(
				time.strftime("%Y%m%d-%H%M%S", time.localtime())),
			dpi=600, format="png")


def result_path_graph(config, mode, odomx, odomy, element=None):
	plt.rcParams["axes.labelsize"] = 14
	plt.rcParams["xtick.labelsize"] = 14
	plt.rcParams["ytick.labelsize"] = 14
	if config.result_graph_type == "gray":
		clist = ['#C8C8C8', 'black']
	else:
		clist = [[0, 1, 0, 0.1], 'red']
	fig = plt.figure(figsize=(8, 3.0), tight_layout=False)
	img = plt.imread(config.map)
	ax = fig.add_subplot(111)
	divider = make_axes_locatable(ax)
	cax = divider.append_axes('right', size='5%', pad=0.5)
	newcm = LinearSegmentedColormap.from_list('chaos', clist)
	abs_element = np.abs(element)
	scatter_size = []
	for i in range(len(odomx)):
		if i % 50 == 0:
			scatter_size.append(30)
		else:
			scatter_size.append(5)

	mappable = ax.scatter(x=odomx, y=odomy, c=abs_element, label='Velocity', s=scatter_size, cmap=newcm, vmin=0,
						  vmax=0.1)
	ax.set_aspect("equal")
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_xlim(-22.025 * 1.0, 22.025 * 1.0)
	ax.set_ylim(-12.4 * 0.7, 12.4 * 0.7)
	ax.set(facecolor=[0.5, 0.5, 0.5])
	ax.imshow(img, extent=[-22.025, 22.025, -12.4, 12.4])
	pp = fig.colorbar(mappable, cax=cax, label='Jerk [' + 'm/$ \mathit{s}^{3}$' + ']')
	if config.show_result_figure is True:
		plt.show()
	if config.save_result_figure is True:
		plt.savefig(
			config.result_path + "path-" + str(config.type) + '-time{}.png'.format(
				time.strftime("%Y%m%d-%H%M%S", time.localtime())),
			dpi=600, format="png")


def save_process_time(config, obs):
	if config.save_process_time is True:
		kyo = time.strftime("%Y%m%d", time.localtime())
		f = open(config.result_path + "process time-" + str(config.type) + '-time' + str(kyo) + '.txt', "a")
		f.write('\n')
		f.write(str(config.dt) + ',' + str(config.type) + ',' + str(obs.laser_nums) + ',')
		f.write(str(len(config.cal_time1)) + ',')
		f.write(str(np.nanmean(config.cal_time1)) + ',')
		f.write(str(np.std(config.cal_time1)) + ',')
		f.write('\n')
		# profiler.print(file=f)
		f.close()
	print("frames =", len(config.cal_time1))
	print("average process time =", np.nanmean(config.cal_time1))


def end_point_range(config):
	plt.rcParams["axes.labelsize"] = 14
	plt.rcParams["xtick.labelsize"] = 14
	plt.rcParams["ytick.labelsize"] = 14

	fig = plt.figure(tight_layout=True)

	ax = fig.add_subplot(111)
	ax.set_aspect(1)
	xinit = [0, 0, 0, 1.0, 0]
	dw = calcDynamicWindow(config, xinit)
	print(dw)


	config.v_all_list = [1.0]
	config.a_all_list = [0.0]
	jinduanx = []
	jinduany = []
	yuanduanx = []
	yuanduany = []
	jizhunx = []
	jizhuny = []
	for v in np.arange(dw[0], dw[1] + 0.5 * config.linear_velocity_resolution, config.linear_velocity_resolution):
		for w in np.arange(dw[2], dw[3] + 0.5 * config.angular_velocity_resolution,
						   config.angular_velocity_resolution):
			linear_velocity_list = [v] * int(config.predict_time / config.dt)
			angular_velocity_list = [w] * int(config.predict_time / config.dt)

			traj = calcTrajectory(config, xinit, linear_velocity_list, angular_velocity_list)
			if v == dw[0]:
				jinduanx.append(traj[-1:, 0])
				jinduany.append(traj[-1:, 1])
			elif v == dw[1]:
				yuanduanx.append(traj[-1:, 0])
				yuanduany.append(traj[-1:, 1])
	plt.plot(jinduanx, jinduany, c='b', linewidth=1, linestyle='dashed', label='Origianl method')
	plt.plot(yuanduanx, yuanduany, c='b', linewidth=1, linestyle='dashed')
	plt.show()


def generate_obstacle(radius,nums):
	obstx = []
	obsty = []
	shift = 0.2
	step = (math.pi-2*shift)/(nums-1)
	for i in np.arange(0.0+shift, math.pi-shift+0.5*step,step):
		# x = (1.5)*math.sin(i)
		# y = (1.5)*(1-math.cos(i))-(0.5)
		x = (radius) * math.sin(i)
		y = (radius) * (1 - math.cos(i)) - radius
		obstx.append(x)
		obsty.append(y)
	return obstx, obsty


def compare_dist2():
	plt.rcParams["axes.labelsize"] = 14
	plt.rcParams["xtick.labelsize"] = 14
	plt.rcParams["ytick.labelsize"] = 14
	config = Config()
	xinit = [0, 0, 0, 1.0, 0]
	obstx, obsty = generate_obstacle(4.0, 11)

	distlist1 = []
	distlist21 = []
	distlist22 = []
	distlist23 = []
	distlist3 = []

	v0 = 1.0
	av = -1
	w0 = 1.0
	vn = v0 + av * config.predict_time
	if av > 0:
		vn = min(config.max_linear_velocity,v0 + av * config.predict_time)
	elif av < 0:
		vn = max(config.min_linear_velocity,v0 + av * config.predict_time)
	else:
		vn = v0 + av * config.predict_time

	x = sympy.Symbol('x')
	t = sympy.Symbol('t')
	if av > 0:
		acct1 = (config.max_linear_velocity - v0) / av
		v = sympy.Piecewise((v0 + av * t, t <= acct1), (config.max_linear_velocity, t > acct1))
	elif av < 0:
		acct1 = abs(v0 / av)
		v = sympy.Piecewise((v0 + av * t, t <= acct1), (0, t > acct1))
	else:
		v = v0
	theta = w0 * t
	f_ = (v * sympy.cos(theta))
	g_ = (v * sympy.sin(theta))
	f = (av * sympy.cos(w0 * t) + w0 * (v0 + av * t) * sympy.sin(w0 * t)) / (w0 ** 2)
	g = (av * sympy.sin(w0 * t) - w0 * (v0 + av * t) * sympy.cos(w0 * t)) / (w0 ** 2)

	dx = f.evalf(subs={t: 0}) - 0
	dy = g.evalf(subs={t: 0}) - 0
	f1 = f - dx
	g1 = g - dy
	f2 = config.max_linear_velocity * sympy.sin(w0 * t) / w0
	g2 = -config.max_linear_velocity * sympy.cos(w0 * t) / w0
	f3 = 0
	g3 = 0
	if av > 0:
		acct1 = (config.max_linear_velocity - v0) / av
		x1 = f1.evalf(subs={t: acct1})
		y1 = g1.evalf(subs={t: acct1})
		x2 = f2.evalf(subs={t: acct1})
		y2 = g2.evalf(subs={t: acct1})
		dx1 = x2 - x1
		dy1 = y2 - y1
		f = sympy.Piecewise((f1, t <= acct1)
							, (f2 - dx1, t > acct1))
		g = sympy.Piecewise((g1, t <= acct1)
							, (g2 - dy1, t > acct1))
	elif av < 0:
		acct1 = abs(v0 / av)
		x1 = f1.evalf(subs={t: acct1})
		y1 = g1.evalf(subs={t: acct1})
		x3 = 0
		y3 = 0
		dx2 = x3 - x1
		dy2 = y3 - y1
		f = sympy.Piecewise((f1, t <= acct1)
							, (f3 - dx2, t > acct1))
		g = sympy.Piecewise((g1, t <= acct1)
							, (g3 - dy2, t > acct1))
	else:
		f = v0 * sympy.sin(w0 * t) / w0 - dx
		g = -v0 * sympy.cos(w0 * t) / w0 - dy

	def ff(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_linear_velocity - v0) / av
				x0 = (av * math.cos(w0 * 0) + w0 * (v0 + av * 0) * math.sin(w0 * 0)) / (w0 ** 2)
				dx1 = x0 - 0
				if 0 <= t <= acct:
					f = (av * math.cos(w0 * t) + w0 * (v0 + av * t) * math.sin(w0 * t)) / (w0 ** 2) - dx1
				elif acct < t <= config.predict_time:
					x1 = (av * math.cos(w0 * acct) + w0 * (v0 + av * acct) * math.sin(w0 * acct)) / (w0 ** 2) - dx1
					x2 = config.max_linear_velocity * math.sin(w0 * acct) / w0
					dx2 = x2 - x1
					f = config.max_linear_velocity * math.sin(w0 * t) / w0 - dx2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0 / av)
				x0 = (av * math.cos(w0 * 0) + w0 * (v0 + av * 0) * math.sin(w0 * 0)) / (w0 ** 2)
				dx1 = x0 - 0
				if 0 <= t <= acct:
					f = (av * math.cos(w0 * t) + w0 * (v0 + av * t) * math.sin(w0 * t)) / (w0 ** 2) - dx1
				elif acct < t <= config.predict_time:
					x1 = (av * math.cos(w0 * acct) + w0 * (v0 + av * acct) * math.sin(w0 * acct)) / (w0 ** 2) - dx1
					x2 = 0
					dx2 = x2 - x1
					f = 0 - dx2
			else:
				x0 = v0 * math.sin(w0 * 0) / w0
				dx1 = x0 - 0
				f = v0 * math.sin(w0 * t) / w0 - dx1
		else:
			if av > 0:
				acct = (config.max_linear_velocity - v0) / av
				if 0 <= t <= acct:
					f = v0 * t + 0.5 * av * t ** 2
				elif acct < t <= config.predict_time:
					f = (v0 * acct + 0.5 * av * acct ** 2) + (config.predict_time - acct) + config.max_linear_velocity
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0 / av)
				if 0 <= t <= acct:
					f = v0 * t + 0.5 * av * t ** 2
				elif acct < t <= config.predict_time:
					f = v0 * acct + 0.5 * av * acct ** 2
				else:
					print('out of range')
			else:
				f = v0 * t
		return f

	def gg(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_linear_velocity - v0) / av
				y0 = (av * math.sin(w0 * 0) - w0 * (v0 + av * 0) * math.cos(w0 * 0)) / (w0 ** 2)
				dy1 = y0 - 0
				if 0 <= t <= acct:
					g = (av * math.sin(w0 * t) - w0 * (v0 + av * t) * math.cos(w0 * t)) / (w0 ** 2) - dy1
				elif acct < t <= config.predict_time:
					y1 = (av * math.sin(w0 * acct) - w0 * (v0 + av * acct) * math.cos(w0 * acct)) / (w0 ** 2) - dy1
					y2 = -config.max_linear_velocity * math.cos(w0 * acct) / w0
					dy2 = y2 - y1
					g = -config.max_linear_velocity * math.cos(w0 * t) / w0 - dy2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0 / av)
				y0 = (av * math.sin(w0 * 0) - w0 * (v0 + av * 0) * math.cos(w0 * 0)) / (w0 ** 2)
				dy1 = y0 - 0
				if 0 <= t <= acct:
					g = (av * math.sin(w0 * t) - w0 * (v0 + av * t) * math.cos(w0 * t)) / (w0 ** 2) - dy1
				elif acct < t <= config.predict_time:
					y1 = (av * math.sin(w0 * acct) - w0 * (v0 + av * acct) * math.cos(w0 * acct)) / (w0 ** 2) - dy1
					y2 = 0
					dy2 = y2 - y1
					g = 0 - dy2
			else:
				y0 = -v0 * sympy.cos(w0 * 0) / w0
				dy1 = y0 - 0
				g = -v0 * sympy.cos(w0 * t) / w0 - dy1
		else:
			g = 0
		return g

	config.dt = 0.1
	linear_velocity_list1 = accelerationToVelocity(config, av, "av")
	angular_velocity_list1 = [w0] * int(config.predict_time / config.dt)
	traj1 = calcTrajectory(config, xinit, linear_velocity_list1, angular_velocity_list1)

	config.dt = 0.05
	linear_velocity_list2 = accelerationToVelocity(config, av, "av")
	angular_velocity_list2 = [w0] * int(config.predict_time / config.dt)
	traj2 = calcTrajectory(config, xinit, linear_velocity_list2, angular_velocity_list2)

	config.dt = 0.02
	linear_velocity_list3 = accelerationToVelocity(config, av, "av")
	angular_velocity_list3 = [w0] * int(config.predict_time / config.dt)
	traj3 = calcTrajectory(config, xinit, linear_velocity_list3, angular_velocity_list3)

	config.dt = 0.01
	trajrx=[]
	trajry=[]
	for i in np.arange(0, config.predict_time+config.dt, config.dt):
		trajrx.append(ff(i))
		trajry.append(gg(i))

	fig = plt.figure(tight_layout=True)
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")
	#plt.plot(-traj1[:, 1], traj1[:, 0], c='blue', label='0.1')
	#plt.plot(-traj2[:, 1], traj2[:, 0], c='red', label='0.05 ')
	#plt.plot(-traj3[:, 1], traj3[:, 0], label='0.02')
	plt.plot(-np.array(trajry), np.array(trajrx), c='black', label='real path')
	plt.scatter(obsty, obstx)
	plt.xlabel('y [m]')
	plt.ylabel('x [m]')
	#plt.legend()
	plt.show()

	traj1x = []
	traj1y = []
	traj2x = []
	traj2y = []
	traj3x = []
	traj3y = []
	for i in np.arange(0, config.predict_time, config.dt):
		traj1x.append(ff(i))
		traj1y.append(gg(i))
		if w0 != 0:
			traj2x.append((v0 / w0) * math.sin(w0 * i))
			traj3x.append((vn / w0) * math.sin(w0 * i))
			traj2y.append((v0 / w0) * (1 - math.cos(w0 * i)))
			traj3y.append((vn / w0) * (1 - math.cos(w0 * i)))
		else:
			traj2x.append(v0 * i)
			traj3x.append(vn * i)
			traj2y.append(0)
			traj3y.append(0)

	print("traj1x=", len(traj1x))
	if w0 != 0:
		k = g_.evalf(subs={t: config.predict_time}) / f_.evalf(subs={t: config.predict_time})
		k = -1 / k
	else:
		k = 0
	print(k)
	b = traj1y[-1] - k * traj1x[-1]
	print(b)
	anglemax = math.atan2(traj1y[-1] - b, traj1x[-1] - 0)
	print(anglemax)
	print(math.degrees(anglemax))
	dx21 = traj2x[-1] - traj1x[-1]
	dy21 = traj2y[-1] - traj1y[-1]
	dx31 = traj3x[-1] - traj1x[-1]
	dy31 = traj3y[-1] - traj1y[-1]
	if av != 0:
		n = len(traj1x) * (math.sqrt((v0 ** 2 + vn ** 2) * 0.5) - v0) / av
	else:
		n = 0.5 * config.predict_time
	print("n=", n)
	print("len n=", len(traj2x))
	kn = g_.evalf(subs={t: (n / len(traj1x))}) / f_.evalf(subs={t: (n / len(traj1x))})
	print('kn=', math.degrees(kn))
	nn = kn * config.dt * len(traj1x) / w0
	print('kn/w0=', len(traj1x) * kn / w0)
	dx41 = traj2x[int(n)] - traj1x[int(n)]
	dy41 = traj2y[int(n)] - traj1y[int(n)]
	dx51 = traj3x[int(n)] - traj1x[int(n)]
	dy51 = traj3y[int(n)] - traj1y[int(n)]
	traj4x = np.array(traj2x) - dx21
	traj4y = np.array(traj2y) - dy21

	traj5x = np.array(traj3x) - dx31
	traj5y = np.array(traj3y) - dy31

	traj6x = np.array(traj2x) - dx41
	traj6y = np.array(traj2y) - dy41

	traj7x = np.array(traj3x) - dx51
	traj7y = np.array(traj3y) - dy51

	# fig = plt.figure()
	fig = plt.figure(figsize=(5.25, 5.25))
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")

	plt.plot(traj2x, traj2y, label='Inside circle', c='blue', linestyle='solid')
	plt.plot(traj4x, traj4y, c='blue', linestyle='solid')
	#plt.plot(traj6x, traj6y, c='blue', linestyle='solid')
	plt.plot(traj3x, traj3y, label='Outside circle', c='red', linestyle='solid')
	plt.plot(traj5x, traj5y, c='red', linestyle='solid')
	#plt.plot(traj7x, traj7y, c='red', linestyle='solid')
	# plt.plot(traj8x,traj8y,label='a = 0.5'+'m/$ \mathit{s}^{2}$',c='black',linestyle='dotted')
	plt.plot(traj1x, traj1y, label='Path', c='black', linestyle='solid')
	#plt.show()

	if w0 != 0:
		c2 = [0, v0 / w0]
		c3 = [0, vn / w0]
		c4 = [0 - dx21, v0 / w0 - dy21]#end
		c5 = [0 - dx31, vn / w0 - dy31]#end
		c6 = [0 - dx41, v0 / w0 - dy41]#mid
		c7 = [0 - dx51, vn / w0 - dy51]#mid
		for i in range(len(obstx)):
			xend = ff(config.predict_time)
			yend = gg(config.predict_time)
			print('===========')
			print('obstacle number', i)
			print('obstacle coordinate', obstx[i], obsty[i])

			r2 = math.sqrt((obstx[i] - c2[0]) ** 2 + (obsty[i] - c2[1]) ** 2)
			r3 = math.sqrt((obstx[i] - c3[0]) ** 2 + (obsty[i] - c3[1]) ** 2)
			r4 = math.sqrt((obstx[i] - c4[0]) ** 2 + (obsty[i] - c4[1]) ** 2)
			r5 = math.sqrt((obstx[i] - c5[0]) ** 2 + (obsty[i] - c5[1]) ** 2)
			r6 = math.sqrt((obstx[i] - c6[0]) ** 2 + (obsty[i] - c6[1]) ** 2)
			r7 = math.sqrt((obstx[i] - c7[0]) ** 2 + (obsty[i] - c7[1]) ** 2)

			#print(r2, r3, r4, r5, r6, r7)
			#print(abs(r2 - (v0 / w0)), abs(r3 - (vn / w0)), abs(r4 - (v0 / w0)), abs(r5 - (vn / w0)),
			#	  abs(r6 - (v0 / w0)), abs(r7 - (vn / w0)))
			angle = math.atan2(obsty[i] - b, obstx[i] - 0)
			#print(math.degrees(angle))

			#print(max(abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0)))))
			#print(min(abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0)))))
			if (w0 > 0 and -0.5 * math.pi < angle < anglemax) or (w0 < 0 and anglemax < angle < 0.5 * math.pi):
				if r3 - (vn / abs(w0)) > 0 and r5 - (vn / abs(w0)) > 0 and r7 - (vn / abs(w0)) > 0:
					print('outside')
					if av > 0:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
					else:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				elif r2 - (v0 / abs(w0)) < 0 and r4 - (v0 / abs(w0)) < 0 and r6 - (v0 / abs(w0)) < 0:
					print('inside')
					if av > 0:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

					else:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				else:
					pdist = 0.5 * (
						max(abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + 0.5 * (
								min(abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0)))))
			else:
				dist1 = math.sqrt((obstx[i] - xend) ** 2 + (obsty[i] - yend) ** 2)
				dist2 = math.sqrt((obstx[i]) ** 2 + (obsty[i]) ** 2)
				pdist = min(dist1, dist2)
			distlist3.append(pdist)

			ob = np.array([[obstx[i], obsty[i]]])
			minr1 = np.min(distance.cdist(ob, traj1[:, 0:2], 'euclidean'))
			minr2 = np.min(distance.cdist(ob, traj2[:, 0:2], 'euclidean'))
			minr3 = np.min(distance.cdist(ob, traj3[:, 0:2], 'euclidean'))
			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)


			# h = f_*(list(obst)[0][0]-(f))+g_*(list(obst)[0][1]-(g))
			f_ = sympy.diff(f, t)
			g_ = sympy.diff(g, t)
			h = f_ * (obstx[i] - (f)) + g_ * (obsty[i] - (g))
			t0 = sympy.nsolve(h, t, [0, 1])
			if 0 <= t0 <= config.predict_time:
				xfoot = f.evalf(subs={t: t0})
				yfoot = g.evalf(subs={t: t0})
				dist = math.sqrt((obstx[i] - xfoot) ** 2 + (obsty[i] - yfoot) ** 2)
			else:
				xfoot = f.evalf(subs={t: config.predict_time})
				yfoot = g.evalf(subs={t: config.predict_time})
				dist1 = math.sqrt((obstx[i]) ** 2 + (obsty[i]) ** 2)
				dist2 = math.sqrt((obstx[i] - xfoot) ** 2 + (obsty[i] - yfoot) ** 2)
				dist = min(dist1, dist2)
			distlist1.append(dist)

	else:
		xend = ff(config.predict_time)
		for i in range(len(obstx)):
			print('===========')
			print('obstacle', obstx[i], obsty[i])

			if 0 <= obstx[i] <= xend:
				pdist = obstx[i]
			else:
				dist1 = math.sqrt((obstx[i] - xend) ** 2 + (obsty[i] - 0) ** 2)
				dist2 = math.sqrt((obstx[i]) ** 2 + (obsty[i]) ** 2)
				pdist = min(dist1, dist2)
			distlist3.append(pdist)

			ob = np.array([[obstx[i], obsty[i]]])
			minr1 = np.min(distance.cdist(ob, traj1[:, 0:2], 'euclidean'))
			minr2 = np.min(distance.cdist(ob, traj2[:, 0:2], 'euclidean'))
			minr3 = np.min(distance.cdist(ob, traj3[:, 0:2], 'euclidean'))
			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)

	print("********************************")
	plt.scatter(np.arange(0, len(distlist1)), distlist1, label='Baseline', c='black', marker='o')
	plt.scatter(np.arange(0, len(distlist21)), distlist21, label='Original 0.1', c='blue', marker='v')
	plt.scatter(np.arange(0, len(distlist22)), distlist22, label='Original 0.05')
	plt.scatter(np.arange(0, len(distlist23)), distlist23, label='Original 0.02')
	plt.scatter(np.arange(0, len(distlist3)), distlist3, label='Proposed', c='red', marker='^')

	plt.legend()
	# plt.ylim(0.445,0.505)
	x_major_locator = MultipleLocator(5)
	# y_major_locator=MultipleLocator(0.005)
	ax = plt.gca()
	ax.xaxis.set_major_locator(x_major_locator)
	plt.xlabel("Obstacle Number")
	plt.ylabel("Distance [m]")
	#plt.show()
	print("motion model =", config.motion_model)
	print("acceleration =", av)
	print("max_error of 0.10 =", np.max(np.abs(np.array(distlist21) - np.array(distlist1))))
	print("max_error of 0.05 =", np.max(np.abs(np.array(distlist22) - np.array(distlist1))))
	print("max_error of 0.02 =", np.max(np.abs(np.array(distlist23) - np.array(distlist1))))
	print("max_error of proposed =", np.max(np.abs(np.array(distlist3) - np.array(distlist1))))


def compare_dist3(a,pm):
	plt.rcParams["axes.labelsize"] = 14
	plt.rcParams["xtick.labelsize"] = 14
	plt.rcParams["ytick.labelsize"] = 14
	xinit = [0, 0, 0, 1.0, 0]
	obstx = []
	obsty = []
	with open("point_set2.txt","r") as f:
		for line in f:
			values = line.strip().split()
			obstx.append(float(values[0]))
			obsty.append(float(values[1]))

	distlist1 = []
	distlist21 = []
	distlist22 = []
	distlist23 = []
	distlist3 = []

	v0 = 1.0
	av = a#0.00001
	w0 = 1.0
	vn = v0 + av * config.predict_time
	if av > 0:
		vn = min(config.max_linear_velocity,v0 + av * config.predict_time)
	elif av < 0:
		vn = max(config.min_linear_velocity,v0 + av * config.predict_time)
	else:
		vn = v0 + av * config.predict_time

	x = sympy.Symbol('x')
	t = sympy.Symbol('t')
	if av > 0:
		acct1 = (config.max_linear_velocity - v0) / av
		v = sympy.Piecewise((v0 + av * t, t <= acct1), (config.max_linear_velocity, t > acct1))
	elif av < 0:
		acct1 = abs(v0 / av)
		v = sympy.Piecewise((v0 + av * t, t <= acct1), (0, t > acct1))
	else:
		v = v0
	theta = w0 * t
	f_ = (v * sympy.cos(theta))
	g_ = (v * sympy.sin(theta))
	f = (av * sympy.cos(w0 * t) + w0 * (v0 + av * t) * sympy.sin(w0 * t)) / (w0 ** 2)
	g = (av * sympy.sin(w0 * t) - w0 * (v0 + av * t) * sympy.cos(w0 * t)) / (w0 ** 2)

	dx = f.evalf(subs={t: 0}) - 0
	dy = g.evalf(subs={t: 0}) - 0
	f1 = f - dx
	g1 = g - dy
	f2 = config.max_linear_velocity * sympy.sin(w0 * t) / w0
	g2 = -config.max_linear_velocity * sympy.cos(w0 * t) / w0
	f3 = 0
	g3 = 0
	if av > 0:
		acct1 = (config.max_linear_velocity - v0) / av
		x1 = f1.evalf(subs={t: acct1})
		y1 = g1.evalf(subs={t: acct1})
		x2 = f2.evalf(subs={t: acct1})
		y2 = g2.evalf(subs={t: acct1})
		dx1 = x2 - x1
		dy1 = y2 - y1
		f = sympy.Piecewise((f1, t <= acct1)
							, (f2 - dx1, t > acct1))
		g = sympy.Piecewise((g1, t <= acct1)
							, (g2 - dy1, t > acct1))
	elif av < 0:
		acct1 = abs(v0 / av)
		x1 = f1.evalf(subs={t: acct1})
		y1 = g1.evalf(subs={t: acct1})
		x3 = 0
		y3 = 0
		dx2 = x3 - x1
		dy2 = y3 - y1
		f = sympy.Piecewise((f1, t <= acct1)
							, (f3 - dx2, t > acct1))
		g = sympy.Piecewise((g1, t <= acct1)
							, (g3 - dy2, t > acct1))
	else:
		f = v0 * sympy.sin(w0 * t) / w0 - dx
		g = -v0 * sympy.cos(w0 * t) / w0 - dy

	def ff(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_linear_velocity - v0) / av
				x0 = (av * math.cos(w0 * 0) + w0 * (v0 + av * 0) * math.sin(w0 * 0)) / (w0 ** 2)
				dx1 = x0 - 0
				if 0 <= t <= acct:
					f = (av * math.cos(w0 * t) + w0 * (v0 + av * t) * math.sin(w0 * t)) / (w0 ** 2) - dx1
				elif acct < t <= config.predict_time:
					x1 = (av * math.cos(w0 * acct) + w0 * (v0 + av * acct) * math.sin(w0 * acct)) / (w0 ** 2) - dx1
					x2 = config.max_linear_velocity * math.sin(w0 * acct) / w0
					dx2 = x2 - x1
					f = config.max_linear_velocity * math.sin(w0 * t) / w0 - dx2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0 / av)
				x0 = (av * math.cos(w0 * 0) + w0 * (v0 + av * 0) * math.sin(w0 * 0)) / (w0 ** 2)
				dx1 = x0 - 0
				if 0 <= t <= acct:
					f = (av * math.cos(w0 * t) + w0 * (v0 + av * t) * math.sin(w0 * t)) / (w0 ** 2) - dx1
				elif acct < t <= config.predict_time:
					x1 = (av * math.cos(w0 * acct) + w0 * (v0 + av * acct) * math.sin(w0 * acct)) / (w0 ** 2) - dx1
					x2 = 0
					dx2 = x2 - x1
					f = 0 - dx2
			else:
				x0 = v0 * math.sin(w0 * 0) / w0
				dx1 = x0 - 0
				f = v0 * math.sin(w0 * t) / w0 - dx1
		else:
			if av > 0:
				acct = (config.max_linear_velocity - v0) / av
				if 0 <= t <= acct:
					f = v0 * t + 0.5 * av * t ** 2
				elif acct < t <= config.predict_time:
					f = (v0 * acct + 0.5 * av * acct ** 2) + (config.predict_time - acct) + config.max_linear_velocity
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0 / av)
				if 0 <= t <= acct:
					f = v0 * t + 0.5 * av * t ** 2
				elif acct < t <= config.predict_time:
					f = v0 * acct + 0.5 * av * acct ** 2
				else:
					print('out of range')
			else:
				f = v0 * t
		return f

	def gg(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_linear_velocity - v0) / av
				y0 = (av * math.sin(w0 * 0) - w0 * (v0 + av * 0) * math.cos(w0 * 0)) / (w0 ** 2)
				dy1 = y0 - 0
				if 0 <= t <= acct:
					g = (av * math.sin(w0 * t) - w0 * (v0 + av * t) * math.cos(w0 * t)) / (w0 ** 2) - dy1
				elif acct < t <= config.predict_time:
					y1 = (av * math.sin(w0 * acct) - w0 * (v0 + av * acct) * math.cos(w0 * acct)) / (w0 ** 2) - dy1
					y2 = -config.max_linear_velocity * math.cos(w0 * acct) / w0
					dy2 = y2 - y1
					g = -config.max_linear_velocity * math.cos(w0 * t) / w0 - dy2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0 / av)
				y0 = (av * math.sin(w0 * 0) - w0 * (v0 + av * 0) * math.cos(w0 * 0)) / (w0 ** 2)
				dy1 = y0 - 0
				if 0 <= t <= acct:
					g = (av * math.sin(w0 * t) - w0 * (v0 + av * t) * math.cos(w0 * t)) / (w0 ** 2) - dy1
				elif acct < t <= config.predict_time:
					y1 = (av * math.sin(w0 * acct) - w0 * (v0 + av * acct) * math.cos(w0 * acct)) / (w0 ** 2) - dy1
					y2 = 0
					dy2 = y2 - y1
					g = 0 - dy2
			else:
				y0 = -v0 * sympy.cos(w0 * 0) / w0
				dy1 = y0 - 0
				g = -v0 * sympy.cos(w0 * t) / w0 - dy1
		else:
			g = 0
		return g

	config.dt = 0.1
	linear_velocity_list1 = accelerationToVelocity(config, av, "av")
	angular_velocity_list1 = [w0] * int(config.predict_time / config.dt)
	traj1 = calcTrajectory(config, xinit, linear_velocity_list1, angular_velocity_list1)

	config.dt = 0.05
	linear_velocity_list2 = accelerationToVelocity(config, av, "av")
	angular_velocity_list2 = [w0] * int(config.predict_time / config.dt)
	traj2 = calcTrajectory(config, xinit, linear_velocity_list2, angular_velocity_list2)

	config.dt = 0.02
	linear_velocity_list3 = accelerationToVelocity(config, av, "av")
	angular_velocity_list3 = [w0] * int(config.predict_time / config.dt)
	traj3 = calcTrajectory(config, xinit, linear_velocity_list3, angular_velocity_list3)

	config.dt = 0.01
	trajrx=[]
	trajry=[]
	for i in np.arange(0, config.predict_time+config.dt, config.dt):
		trajrx.append(ff(i))
		trajry.append(gg(i))

	fig = plt.figure(figsize=(8, 4.5),tight_layout=True)
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")
	#plt.plot(-traj1[:, 1], traj1[:, 0], c='blue', label='0.1')
	#plt.plot(-traj2[:, 1], traj2[:, 0], c='red', label='0.05 ')
	#plt.plot(-traj3[:, 1], traj3[:, 0], label='0.02')
	plt.plot(-np.array(trajry), np.array(trajrx), c='black', label='real path')
	plt.scatter(obsty, obstx)
	plt.xlabel('y [m]')
	plt.ylabel('x [m]')
	plt.xlim(-5.2, 5.2)
	plt.ylim(0, 5.2)

	plt.xlabel('y [m]')
	#plt.legend()
	#plt.show()

	traj1x = []
	traj1y = []
	traj2x = []
	traj2y = []
	traj3x = []
	traj3y = []
	for i in np.arange(0, config.predict_time, config.dt):
		traj1x.append(ff(i))
		traj1y.append(gg(i))
		if w0 != 0:
			traj2x.append((v0 / w0) * math.sin(w0 * i))
			traj3x.append((vn / w0) * math.sin(w0 * i))
			traj2y.append((v0 / w0) * (1 - math.cos(w0 * i)))
			traj3y.append((vn / w0) * (1 - math.cos(w0 * i)))
		else:
			traj2x.append(v0 * i)
			traj3x.append(vn * i)
			traj2y.append(0)
			traj3y.append(0)

	print("traj1x=", len(traj1x))
	if av == -1.0:
		tt = 0.99
	else:
		tt = 1.99
	if w0 != 0:
		k = g_.evalf(subs={t: tt}) / f_.evalf(subs={t: tt})
		k = -1 / k
		print("slope", k)
	else:
		k = 0
	print(k)
	b = traj1y[-1] - k * traj1x[-1]
	print(b)
	anglemax = math.atan2(traj1y[-1] - b, traj1x[-1] - 0)
	print(anglemax)
	print(math.degrees(anglemax))
	dx21 = traj2x[-1] - traj1x[-1]
	dy21 = traj2y[-1] - traj1y[-1]
	dx31 = traj3x[-1] - traj1x[-1]
	dy31 = traj3y[-1] - traj1y[-1]
	if av != 0:
		total_length = 0
		for i in range(len(traj2x)-1):
			segment_length = np.sqrt((traj1x[i+1]-traj1x[i])**2+(traj1y[i+1]-traj1y[i])**2)
			total_length += segment_length
		target_length = total_length*0.5
		accumulated_length = 0
		midpoint = None
		for i in range(len(traj2x)-1):
			segment_length = np.sqrt((traj1x[i+1]-traj1x[i])**2+(traj1y[i+1]-traj1y[i])**2)
			accumulated_length += segment_length
			if accumulated_length >= target_length:
				n = i
				break
	else:
		n = 0.5 * config.predict_time

	print("n=", n)
	print("len n=", len(traj2x))
	kn = g_.evalf(subs={t: (n / len(traj1x))}) / f_.evalf(subs={t: (n / len(traj1x))})
	print('kn=', math.degrees(kn))
	nn = kn * config.dt * len(traj1x) / w0
	print('kn/w0=', len(traj1x) * kn / w0)
	dx41 = traj2x[int(n)] - traj1x[int(n)]
	dy41 = traj2y[int(n)] - traj1y[int(n)]
	dx51 = traj3x[int(n)] - traj1x[int(n)]
	dy51 = traj3y[int(n)] - traj1y[int(n)]
	traj4x = np.array(traj2x) - dx21
	traj4y = np.array(traj2y) - dy21

	traj5x = np.array(traj3x) - dx31
	traj5y = np.array(traj3y) - dy31

	traj6x = np.array(traj2x) - dx41
	traj6y = np.array(traj2y) - dy41

	traj7x = np.array(traj3x) - dx51
	traj7y = np.array(traj3y) - dy51

	# fig = plt.figure()
	fig = plt.figure(figsize=(5.25, 5.25))
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")

	plt.plot(traj2x, traj2y, label='Inside circle', c='blue', linestyle='solid')
	plt.plot(traj4x, traj4y, c='blue', linestyle='solid')
	#plt.plot(traj6x, traj6y, c='blue', linestyle='solid')
	plt.plot(traj3x, traj3y, label='Outside circle', c='red', linestyle='solid')
	plt.plot(traj5x, traj5y, c='red', linestyle='solid')
	#plt.plot(traj7x, traj7y, c='red', linestyle='solid')
	# plt.plot(traj8x,traj8y,label='a = 0.5'+'m/$ \mathit{s}^{2}$',c='black',linestyle='dotted')
	plt.plot(traj1x, traj1y, label='Path', c='black', linestyle='solid')
	#plt.show()

	if w0 != 0:
		c2 = [0, v0 / w0]
		c3 = [0, vn / w0]
		c4 = [0 - dx21, v0 / w0 - dy21]#end
		c5 = [0 - dx31, vn / w0 - dy31]#end
		c6 = [0 - dx41, v0 / w0 - dy41]#mid
		c7 = [0 - dx51, vn / w0 - dy51]#mid
		for i in range(len(obstx)):
			xend = ff(config.predict_time)
			yend = gg(config.predict_time)
			#print('===========')
			#print('obstacle number', i)
			#print('obstacle coordinate', obstx[i], obsty[i])
			#print(pm)
			if pm == "sme":
				r2 = math.sqrt((obstx[i] - c2[0]) ** 2 + (obsty[i] - c2[1]) ** 2)#s
				r3 = math.sqrt((obstx[i] - c3[0]) ** 2 + (obsty[i] - c3[1]) ** 2)#s
				r4 = math.sqrt((obstx[i] - c4[0]) ** 2 + (obsty[i] - c4[1]) ** 2)#e
				r5 = math.sqrt((obstx[i] - c5[0]) ** 2 + (obsty[i] - c5[1]) ** 2)#e
				r6 = math.sqrt((obstx[i] - c6[0]) ** 2 + (obsty[i] - c6[1]) ** 2)#m
				r7 = math.sqrt((obstx[i] - c7[0]) ** 2 + (obsty[i] - c7[1]) ** 2)#m
			elif pm == "sm":
				r2 = math.sqrt((obstx[i] - c2[0]) ** 2 + (obsty[i] - c2[1]) ** 2)#s
				r3 = math.sqrt((obstx[i] - c3[0]) ** 2 + (obsty[i] - c3[1]) ** 2)#s
				r6 = math.sqrt((obstx[i] - c6[0]) ** 2 + (obsty[i] - c6[1]) ** 2)#m
				r7 = math.sqrt((obstx[i] - c7[0]) ** 2 + (obsty[i] - c7[1]) ** 2)#m
				r4 = r2
				r5 = r2
			elif pm == "me":
				r4 = math.sqrt((obstx[i] - c4[0]) ** 2 + (obsty[i] - c4[1]) ** 2)#e
				r5 = math.sqrt((obstx[i] - c5[0]) ** 2 + (obsty[i] - c5[1]) ** 2)#e
				r6 = math.sqrt((obstx[i] - c6[0]) ** 2 + (obsty[i] - c6[1]) ** 2)#m
				r7 = math.sqrt((obstx[i] - c7[0]) ** 2 + (obsty[i] - c7[1]) ** 2)#m
				r2 = r4
				r3 = r5
			elif pm == "se":
				r2 = math.sqrt((obstx[i] - c2[0]) ** 2 + (obsty[i] - c2[1]) ** 2)#s
				r3 = math.sqrt((obstx[i] - c3[0]) ** 2 + (obsty[i] - c3[1]) ** 2)#s
				r4 = math.sqrt((obstx[i] - c4[0]) ** 2 + (obsty[i] - c4[1]) ** 2)#e
				r5 = math.sqrt((obstx[i] - c5[0]) ** 2 + (obsty[i] - c5[1]) ** 2)#e
				r6 = r2
				r7 = r3
			elif pm == "s":
				r2 = math.sqrt((obstx[i] - c2[0]) ** 2 + (obsty[i] - c2[1]) ** 2)#s
				r3 = math.sqrt((obstx[i] - c3[0]) ** 2 + (obsty[i] - c3[1]) ** 2)#s
				r4 = r2
				r5 = r3
				r6 = r2
				r7 = r3
			elif pm == "m":
				r6 = math.sqrt((obstx[i] - c6[0]) ** 2 + (obsty[i] - c6[1]) ** 2)#m
				r7 = math.sqrt((obstx[i] - c7[0]) ** 2 + (obsty[i] - c7[1]) ** 2)#m
				r2 = r6
				r3 = r7
				r4 = r6
				r5 = r7
			elif pm == "e":
				r4 = math.sqrt((obstx[i] - c4[0]) ** 2 + (obsty[i] - c4[1]) ** 2)#e
				r5 = math.sqrt((obstx[i] - c5[0]) ** 2 + (obsty[i] - c5[1]) ** 2)#e
				r2 = r4
				r3 = r5
				r6 = r4
				r7 = r3
			angle = math.atan2(obsty[i] - b, obstx[i] - 0)
			#print(r2,r3,r4,r5,r6,r7)
			if (w0 > 0 and -0.5 * math.pi < angle < anglemax) or (w0 < 0 and anglemax < angle < 0.5 * math.pi):
				if r3 - (vn / abs(w0)) > 0 and r5 - (vn / abs(w0)) > 0 and r7 - (vn / abs(w0)) > 0:
					#print('outside')
					if av > 0:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
					else:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				elif r2 - (v0 / abs(w0)) < 0 and r4 - (v0 / abs(w0)) < 0 and r6 - (v0 / abs(w0)) < 0:
					#print('inside')
					if av > 0:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

					else:
						#pdist = ((((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * max(
						#	abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + (
						#					1 - ((angle + 0.5 * math.pi) / (anglemax + 0.5 * math.pi))) * min(
						#	abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0))))
						pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				else:
					pdist = 0.5 * (
						max(abs(r3 - (vn / abs(w0))), abs(r5 - (vn / abs(w0))), abs(r7 - (vn / abs(w0))))) + 0.5 * (
								min(abs(r2 - (v0 / abs(w0))), abs(r4 - (v0 / abs(w0))), abs(r6 - (v0 / abs(w0)))))
			else:
				dist1 = math.sqrt((obstx[i] - xend) ** 2 + (obsty[i] - yend) ** 2)
				dist2 = math.sqrt((obstx[i]) ** 2 + (obsty[i]) ** 2)
				pdist = min(dist1, dist2)
			distlist3.append(pdist)

			ob = np.array([[obstx[i], obsty[i]]])
			minr1 = np.min(distance.cdist(ob, traj1[:, 0:2], 'euclidean'))
			minr2 = np.min(distance.cdist(ob, traj2[:, 0:2], 'euclidean'))
			minr3 = np.min(distance.cdist(ob, traj3[:, 0:2], 'euclidean'))
			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)


			# h = f_*(list(obst)[0][0]-(f))+g_*(list(obst)[0][1]-(g))
			f_ = sympy.diff(f, t)
			g_ = sympy.diff(g, t)
			h = f_ * (obstx[i] - (f)) + g_ * (obsty[i] - (g))
			t0 = sympy.nsolve(h, t, [0, 1])
			if 0 <= t0 <= config.predict_time:
				xfoot = f.evalf(subs={t: t0})
				yfoot = g.evalf(subs={t: t0})
				dist = math.sqrt((obstx[i] - xfoot) ** 2 + (obsty[i] - yfoot) ** 2)
			else:
				xfoot = f.evalf(subs={t: config.predict_time})
				yfoot = g.evalf(subs={t: config.predict_time})
				dist1 = math.sqrt((obstx[i]) ** 2 + (obsty[i]) ** 2)
				dist2 = math.sqrt((obstx[i] - xfoot) ** 2 + (obsty[i] - yfoot) ** 2)
				dist = min(dist1, dist2)
			distlist1.append(dist)

	else:
		xend = ff(config.predict_time)
		for i in range(len(obstx)):
			#print('===========')
			#print('obstacle', obstx[i], obsty[i])

			if 0 <= obstx[i] <= xend:
				pdist = obstx[i]
			else:
				dist1 = math.sqrt((obstx[i] - xend) ** 2 + (obsty[i] - 0) ** 2)
				dist2 = math.sqrt((obstx[i]) ** 2 + (obsty[i]) ** 2)
				pdist = min(dist1, dist2)
			distlist3.append(pdist)

			ob = np.array([[obstx[i], obsty[i]]])
			minr1 = np.min(distance.cdist(ob, traj1[:, 0:2], 'euclidean'))
			minr2 = np.min(distance.cdist(ob, traj2[:, 0:2], 'euclidean'))
			minr3 = np.min(distance.cdist(ob, traj3[:, 0:2], 'euclidean'))
			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)

	print("********************************")
	plt.scatter(np.arange(0, len(distlist1)), distlist1, label='Baseline', c='black', marker='o')
	plt.scatter(np.arange(0, len(distlist21)), distlist21, label='Original 0.1', c='blue', marker='v')
	plt.scatter(np.arange(0, len(distlist22)), distlist22, label='Original 0.05')
	plt.scatter(np.arange(0, len(distlist23)), distlist23, label='Original 0.02')
	plt.scatter(np.arange(0, len(distlist3)), distlist3, label='Proposed', c='red', marker='^')

	plt.legend()
	# plt.ylim(0.445,0.505)
	x_major_locator = MultipleLocator(5)
	# y_major_locator=MultipleLocator(0.005)
	ax = plt.gca()
	ax.xaxis.set_major_locator(x_major_locator)
	plt.xlabel("Obstacle Number")
	plt.ylabel("Distance [m]")
	#plt.show()
	print("motion model =", config.motion_model)
	print("proposed model =", pm)
	print("acceleration =", av)
	print("mean_error of 0.10 =", np.nanmean(np.abs(np.array(distlist21) - np.array(distlist1))))
	print("mean_error of 0.05 =", np.nanmean(np.abs(np.array(distlist22) - np.array(distlist1))))
	print("mean_error of 0.02 =", np.nanmean(np.abs(np.array(distlist23) - np.array(distlist1))))
	print("mean_error of proposed =", np.nanmean(np.abs(np.array(distlist3) - np.array(distlist1))))
	if not os.path.exists(config.result_path):
		os.makedirs(config.result_path)
	f = open(config.result_path + 'static_error_p.txt', "a")
	#f.write(str(config.motion_model) + ','+str(av)+ ',')
	f.write(str(pm) + ','+str(av)+ ',')

	#f.write(str(np.nanmean(np.abs(np.array(distlist21) - np.array(distlist1)))) + ',')
	#f.write(str(np.nanmean(np.abs(np.array(distlist22) - np.array(distlist1)))) + ',')
	#f.write(str(np.nanmean(np.abs(np.array(distlist23) - np.array(distlist1)))) + ',')
	f.write(str(np.nanmean(np.abs(np.array(distlist3) - np.array(distlist1)))))
	f.write('\n')
	f.close()
def point_set():
	center = (0,0)
	radius = 5
	num_points = 200
	points=[]
	for _ in range(num_points):
		r = np.sqrt(np.random.uniform(0,radius**2))
		theta = np.random.uniform(-np.pi,np.pi)
		x = center[0] + r * np.cos(theta)
		y = center[1] + r * np.sin(theta)
		if x > 0:
			points.append((x,y))
	np.savetxt("point_set2.txt",points)

def func(x,a,b):
	return a / x + b
	#return a * np.log(x) +b
def coq():
	plt.rcParams["axes.labelsize"] = 14
	plt.rcParams["xtick.labelsize"] = 14
	plt.rcParams["ytick.labelsize"] = 14

	fig = plt.figure(figsize=(8, 5), tight_layout=True)

	x1 = np.array([35, 37, 45])
	y1 = np.array([0.0438796, 0.0247629, 0.0133887])*100
	x2 = np.array([27, 28, 30])
	y2 = np.array([0.0468112, 0.0133148, 0.0075940])*100
	# 绘制散点图
	plt.scatter(x1, y1, color='black', label='Conventional DWA')
	plt.scatter(x2, y2, color='red', label='Proposed method')
	# 计算趋势线
	params1, params_convariance = curve_fit(func,x1,y1)
	a1,b1 = params1
	x1fit = np.linspace(x1.min(), x1.max(), 100)
	y1fit = func(x1fit,a1,b1)
	params2, params_convariance = curve_fit(func,x2,y2)
	a2,b2 = params2
	x2fit = np.linspace(x2.min(), x2.max(), 100)
	y2fit = func(x2fit,a2,b2)
	plt.plot(x1fit, y1fit, color='black')
	plt.plot(x2fit, y2fit, color='red')

	#coefficients1 = np.polyfit(x1, y1, 1)
	#trendline_x1 = np.linspace(x1.min(), x1.max(), 100)
	#trendline_y1 = np.polyval(coefficients1, trendline_x1)
	#coefficients2 = np.polyfit(x2, y2, 1)
	#trendline_x2 = np.linspace(x2.min(), x2.max(), 100)
	#trendline_y2 = np.polyval(coefficients2, trendline_x2)
	# 绘制趋势线
	#plt.plot(trendline_x1, trendline_y1, color='black')
	#plt.plot(trendline_x2, trendline_y2, color='red')

	# 添加图例和标签
	plt.legend()
	plt.xlabel('Average computation time [ms]')
	plt.ylabel('Average distance estimation error [mm]')
	plt.xlim(0.0, 47.5)
	plt.ylim(0.0, 6)

	# 显示图形
	plt.show()

def line_graph():
	plt.rcParams["axes.labelsize"] = 14
	plt.rcParams["xtick.labelsize"] = 14
	plt.rcParams["ytick.labelsize"] = 14
	x = np.array([20, 40, 100])
	y1 = np.array([59, 38, 26])
	y2 = np.array([33, 25, 21])
	y3 = np.array([39, 46, 54])
	y1_std = np.array([29.5, 21.5, 17.1])
	y2_std = np.array([19.9, 17.1, 15.6])
	y3_std = np.array([3.1,4.2,3.0])

	xp = np.array([2, 4, 6])
	yp = np.array([32, 25, 23])
	yp_std = np.array([25.5, 25.7, 14.9])
	yp2 = np.array([34, 37, 41])
	yp2_std = np.array([3.9,5.3,4.4])

	fig, ax1 = plt.subplots(figsize=(8, 4), tight_layout=True)
	ax2 = ax1.twiny()
	ax1.plot(x, y1,marker="o",linestyle="-",color="black",label="Tangent model")
	ax1.plot(x, y2, marker="o", linestyle="-", color="blue",label="Secant model")
	ax1.errorbar(x, y1, yerr=y1_std,color="black",alpha=0.4)
	ax1.errorbar(x, y2, yerr=y2_std,color="blue",alpha=0.4)
	#ax1.fill_between(x,y1-y1_std,y1+y1_std,alpha=0.2,color="black")
	#ax1.fill_between(x,y2-y2_std,y2+y2_std,alpha=0.2,color="blue")

	ax2.plot(xp,yp,marker="o",linestyle="-",color="red",label="Proposed method")
	ax2.errorbar(xp, yp, yerr=yp_std,color="red",alpha=0.4)

	#ax2.fill_between(xp,yp-yp_std,yp+yp_std,alpha=0.2,color="red")
	ax1.set_xlim(20-80*0.05,100+80*0.05)
	ax1.set_xticks(np.arange(20,101,20))
	ax2.set_xlim(2-4*0.05, 6+4*0.05)
	ax2.set_xticks(np.arange(2, 7, 1))
	ax1.set_xlabel("Number of path points")
	ax2.set_xlabel("Number of circular arcs")
	ax1.set_ylabel("Distance estimation error [mm]")
	lines = ax1.lines+ax2.lines
	labels = [line.get_label() for line in lines]
	plt.legend(lines,labels)
	plt.show()

	fig, ax1 = plt.subplots(figsize=(8, 4), tight_layout=True)
	ax2 = ax1.twiny()
	ax1.plot(x, y3,marker="o",linestyle="-",color="black",label="Conventional method")
	ax1.errorbar(x, y3, yerr=y3_std,color="black",alpha=0.4)
	#ax1.fill_between(x,y3-y3_std,y3+y3_std,alpha=0.2,color="black")

	ax2.plot(xp,yp2,marker="o",linestyle="-",color="red",label="Proposed method")
	ax2.errorbar(xp, yp2, yerr=yp2_std,color="red",alpha=0.4)

	#ax2.fill_between(xp,yp2-yp2_std,yp2+yp2_std,alpha=0.2,color="red")
	ax1.set_xlim(20-80*0.05,100+80*0.05)
	ax1.set_xticks(np.arange(20,101,20))
	ax2.set_xlim(2-4*0.05, 6+4*0.05)
	ax2.set_xticks(np.arange(2, 7, 1))
	ax1.set_xlabel("Number of path points")
	ax2.set_xlabel("Number of circular arcs")
	ax1.set_ylabel("Computation Time [ms]")
	lines = ax1.lines+ax2.lines
	labels = [line.get_label() for line in lines]
	plt.legend(lines,labels)
	plt.show()

	fig, ax = plt.subplots(figsize=(8, 4), tight_layout=True)
	plt.plot(y3,y2,marker="o",linestyle="-",color="black",label="Conventional method")
	plt.plot(yp2, yp,marker="o",linestyle="-",color="red",label="Proposed method")
	ax.errorbar(y3,y2,xerr=y3_std,yerr=y2_std,color="black",alpha=0.4)
	ax.errorbar(yp2, yp,xerr=yp2_std,yerr=yp_std,color="red",alpha=0.4)

	plt.legend()
	plt.xlim(20,60)
	plt.ylim(0, 60)
	plt.xlabel("Computation time [ms]")
	plt.ylabel("Distance estimation error [mm]")
	plt.show()


def main():
	print(__file__ + " start!!")
	config = Config()
	config.path = ast(config)
	obs = Obstacles()

	subOdom = rospy.Subscriber("odom", Odometry, config.assignOdomCoords)
	subLaser = rospy.Subscriber("/base_scan", LaserScan, obs.assignObs, config)
	subCmd = rospy.Subscriber("odom", Odometry, config.callcmd)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	speed = Twist()

	# initial linear and angular velocities
	u = np.array([0.0, 0.0])
	t = 0
	if not os.path.exists(config.result_path):
		os.makedirs(config.result_path)


if __name__ == '__main__':
	rospy.init_node('dwa')
	# main()
	config = Config()
	parser = argparse.ArgumentParser()
	parser.add_argument("--model", type=str,default="tangent")
	parser.add_argument("--av", type=float, default=1.0)
	parser.add_argument("--pm", type=str, default="sme")

	args = parser.parse_args()
	config.motion_model = args.model
	a = args.av
	pm = args.pm
	print(args.model)
	print(config.motion_model)
	print(pm)

#end_point_range(config)
	#point_set()
	#compare_dist2()
	#compare_dist3(a,pm)
	line_graph()
	#coq()
	#cv2.imshow("image",img)

