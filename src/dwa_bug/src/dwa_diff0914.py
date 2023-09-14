#!/usr/bin/env python

# Author: Connor McGuile
# Feel free to use in any way.

# A custom Dynamic Window Approach implementation for use with Turtlebot.
# Obstacles are registered by a front-mounted laser and stored in a set.
# If, for testing purposes or otherwise, you do not want the laser to be used,
# disable the laserscan subscriber and create your own obstacle set in main(),
# before beginning the loop. If you do not want obstacles, create an empty set.
# Implentation based off Fox et al."s paper, The Dynamic Window Approach to
# Collision Avoidance (1997).

import cv2
import rospy
import math
import numpy as np
import os
import datetime
import time
import argparse
import matplotlib as mpl
import sympy
import mpmath as mp
import random

from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
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


class Config:
	# simulation parameters
	def __init__(self):
		self.dt = 0.10  # [s]
		self.predict_time = 2.0

		# robot parameter
		self.max_linear_velocity = 2.0  # [m/s]
		self.min_linear_velocity = 0.0

		self.max_angular_velocity = 6.28  # [rad/s]360 degree
		self.min_angular_velocity = -6.28

		self.max_linear_acceleration = 1.0  # 2.0  # [m/ss]
		self.max_angular_acceleration = 5.24  # [rad/ss]300 degree

		self.max_linear_jerk = 0.5  # [m/sss]
		self.max_angular_jerk = 2.27  # [m/sss]

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

		self.predicted_x = self.x
		self.predicted_y = self.y
		self.predicted_th = self.th

		self.local_goal_index = 0

		self.current_v = 0.0
		self.current_w = 0.0
		self.sampled_v = 0.0
		self.sampled_w = 0.0
		self.r = rospy.Rate(10)

		self.error = []

		self.linear_velocity_list = [0.0]
		self.angular_velocity_list = [0.0]
		self.linear_acceleration_list = [0.0]
		self.angular_acceleration_list = [0.0]
		self.linear_jerk_list = [0.0]
		self.angular_jerk_list = [0.0]

		self.right_velocity_list = [0.0]
		self.left_velocity_list = [0.0]

		self.right_acceleration_list = [0.0]
		self.left_acceleration_list = [0.0]

		self.right_jerk_list = [0.0]
		self.left_jerk_list = [0.0]

		self.support_linear_velocity_list = [0.0]

		self.odom_x_list = [self.startX]
		self.odom_y_list = [self.startY]
		self.odom_th_list = [self.th]

		self.cal_time1 = []
		self.cal_time2 = []
		self.cal_time3 = []

		self.ob_nums = []
		self.path = []
		self.remain_path = self.path

		self.stop_dist = 1.0

		self.v_weight = 0.0
		self.jv_weight = 0.10
		self.jw_weight = 0.10

		self.show_path_figure = False

		self.type = "a"  # dwa a a2 j
		self.map_name = "e2"
		self.map = "../world/" + self.map_name + ".png"
		self.result_path = "../results/{}/{}/{}/".format(time.strftime("%Y%m%d", time.localtime()), self.type,
														 self.map_name)

	def __readMap(self):
		img = cv2.imread(self.map, 0)
		x, y = img.shape[0:2]
		img_one_of_four = cv2.resize(img, (int(y / 10), int(x / 10)))
		kernel = np.ones((15, 15), np.uint8)
		img_one_of_four = cv2.erode(img_one_of_four, kernel, iterations=1)
		return img_one_of_four

	def assignOdomCoords(self, msg):
		# X- and Y- coords and pose of robot fed back into the robot config
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		r = R.from_quat([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		(roll, pitch, theta) = r.as_euler("xyz", degrees=False)
		self.th = theta

	def callcmd(self, msg):
		self.current_v = msg.twist.twist.linear.x
		self.current_w = msg.twist.twist.angular.z

	def aStar(self):
		solve = solver()
		img = self.__readMap()
		block_img = ~img
		grid = block_img
		print(grid.shape)
		img_h, img_w = grid.shape

		dx = img_w * 0.5
		dy = img_h * 0.5
		ds = 0.1
		st = (self.startX, self.startY)
		ed = (self.goalX, self.goalY)
		start = (int((st[1] / ds) + dy), int((st[0] / ds) + dx))
		print("start point:{}".format(start))
		end = (int((ed[1] / ds) + dy), int((ed[0] / ds) + dx))
		print("end point:{}".format(end))
		route = solve.astar(start, end, grid)
		if not route:
			print("No path")
			return 0
		route += [start]
		route = route[::-1]
		path = []
		for i in route:
			px = (dy - i[0]) * ds
			py = (i[1] - dx) * ds
			path.append((py, px))
		print("=================")
		xc = []
		yc = []
		for i in (range(0, len(route))):
			x = route[i][0]
			y = route[i][1]
			xc.append(x)
			yc.append(y)
		if self.show_path_figure is True:
			fig, ax = plt.subplots()
			ax.imshow(grid, cmap=plt.cm.Spectral)
			ax.plot(yc, xc, color="black")
			ax.scatter(start[1], start[0])
			ax.scatter(end[1], end[0])
			plt.show()
		self.path = path

	# return path

	def motion(self):
		# x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
		current_state = [self.predicted_x, self.predicted_y, self.predicted_th, self.sampled_v, self.sampled_w]
		kinematic_model_matrix = np.array([[1, 0, 0, self.dt * math.cos(current_state[2]), 0],
										   [0, 1, 0, self.dt * math.sin(current_state[2]), 0],
										   [0, 0, 1, 0, self.dt],
										   [0, 0, 0, 1, 0],
										   [0, 0, 0, 0, 1]])
		current_state = np.dot(kinematic_model_matrix, current_state)
		return current_state

	def atGoal(self):
		if math.sqrt((self.x - self.goalX) ** 2 + (self.y - self.goalY) ** 2) <= self.stop_dist:
			if self.current_v <= 0.05:  # 0.1 * config.max_linear_velocity:
				return True
		return False


class Obstacles:
	def __init__(self):
		# Set of coordinates of obstacles in view
		self.obst = set()
		self.obst_rc = set()
		self.laser_nums = 0

	def assignObs(self, msg, config):
		deg = len(msg.ranges)
		self.laser_nums = len(msg.ranges)
		self.obst = []  # reset the obstacle list to only keep visible objects
		self.obst_rc = []
		transform_matrix = np.array([[math.cos(config.th), -math.sin(config.th), 0, config.x],
									 [math.sin(config.th), math.cos(config.th), 0, config.y],
									 [0, 0, 1, 0],
									 [0, 0, 0, 1]])
		for angle, distance_from_obstacle in enumerate(msg.ranges):
			if distance_from_obstacle < config.sensor_range:
				angle_in_sensor = -math.pi + angle * msg.angle_increment
				obstacle_in_robot_coordinate = np.array([distance_from_obstacle * math.cos(angle_in_sensor),
														 distance_from_obstacle * math.sin(angle_in_sensor),
														 0, 1])
				obstacle_in_world_coordinate = np.dot(transform_matrix, obstacle_in_robot_coordinate)
				self.obst.append((obstacle_in_world_coordinate[0], obstacle_in_world_coordinate[1]))
				self.obst_rc.append((distance_from_obstacle * math.cos(angle_in_sensor),
									 distance_from_obstacle * math.sin(angle_in_sensor)))

		if len(self.obst) != len(self.obst_rc):
			print("not equal")
		if isinstance(config.path, list):
			if len(config.path) == 0:
				config.localX = config.goalX
				config.localY = config.goalY
			else:
				squared_distances = [(point[0] - config.x) ** 2 + (point[1] - config.y) ** 2 for point in config.path]
				indices_to_remove = [i for i, d in enumerate(squared_distances) if
									 math.sqrt(d) < config.sensor_range and
									 math.sqrt(d) < msg.ranges[
										 int(np.rad2deg(math.atan2(config.path[i][1], config.path[i][0])))]]
				config.path = [point for i, point in enumerate(config.path) if i not in indices_to_remove]

				if len(config.path) == 0:
					config.localX = config.goalX
					config.localY = config.goalY
				else:
					config.localX = config.path[0][0]
					config.localY = config.path[0][1]


class AfterProcess(Config):
	def __init__(self):
		super().__init__()
		self.show_result_figure = False
		self.result_graph_type = "gray"  # gray or color
		self.show_pyinstrument = False
		self.save_result_figure = True
		self.save_process_time = True

	def result_graph(self, mode, element1, element2=None, element3=None):
		print(self.show_result_figure)
		plt.rcParams["axes.labelsize"] = 14
		plt.rcParams["xtick.labelsize"] = 14
		plt.rcParams["ytick.labelsize"] = 14

		fig = plt.figure(figsize=(8, 3.0), tight_layout=True)
		ax_element1 = HostAxes(fig, [0.10, 0.15, 0.65, 0.8])
		ax_element2 = ParasiteAxes(ax_element1, sharex=ax_element1)
		ax_element3 = ParasiteAxes(ax_element1, sharex=ax_element1)

		ax_element1.parasites.append(ax_element2)
		ax_element1.parasites.append(ax_element3)

		ax_element1.axis["right"].set_visible(False)
		ax_element1.axis["top"].set_visible(False)
		ax_element2.axis["right"].set_visible(True)
		ax_element2.axis["right"].major_ticklabels.set_visible(True)
		ax_element2.axis["right"].label.set_visible(True)

		element3_axisline = ax_element3.get_grid_helper().new_fixed_axis
		ax_element3.axis["right2"] = element3_axisline(loc="right", axes=ax_element3, offset=(70, 0))

		fig.add_axes(ax_element1)

		ax_element2.axis["right"].label.set_color("black")
		ax_element3.axis["right2"].label.set_color("black")

		ax_element2.axis["right"].major_ticks.set_color("black")
		ax_element3.axis["right2"].major_ticks.set_color("black")

		ax_element2.axis["right"].major_ticklabels.set_color("black")
		ax_element3.axis["right2"].major_ticklabels.set_color("black")

		ax_element2.axis["right"].line.set_color("black")
		ax_element3.axis["right2"].line.set_color("black")
		if mode == "vaj":
			ax_element1.set_ylabel("Velocity [m/s]")
			ax_element1.set_xlabel("Time [s]")
			ax_element2.set_ylabel("Velocity [rad/s]")
			ax_element3.set_ylabel("Jerk [" + "m/$ \mathrm{s}^{3}$" + "]")
			ax_element1.set_ylim(0.0, self.max_linear_velocity + 0.5)
			ax_element2.set_ylim(-self.max_angular_velocity * 1.05, self.max_angular_velocity * 1.05)
			ax_element3.set_ylim(-self.max_linear_acceleration * 20, self.max_linear_acceleration * 20)
			# ax_element1.parasites.remove(ax_element2)
			ax_element1.parasites.remove(ax_element3)
			# ax_element2 = None
			# ax_element3 = None
			if self.result_graph_type == "gray":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Linear velocity", color="black", linewidth=1,
												   linestyle="solid")
				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Angular velocity", color="black",
													   linewidth=1,
													   linestyle="dashed")
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="black", linewidth=1)
			elif self.result_graph_type == "color":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color="black", linewidth=1)

				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color="blue", linewidth=1)
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="red", linewidth=1)
		if mode == "waj":
			ax_element1.set_ylabel("Velocity [rad/s]")
			ax_element1.set_xlabel("Time [s]")
			ax_element2.set_ylabel("Acceleration [" + "rad/$ \mathrm{s}^{2}$" + "]")
			ax_element3.set_ylabel("Jerk [" + "rad/$ \mathrm{s}^{3}$" + "]")
			ax_element1.set_ylim(-self.max_angular_velocity - 0.5, self.max_angular_velocity + 0.5)
			ax_element2.set_ylim(-self.max_angular_acceleration * 2, self.max_angular_acceleration * 2)
			ax_element3.set_ylim(-self.max_angular_acceleration * 20, self.max_angular_acceleration * 20)
			ax_element1.parasites.remove(ax_element2)
			ax_element1.parasites.remove(ax_element3)
			if self.result_graph_type == "gray":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color="black", linewidth=1,
												   linestyle="solid")
				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color="black", linewidth=1,
													   linestyle="dotted")
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="black", linewidth=1)
			elif self.result_graph_type == "color":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color="black", linewidth=1)

				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color="blue", linewidth=1)
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="red", linewidth=1)
		if mode == "right":
			ax_element1.set_ylabel("Velocity [m/s]")
			ax_element1.set_xlabel("Time [s]")
			ax_element2.set_ylabel("Acceleration [" + "m/$ \mathrm{s}^{2}$" + "]")
			ax_element3.set_ylabel("Jerk [" + "m/$ \mathrm{s}^{3}$" + "]")
			# ax_element1.set_ylim(-1.0, config.max_linear_velocity + 0.5)
			# ax_element2.set_ylim(-config.max_linear_acceleration * 2.5, config.max_linear_acceleration * 2.5)
			# ax_element3.set_ylim(-config.max_linear_acceleration * 45, config.max_linear_acceleration * 45)
			ax_element1.set_ylim(-1.0, 2.5)
			ax_element2.set_ylim(-2, 5)
			ax_element3.set_ylim(-20, 50)
			if self.result_graph_type == "gray":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color="black", linewidth=1,
												   linestyle="dashed")
				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color="black", linewidth=1,
													   linestyle="dotted")
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="black", linewidth=1)
			elif self.result_graph_type == "color":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color="black", linewidth=1)

				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color="blue", linewidth=1)
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="red", linewidth=1)

		if mode == "left":
			ax_element1.set_ylabel("Velocity [m/s]")
			ax_element1.set_xlabel("Time [s]")
			ax_element2.set_ylabel("Acceleration [" + "m/$ \mathrm{s}^{2}$" + "]")
			ax_element3.set_ylabel("Jerk [" + "m/$ \mathrm{s}^{3}$" + "]")
			# ax_element1.set_ylim(-1.0, config.max_linear_velocity + 0.5)
			# ax_element2.set_ylim(-config.max_linear_acceleration * 2.5, config.max_linear_acceleration * 2.5)
			# ax_element3.set_ylim(-config.max_linear_acceleration * 45, config.max_linear_acceleration * 45)
			ax_element1.set_ylim(-1.0, 2.5)
			ax_element2.set_ylim(-2, 5)
			ax_element3.set_ylim(-20, 50)
			if self.result_graph_type == "gray":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color="black", linewidth=1,
												   linestyle="dashed")
				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color="black", linewidth=1,
													   linestyle="dotted")
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="black", linewidth=1)
			elif self.result_graph_type == "color":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity", color="black", linewidth=1)

				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Acceleraion", color="blue", linewidth=1)
				if element3 is not None:
					xl3 = np.arange(0, len(element3)) * 0.1
					curve_element3, = ax_element3.plot(xl3, element3, label="Jerk", color="red", linewidth=1)

		if mode == "car":
			ax_element1.set_ylabel("Velocity command [m/s]")
			ax_element1.set_xlabel("Time [s]")
			ax_element2.set_ylabel("Real velocity [m/s]")
			ax_element1.set_ylim(0.0, self.max_linear_velocity + 0.5)
			ax_element2.set_ylim(0.0, self.max_linear_velocity + 0.5)
			if self.result_graph_type == "gray":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity command", color="black", linewidth=1,
												   linestyle="solid")
				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Real velocity", color="black", linewidth=1,
													   linestyle="dashed")
			elif self.result_graph_type == "color":
				xl = np.arange(0, len(element1)) * 0.1
				curve_element1, = ax_element1.plot(xl, element1, label="Velocity command", color="black", linewidth=1)

				if element2 is not None:
					xl2 = np.arange(0, len(element2)) * 0.1
					curve_element2, = ax_element2.plot(xl2, element2, label="Real velocity", color="blue", linewidth=1)

		ax_element1.legend()
		ax_element1.legend(loc="upper right")
		if self.show_result_figure is True:
			plt.show()
		if self.save_result_figure is True:
			if mode == "vaj":
				plt.savefig(
					self.result_path + "vaj-" + str(self.type) + "-time{}.png".format(
						time.strftime("%Y%m%d-%H%M%S", time.localtime())),
					dpi=600, format="png")
			if mode == "waj":
				plt.savefig(
					self.result_path + "waj-" + str(self.type) + "-time{}.png".format(
						time.strftime("%Y%m%d-%H%M%S", time.localtime())),
					dpi=600, format="png")
			if mode == "right":
				plt.savefig(
					self.result_path + "right-" + str(self.type) + "-time{}.png".format(
						time.strftime("%Y%m%d-%H%M%S", time.localtime())),
					dpi=600, format="png")
			if mode == "left":
				plt.savefig(
					self.result_path + "left-" + str(self.type) + "-time{}.png".format(
						time.strftime("%Y%m%d-%H%M%S", time.localtime())),
					dpi=600, format="png")

	def result_path_graph(self, mode, odomx, odomy, element=None):
		plt.rcParams["axes.labelsize"] = 14
		plt.rcParams["xtick.labelsize"] = 14
		plt.rcParams["ytick.labelsize"] = 14
		if self.result_graph_type == "gray":
			clist = ["#C8C8C8", "black"]
		else:
			clist = [[0, 1, 0, 0.1], "red"]
		fig = plt.figure(figsize=(8, 3.0), tight_layout=True)
		img = plt.imread(self.map)
		ax = fig.add_subplot(111)
		divider = make_axes_locatable(ax)
		cax = divider.append_axes("right", size="5%", pad=0.5)
		newcm = LinearSegmentedColormap.from_list("chaos", clist)
		abs_element = np.abs(element)
		scatter_size = []
		for i in range(len(odomx)):
			if i % 50 == 0:
				scatter_size.append(30)
			else:
				scatter_size.append(5)

		mappable = ax.scatter(x=odomx, y=odomy, c=abs_element, label="Velocity", s=scatter_size, cmap=newcm, vmin=0,
							  vmax=1.0)
		ax.set_aspect("equal")
		ax.set_xlabel("x [m]")
		ax.set_ylabel("y [m]")
		ax.set_xlim(-22.025 * 1.0, 22.025 * 1.0)
		ax.set_ylim(-12.4 * 0.7, 12.4 * 0.7)
		ax.set(facecolor=[0.5, 0.5, 0.5])
		ax.imshow(img, extent=[-22.025, 22.025, -12.4, 12.4])
		pp = fig.colorbar(mappable, cax=cax, label="Jerk [" + "m/$ \mathrm{s}^{3}$" + "]")
		if self.show_result_figure is True:
			plt.show()
		if self.save_result_figure is True:
			plt.savefig(
				self.result_path + "path-" + str(self.type) + "-time{}.png".format(
					time.strftime("%Y%m%d-%H%M%S", time.localtime())),
				dpi=600, format="png")

	def result_save_process_time(self, obs):
		if self.save_process_time is True:
			kyo = time.strftime("%Y%m%d", time.localtime())
			f = open(self.result_path + "process time-" + str(self.type) + "-time" + str(kyo) + ".txt", "a")
			f.write("\n")
			f.write(str(self.dt) + "," + str(self.type) + "," + str(obs.laser_nums) + ",")
			f.write(str(len(self.cal_time1)) + ",")
			f.write(str(np.nanmean(self.cal_time1)) + ",")
			f.write(str(np.std(self.cal_time1)) + ",")
			f.write("\n")
			# profiler.print(file=f)
			f.close()
		print("frames =", len(self.cal_time1))
		print("average process time =", np.nanmean(self.cal_time1))


# Begin DWA calculations
def dwaControl(config, obs, speed):
	robot_state = [config.x, config.y, config.th, speed.linear.x, speed.angular.z]

	def calcDynamicWindow(robot_state, config):
		v1 = robot_state[3] - config.max_linear_acceleration * config.dt
		v2 = robot_state[3] + config.max_linear_acceleration * config.dt
		Vs = [config.min_linear_velocity, config.max_linear_velocity,
			  -config.max_angular_velocity, config.max_angular_velocity]
		Vd = [v1,
			  v2,
			  robot_state[4] - config.max_angular_acceleration * config.dt,
			  robot_state[4] + config.max_angular_acceleration * config.dt]
		#  [vmin, vmax, yawrate min, yawrate max]
		dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
		return dw

	def calcFinalInput(robot_state, config, obs):
		max_cost = -100000000
		state = 0

		def accelerationToVelocity(config, a, mode: str):
			time = 0
			predicted_velocity_list = []
			dt = config.dt
			if mode == "av":
				predicted_velocity = config.linear_velocity_list[-1]
				while time <= config.predict_time:
					predicted_velocity += a * dt
					predicted_velocity = max(0, predicted_velocity)
					predicted_velocity = min(config.max_linear_velocity, predicted_velocity)
					predicted_velocity_list.append(predicted_velocity)
					time += dt
			elif mode == "aw":
				predicted_velocity = config.angular_velocity_list[-1]
				while time <= config.predict_time:
					predicted_velocity += a * dt
					predicted_velocity = max(-config.max_angular_velocity, predicted_velocity)
					predicted_velocity = min(config.max_angular_velocity, predicted_velocity)
					predicted_velocity_list.append(predicted_velocity)
					time += dt
			return predicted_velocity_list

		def jerkToVelocity(config, j, mode: str):
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
														 predict_velocity_list[-1] + predict_acceleration_list[
															 i] * config.dt))
					else:
						predict_velocity_list.append(
							max(0, predict_velocity_list[-1] + predict_acceleration_list[i] * config.dt))
				del (predict_velocity_list[0])
			if mode == "jw":
				predict_acceleration_list.append(config.angular_acceleration_list[-1])
				predict_velocity_list.append(config.angular_velocity_list[-1])
				while time <= config.predict_time:
					if predict_acceleration_list[-1] + j * config.dt >= 0:
						predict_acceleration_list.append(
							min(config.max_angular_acceleration, predict_acceleration_list[-1] + j * config.dt))
					else:
						predict_acceleration_list.append(
							max(-config.max_angular_acceleration, predict_acceleration_list[-1] + j * config.dt))
					time = time + config.dt
				del (predict_acceleration_list[0])

				for i in range(len(predict_acceleration_list)):
					if predict_velocity_list[-1] + predict_acceleration_list[i] * config.dt >= 0:
						predict_velocity_list.append(min(config.max_angular_velocity,
														 predict_velocity_list[-1] + predict_acceleration_list[
															 i] * config.dt))
					else:
						predict_velocity_list.append(
							max(-config.max_angular_velocity,
								predict_velocity_list[-1] + predict_acceleration_list[i] * config.dt))
				del (predict_velocity_list[0])
			return predict_velocity_list, predict_acceleration_list

		def calcTrajectory(config, robot_state, v_predict_list, w_predict_list):
			x = np.array(robot_state)
			traj = np.array(robot_state)  # many motion models stored per trajectory

			config.predicted_x = config.x
			config.predicted_y = config.y
			config.predicted_th = config.th
			for i in range(len(v_predict_list)):
				config.sampled_v = v_predict_list[i]
				config.sampled_w = w_predict_list[i]
				x = config.motion()
				config.predicted_x = x[0]
				config.predicted_y = x[1]
				config.predicted_th = x[2]
				traj = np.vstack((traj, x))
			traj = np.delete(traj, 0, 0)
			return traj

		def calcObstacleCost(config, traj, obs):
			minr = 1000
			# Loop through every obstacle in set and calc Pythagorean distance
			# Use robot radius to determine if collision
			path = traj[:, 0:2]
			obst = list(obs.obst)
			if len(obst) == 0:
				minr = 1000
			else:
				obst = np.array(obst)
				config.ob_nums.append(obst.shape[0])
				dist_matrix = distance.cdist(obst, path, metric="euclidean")
				minr = np.min(dist_matrix)
			if minr <= config.robot_radius:
				cost = -10000  # collision
			elif config.robot_radius < minr <= 1.2:
				cost = minr - 0.2
			else:
				cost = 1
			return cost, minr

		def calcObstacleCost2(config, obs, av, v_predict_list, w):
			# minr = 1000
			dist_list = []
			vn = v_predict_list[-1]
			v0 = config.linear_velocity_list[-1]
			w0 = w
			theta0 = config.odom_th_list[-1]

			def ff_(t):
				if av > 0:
					acct = (config.max_linear_velocity - v0) / av
					cos_w0_t = math.cos(w0 * t)
					if 0 <= t <= acct:
						ff_ = (v0 + av * t) * cos_w0_t
					elif acct < t <= config.predict_time:
						ff_ = config.max_linear_velocity * cos_w0_t
					else:
						print("out of range")
				elif av < 0:
					acct = abs(v0 / av)
					cos_w0_t = math.cos(w0 * t)
					if 0 <= t <= acct:
						ff_ = (v0 + av * t) * cos_w0_t
					elif acct < t <= config.predict_time:
						ff_ = 0
					else:
						print("out of range")
				else:
					ff_ = v0 * math.cos(w0 * t)
				return ff_

			def gg_(t):
				if av > 0:
					acct = (config.max_linear_velocity - v0) / av
					sin_w0_t = math.sin(w0 * t)
					if 0 <= t <= acct:
						gg_ = (v0 + av * t) * sin_w0_t
					elif acct < t <= config.predict_time:
						gg_ = config.max_linear_velocity * sin_w0_t
					else:
						print("out of range")
				elif av < 0:
					acct = abs(v0 / av)
					sin_w0_t = math.sin(w0 * t)
					if 0 <= t <= acct:
						gg_ = (v0 + av * t) * sin_w0_t
					elif acct < t <= config.predict_time:
						gg_ = 0
					else:
						print("out of range")
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
						print("av > 0")
						print("out of range")
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
						print("t=", t)
						print("av < 0")
						print("out of range")
						print("==============")
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
						print("out of range")
				elif av < 0:
					acct = abs(v0 / av)
					y0 = (av * math.sin(w0 * 0) - w0 * (v0 + av * 0) * math.cos(w0 * 0)) / (w0 ** 2)
					dy1 = y0 - 0
					if 0 <= t <= acct:
						g = (av * math.sin(w0 * t) - w0 * (v0 + av * t) * math.cos(w0 * t)) / (w0 ** 2) - dy1
					elif acct < t <= config.predict_time:
						g = (av * math.sin(w0 * acct) - w0 * (v0 + av * acct) * math.cos(w0 * acct)) / (w0 ** 2) - dy1
					else:
						print("out of range")
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

				w0_t = w0 * config.predict_time
				cos_w0t = math.cos(w0_t)
				sin_w0t = math.sin(w0_t)

				dx21 = (v0 / w0) * sin_w0t - xend
				dy21 = (v0 / w0) * (1 - cos_w0t) - yend
				dx31 = (vn / w0) * sin_w0t - xend
				dy31 = (vn / w0) * (1 - cos_w0t) - yend

				w0_t_mid = w0 * midtime
				cos_w0t_mid = math.cos(w0_t_mid)
				sin_w0t_mid = math.sin(w0_t_mid)

				dx41 = (v0 / w0) * sin_w0t_mid - xmid
				dy41 = (v0 / w0) * (1 - cos_w0t_mid) - ymid
				dx51 = (vn / w0) * sin_w0t_mid - xmid
				dy51 = (vn / w0) * (1 - cos_w0t_mid) - ymid

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
				# anglemax = math.atan2(yend - b, xend)

				anglemax2 = theta0 + w0 * config.predict_time
				if anglemax2 > 0:
					anglemax = anglemax2 - 0.5 * math.pi
				elif anglemax2 < 0:
					anglemax = anglemax2 + 0.5 * math.pi

				obst_rc_list = list(obs.obst_rc)
				if len(obst_rc_list) == 0:
					dist = 1000
					obst_inrange = np.array([0])
					obst_outrange = np.array([0])
				else:
					obst_rc_array = np.array(obst_rc_list)
					x_diff = obst_rc_array[:, 0] - b
					y_diff = obst_rc_array[:, 1]
					angle_matrix = np.arctan2(y_diff, x_diff)

					obst_rc_array = np.column_stack((obst_rc_array, angle_matrix))

					if w0 > 0:
						inrange_condition = np.logical_and((-0.5 * math.pi < obst_rc_array[:, 2]),
														   (obst_rc_array[:, 2] < anglemax))
					else:
						inrange_condition = np.logical_and((anglemax < obst_rc_array[:, 2]),
														   (obst_rc_array[:, 2] < 0.5 * math.pi))

					obst_inrange = obst_rc_array[inrange_condition]
					obst_outrange = obst_rc_array[~inrange_condition]

					dist_to_outside_center = distance.cdist(outside_circle_center, obst_inrange[:, 0:2],
															metric="euclidean")
					dist_to_inside_center = distance.cdist(inside_circle_center, obst_inrange[:, 0:2],
														   metric="euclidean")

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
						dist_b1 = np.min(
							distance.cdist(np.array([[xend, yend]]), obst_outrange[:, 0:2], metric="euclidean"))
						dist_b2 = np.min(distance.cdist(np.array([[0, 0]]), obst_outrange[:, 0:2], metric="euclidean"))
						dist_b = min(dist_b1, dist_b2)
					else:
						dist_b = 1000

					dist = min(dist_a, dist_b)

			else:
				if av > 0:
					acct = (config.max_linear_velocity - v0) / av
					xend = (v0 * acct + 0.5 * av * acct ** 2) + config.max_linear_velocity * (
							config.predict_time - acct)
				elif av < 0:
					acct = abs(v0 / av)
					xend = min(v0 * acct + 0.5 * av * acct ** 2, v0 * config.predict_time)
				else:
					xend = v0 * config.predict_time
				yend = 0
				obst_rc_list = list(obs.obst_rc)
				# if len(obst_rc_list) == 0:
				if ~np.any(obst_rc_list):
					dist = 1000
					obst_inrange = np.array([0])
					obst_outrange = np.array([0])
				else:
					obst_rc_array = np.array(obst_rc_list)
					inrange_condition = np.logical_and((0 < obst_rc_array[:, 0]), (obst_rc_array[:, 0] < xend))
					obst_inrange = obst_rc_array[inrange_condition]
					obst_outrange = obst_rc_array[~inrange_condition]
					# if len(obst_inrange) > 0:
					if np.any(obst_inrange):
						dist_a = np.min(np.absolute(obst_inrange[:, 1]))
					else:
						dist_a = 1000
					# if len(obst_outrange) > 0:
					if np.any(obst_outrange):
						dist_b1 = np.min(distance.cdist(np.array([[0, 0]]), obst_outrange, metric="euclidean"))
						dist_b2 = np.min(distance.cdist(np.array([[xend, yend]]), obst_outrange, metric="euclidean"))
						dist_b = min(dist_b1, dist_b2)
					else:
						dist_b = 1000
					dist = min(dist_a, dist_b)

			if dist <= config.robot_radius:
				cost = -10000  # collision
			elif dist <= 1.2:
				cost = dist - 0.2
			else:
				cost = 1

			return cost, xend, yend, obst_inrange.shape[0], obst_outrange.shape[0]

		def calcToGoalCost(config, traj):
			# If-Statements to determine negative vs positive goal/trajectory position
			# traj[-1,0] is the last predicted X coord position on the trajectory
			remain_end = math.sqrt((config.localX - traj[-1, 0]) ** 2 + (config.localY - traj[-1, 1]) ** 2)
			# togoal1 = math.sqrt((config.localX - config.x) ** 2 + (config.localY - config.y) ** 2)
			to_end = math.sqrt((config.x - traj[-1, 0]) ** 2 + (config.y - traj[-1, 1]) ** 2)
			if config.type == "dwa" or "a":
				if config.map == "../world/1183.png":
					cost = 1 - (remain_end / 5)
				else:
					cost = 1 - (remain_end / 5)
			else:
				cost = 1 - ((remain_end ** 2) / 5)
				if config.localX == config.goalX:
					cost = (1 - (((remain_end + to_end) ** 2) / 5))
			return cost

		def calcToGoalCost2(config, xl, yl):
			ABT = np.array([[math.cos(config.th), -math.sin(config.th), 0, config.x],
							[math.sin(config.th), math.cos(config.th), 0, config.y],
							[0, 0, 1, 0],
							[0, 0, 0, 1]])
			BP = [xl, yl, 0, 1]

			CP = np.dot(ABT, BP)
			# total = math.sqrt((config.x-config.localX)**2 + (config.y-config.localY)**2)
			remain = math.sqrt((config.localX - CP[0]) ** 2 + (config.localY - CP[1]) ** 2)
			cost = 1 - (remain / 5)

			return cost

		def calcOdomAngleCost(config, traj):
			# function in dwa paper
			angle_end1 = math.atan2(config.localY - traj[-1, 1], config.localX - traj[-1, 0])  # -traj[-1,2]
			angle_end2 = traj[-1, 2]
			cost = 1 - abs((angle_end1 - angle_end2) / math.pi)
			return cost

		def calcToPathCost(config, traj):
			if len(config.path) != 0:
				a = 10 / len(config.path) ** 2
			else:
				a = 1
			path_matrix = np.reshape(np.array(config.path), (len(config.path), 2))
			end_point = np.array([[traj[-1, 0], traj[-1, 1]]])
			dist_matrix = distance.cdist(end_point, path_matrix, metric="euclidean")
			path_points_index = np.argmin(dist_matrix)
			if path_points_index > config.local_goal_index:
				config.local_goal_index = path_points_index
				cost = a * config.local_goal_index ** 2
			return cost

		if config.type == "dwa":
			dw = calcDynamicWindow(robot_state, config)
			for v in np.linspace(dw[0], dw[1], 5):
				for w in np.linspace(dw[2], dw[3], 5):
					v_predict_list = [v] * int(config.predict_time / config.dt)
					w_predict_list = [w] * int(config.predict_time / config.dt)
					traj = calcTrajectory(config, robot_state, v_predict_list, w_predict_list)
					# to_goal_cost,stop_mark,indexremain,remain_end = calc_to_goal_cost(traj, config)
					to_goal_cost = calcToGoalCost(config, traj)
					ob_cost, _ = calcObstacleCost(config, traj, obs)
					to_path_cost = 0  # calcToPathCost(config, traj)

					odom_angle_cost = calcOdomAngleCost(config, traj)
					add_cost = 0
					speed_cost = np.nanmean(v_predict_list) / config.max_linear_velocity

					final_cost = 1.0 * to_goal_cost + 2.0 * ob_cost + \
								 config.v_weight * speed_cost
					if final_cost > max_cost:
						max_cost = final_cost
						max_u = [v, w]
			if max_cost <= 0:
				state = -1
			return max_u, state

		elif config.type == "a":
			dw = calcDynamicWindow(robot_state, config)
			per_pair1 = []
			for av in np.linspace(-config.max_linear_acceleration, config.max_linear_acceleration, 5):
				for w in np.linspace(dw[2], dw[3], 5):
					v_predict_list = accelerationToVelocity(config, av, "av")
					w_predict_list = [w] * int(config.predict_time / config.dt)

					time_s = time.perf_counter()

					traj = calcTrajectory(config, robot_state, v_predict_list, w_predict_list)
					ob_cost1, _ = calcObstacleCost(config, traj, obs)
					to_goal_cost1 = calcToGoalCost(config, traj)
					final_cost = (5.0 * to_goal_cost1 + 1.0 * ob_cost1)

					time_e = time.perf_counter()
					per_pair1.append(time_e - time_s)

					if final_cost > max_cost:
						max_cost = final_cost
						max_u = [v_predict_list[int(0.1 / config.dt - 1)], w, av]
			config.cal_time1.append(sum(per_pair1))
			if max_cost <= 0:
				state = -1
			return max_u, state

		elif config.type == "a2":
			per_pair1 = []
			for av in np.linspace(-config.max_linear_acceleration, config.max_linear_acceleration, 5):
				for w in np.linspace(dw[2], dw[3], 5):
					v_predict_list = accelerationToVelocity(config, av, "av")

					time_s = time.perf_counter()

					ob_cost2, xl, yl, _, _ = calcObstacleCost2(config, obs, av, v_predict_list, w)
					to_goal_cost2 = calcToGoalCost2(config, xl, yl)
					final_cost = (3.0 * to_goal_cost2 + 1.0 * ob_cost2)

					time_e = time.perf_counter()
					per_pair1.append(time_e - time_s)

					if final_cost > max_cost:
						max_cost = final_cost
						max_u = [v_predict_list[int(0.1 / config.dt - 1)], w, av]

			config.cal_time1.append(sum(per_pair1))
			if max_cost <= 0:
				state = -1
			return max_u, state

		elif config.type == "j":
			dw = calcDynamicWindow(robot_state, config)
			for jv in np.arange(-config.max_linear_jerk, config.max_linear_jerk + 0.5 * config.linear_jerk_resolution,
								config.linear_jerk_resolution):
				for w in np.arange(dw[2], dw[3] + 0.5 * config.angular_velocity_resolution,
								   config.angular_velocity_resolution):
					al = []
					ssl = []
					v_predict_list, aa = jerkToVelocity(config, jv, "jv")
					w_predict_list = [w] * int(config.predict_time / config.dt)
					for i in range(len(v_predict_list) - 1):
						al.append((v_predict_list[i + 1] - v_predict_list[i]) / config.dt)
					for i in range(len(al) - 1):
						ssl.append(abs(al[i + 1] - al[i]) / config.dt)
					jv_cost = 1 - abs(round(max(ssl), 1) / config.max_linear_jerk)
					if jv_cost < 0:
						jv_cost = -100000000
					traj = calcTrajectory(config, robot_state, v_predict_list, w_predict_list)
					ob_cost1, _ = calcObstacleCost(config, traj, obst)
					to_goal_cost1 = calcToGoalCost(config, traj)
					to_path_cost = calcToPathCost(config, traj)
					speed_cost = np.nanmean(v_predict_list) / config.max_linear_velocity
					final_cost = (
							2.0 * to_path_cost + 1.0 * to_goal_cost1 + 2.0 * ob_cost1 + config.v_weight * speed_cost + config.j_weight * jv_cost)
					if final_cost > max_cost:
						max_cost = final_cost
						max_u = [v_predict_list[int(0.1 / config.dt - 1)], w]

			if max_cost <= 0:
				state = -1
			return max_u, state

		elif config.type == "jj":
			# max_cost = -100000000

			for jv in np.linspace(-config.max_linear_jerk, config.max_linear_jerk, 7):
				for jw in np.linspace(-config.max_angular_jerk, config.max_angular_jerk, 7):
					# print(jv,jw)
					if abs(jv) + abs(jw) * 0.22 > 0.5:
						continue
					val = []
					vssl = []
					wal = []
					wssl = []
					v_predict_list, aa = jerkToVelocity(config, jv, "jv")
					w_predict_list, ww = jerkToVelocity(config, jw, "jw")
					for i in range(len(v_predict_list) - 1):
						val.append((v_predict_list[i + 1] - v_predict_list[i]) / config.dt)
					for i in range(len(val) - 1):
						vssl.append(abs(val[i + 1] - val[i]) / config.dt)
					for i in range(len(w_predict_list) - 1):
						wal.append((w_predict_list[i + 1] - w_predict_list[i]) / config.dt)
					for i in range(len(wal) - 1):
						wssl.append(abs(wal[i + 1] - wal[i]) / config.dt)

					jv_cost = 1 - abs(round(max(vssl), 1) / config.max_linear_jerk)
					jw_cost = 1 - abs(round(max(wssl), 1) / config.max_angular_jerk)
					if jv_cost < 0:
						jv_cost = -100000000
					if jw_cost < 0:
						jw_cost = -100000000
					traj = calcTrajectory(config, robot_state, v_predict_list, w_predict_list)
					ob_cost1, _ = calcObstacleCost(config, traj, obst)
					to_goal_cost1 = calcToGoalCost(config, traj)
					angle_cost = calcOdomAngleCost(config, traj)
					# speed_cost = 1-(np.nanmean(np.abs(w_predict_list)) / config.max_angular_velocity)
					speed_cost = -(np.abs(w_predict_list[0]))
					final_cost = 1.0 * to_goal_cost1 + 2.0 * ob_cost1 + 0.0 * angle_cost + \
								 config.jv_weight * jv_cost + config.jw_weight * jw_cost
					if config.current_v <= 0.05:
						final_cost = 1.0 * to_goal_cost1 + 2.0 * ob_cost1 + 10.0 * speed_cost + \
									 config.jv_weight * jv_cost + config.jw_weight * jw_cost
					# print(final_cost, speed_cost, config.jv_weight * jv_cost, config.jw_weight * jw_cost)
					# print(final_cost, max_cost,w_predict_list[-1])
					if final_cost > max_cost:
						# print("****************")
						max_cost = final_cost
						max_u = [v_predict_list[int(0.1 / config.dt - 1)], w_predict_list[int(0.1 / config.dt - 1)]]
			# print(max_u)
			if max_cost <= 0:
				state = -1
			return max_u, state

	# Dynamic Window control
	u, state = calcFinalInput(robot_state, config, obs)
	return u, state


def main():
	print(__file__ + " start!!")
	config = Config()
	speed = Twist()
	after = AfterProcess()
	obs = Obstacles()
	config.aStar()

	parser = argparse.ArgumentParser()
	parser.add_argument("--jv", type=float, default=0.10)
	parser.add_argument("--jw", type=float, default=0.10)
	args = parser.parse_args()
	config.jv_weight = args.jv
	config.jw_weight = args.jw
	print(config.jv_weight, config.jw_weight)

	if config.map == "../world/1180.png":
		pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
		subOdom = rospy.Subscriber("robot_0/odom", Odometry, config.assignOdomCoords, queue_size=1, buff_size=2 ** 24)
		subLaser = rospy.Subscriber("robot_0/base_scan", LaserScan, obs.assignObs, config, queue_size=1,
									buff_size=2 ** 24)
		subCmd = rospy.Subscriber("robot_0/odom", Odometry, config.callcmd, queue_size=1, buff_size=2 ** 24)
	else:
		subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
		subLaser = rospy.Subscriber("/base_scan", LaserScan, obs.assignObs, config)
		subCmd = rospy.Subscriber("/odom", Odometry, config.callcmd)
		pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)

	t = 0
	if not os.path.exists(config.result_path):
		os.makedirs(config.result_path)
	profiler = Profiler()
	profiler.start()
	time.sleep(1)
	ap = []
	bp = []
	cp = []
	while not rospy.is_shutdown():
		t = t + 1
		if (config.atGoal() is False) and t <= 800:
			u, state = dwaControl(config, obs, speed)
			speed.linear.x = u[0]
			speed.angular.z = u[1]
		elif config.atGoal() is True:
			print("ending linear velocity =", config.linear_velocity_list[-1])
			print("ending angular velocity =", config.angular_velocity_list[-1])
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			pub.publish(speed)
			print(f"{Fore.GREEN}Reached goal{Style.RESET_ALL}")
			break
		else:
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			pub.publish(speed)
			print(f"{Fore.RED}Overtime{Style.RESET_ALL}")
			break
		pub.publish(speed)

		config.linear_velocity_list.append(speed.linear.x)  # velocity
		config.support_linear_velocity_list.append(config.current_v)
		# config.linear_velocity_list.append(config.current_v)
		config.linear_acceleration_list.append(
			(config.linear_velocity_list[-1] - config.linear_velocity_list[-2]) / 0.1)  # acceleration
		config.linear_jerk_list.append(
			(config.linear_acceleration_list[-1] - config.linear_acceleration_list[-2]) / 0.1)  # acceleration

		config.angular_velocity_list.append(speed.angular.z)
		config.angular_acceleration_list.append(
			(config.angular_velocity_list[-1] - config.angular_velocity_list[-2]) / 0.1)
		config.angular_jerk_list.append(
			(config.angular_acceleration_list[-1] - config.angular_acceleration_list[-2]) / 0.1)  # acceleration

		config.right_velocity_list.append(config.linear_velocity_list[-1] + 0.22 * config.angular_velocity_list[-1])
		config.left_velocity_list.append(config.linear_velocity_list[-1] - 0.22 * config.angular_velocity_list[-1])

		config.right_acceleration_list.append((config.right_velocity_list[-1] - config.right_velocity_list[-2]) / 0.1)
		config.left_acceleration_list.append((config.left_velocity_list[-1] - config.left_velocity_list[-2]) / 0.1)

		config.right_jerk_list.append((config.right_acceleration_list[-1] - config.right_acceleration_list[-2]) / 0.1)
		config.left_jerk_list.append((config.left_acceleration_list[-1] - config.left_acceleration_list[-2]) / 0.1)

		config.odom_x_list.append(config.x)
		config.odom_y_list.append(config.y)
		config.odom_th_list.append(config.th)
		if config.x <= -15:
			ap.append(config.x)
		elif -15 < config.x <= 15:
			bp.append(config.x)
		else:
			cp.append(config.x)

		config.r.sleep()
	profiler.stop()
	print("obst_mean =", np.nanmean(config.ob_nums))

	after.result_save_process_time(obs)
	after.result_graph("vaj", config.linear_velocity_list, config.angular_velocity_list)
	after.result_graph("right", config.right_velocity_list, config.right_acceleration_list, config.right_jerk_list)
	after.result_graph("left", config.left_velocity_list, config.left_acceleration_list, config.left_jerk_list)
	after.result_path_graph("mode", config.odom_x_list, config.odom_y_list, config.linear_jerk_list)
	after.result_graph("car", config.linear_velocity_list, config.support_linear_velocity_list)

	if after.show_pyinstrument is True:
		profiler.print()

	print("max rv =", max(config.right_velocity_list))
	print("max lv =", max(config.left_velocity_list))
	print("max ra =", max(config.right_acceleration_list))
	print("max la =", max(config.left_acceleration_list))
	print("max rj =", max(config.right_jerk_list))
	print("max lj =", max(config.left_jerk_list))

	f = open(config.result_path + "0609" + config.type + ".txt", "a")
	f.write(str(args.jv) + "," + str(args.jw) + ",")
	f.write(str(len(ap)) + ",")
	f.write(str(len(bp)) + ",")
	f.write(str(len(cp)) + ",")
	f.write(str(len(ap) + len(bp) + len(cp)))
	f.write("\n")
	f.close()

	print("=================")


if __name__ == "__main__":
	rospy.init_node("dwa")
	main()
