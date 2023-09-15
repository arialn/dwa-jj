
import cv2
import rospy
import math
import numpy as np
import os
import time
import heapq

from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap, LinearSegmentedColormap
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from mpl_toolkits.axes_grid1 import make_axes_locatable
from pyinstrument import Profiler
from colorama import Fore, Style

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
		self.map = os.path.dirname(os.path.dirname(__file__))+"/world/" + self.map_name + ".png"
		self.result_path = os.path.dirname(os.path.dirname(__file__))+"/results/{}/{}/{}/".format(time.strftime("%Y%m%d", time.localtime()), self.type,
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
		solve = Solver()
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


class Solver:
	def __init__(self):
		pass

	def hue(self, a, b):
		return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

	def astar(self, start, end, grid):
		neighbours = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
		heap = []
		closed_list = set()
		gscore = {start: 0}
		fscore = {start: self.hue(start, end)}
		parent = {}
		heapq.heappush(heap, (fscore[start], start))
		while heap:
			current = heapq.heappop(heap)[1]
			closed_list.add(current)
			if (current == end):
				data = []
				while current in parent:
					data.append(current)
					current = parent[current]
				return data

			for i, j in neighbours:
				neighbour = current[0] + i, current[1] + j
				tgscore = gscore[current] + self.hue(current, neighbour)
				if 0 <= neighbour[0] < grid.shape[0]:
					if 0 <= neighbour[1] < grid.shape[1]:
						if grid[neighbour[0]][neighbour[1]]:
							continue
					else:
						continue
				else:
					continue
				if neighbour in closed_list and tgscore >= gscore.get(neighbour, 0):
					continue
				if tgscore < gscore.get(neighbour, 0) or neighbour not in [i[1] for i in heap]:
					parent[neighbour] = current
					gscore[neighbour] = tgscore
					fscore[neighbour] = tgscore + self.hue(neighbour, end)
					heapq.heappush(heap, (fscore[neighbour], neighbour))

		return False
