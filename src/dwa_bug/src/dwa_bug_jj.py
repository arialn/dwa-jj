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
import pandas as pd
import datetime
import time
import matplotlib as mpl
import mpl_toolkits.mplot3d

from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap,LinearSegmentedColormap
from matplotlib.ticker import MultipleLocator

from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from mpl_toolkits.axes_grid1 import make_axes_locatable
from mpl_toolkits.axes_grid1.inset_locator import mark_inset,inset_axes
from pynverse import inversefunc
from pyinstrument import Profiler
import PIL.Image as Image



class Config():
	# simulation parameters
	def __init__(self):
		# robot parameter
		self.max_speed = 1.5  # [m/s]
		self.min_speed = 0.0  # [m/s]
		self.max_yawrate = 6.28  # [rad/s]360 degree

		self.max_accel = 2.0  # [m/ss]
		self.max_dyawrate = 5.24  # [rad/ss]300 degree

		self.max_vjerk = 0.5  # [m/sss]
		self.vj_reso = self.max_vjerk/2

		self.max_wjerk = 1.31  # [m/sss]
		self.wj_reso = self.max_wjerk/2

		self.v_reso = 0.1  # [m/s] 1d method
		#self.v_reso = 0.1    # [m/s] 2d method

		self.yawrate_reso = 0.2 # [rad/s]

		self.dt = 0.1 # [s]
		self.predict_time = 2.0

		self.robot_radius = 0.2 # [m]

		self.goalX = 20.0
		self.goalY = 0.0
		self.startX = -20.0
		self.startY = 0.0
		self.x = self.startX
		self.y =self.startY
		self.th = 0.0
		self.localX = self.goalX
		self.localY = self.goalY
		self.acc_time = self.max_speed/self.max_accel

		self.current_v = 0.0
		self.current_w = 0.0
		self.r = rospy.Rate(10)
		self.discthresh = 0.1

		self.v_all_list = [0]
		self.w_all_list = [0]
		self.av_all_list = [0]
		self.aw_all_list = [0]

		self.mode = "equal"
		#self.mode = "quad"
		#self.mode = "sin2"
		self.dwa_mode = "2d"
		#self.dwa_mode = "1d"
		self.state = "goto"
		#self.local_minimum = list()
		self.laser_reso = []
		#self.local_minimum = set()
		self.map = '/home/z_lin/mapless/src/dwa_bug/world/simple.png'

	def assignOdomCoords(self, msg):
		# X- and Y- coords and pose of robot fed back into the robot config
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		r = R.from_quat([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
		(roll,pitch,theta) = r.as_euler('xyz',degrees=False)
		self.th = theta

	# Callback for attaining goal co-ordinates from Rviz Publish Point
	def goalCB(self,msg):
		self.goalX = 20.0#msg.linear.x
		self.goalY = 0.0#msg.linear.y

	# Callback for cmd_vel
	def callcmd(self,msg):
		self.current_v = msg.linear.x
		self.current_w = msg.angular.z

class Obstacles():
	def __init__(self):
		# Set of coordinates of obstacles in view
		self.obst = set()
		self.sudden = [(20,3)]#list()
		self.bestDist = 10000.0
		#self.bestDistinturn = 10000.0
		self.bestsudden = 10000.0
		self.state='goto'

	# Custom range implementation to loop over LaserScan degrees with
	# a step and include the final degree
	def myRange(self,start,end,step):
		i = start
		while i < end:
			yield i
			i += step
		yield end

	# Callback for LaserScan
	def assignObs(self, msg, config):
		deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
		self.obst = set()   # reset the obstacle set to only keep visible objects
		#self.sudden = list()
		#bestDist = 10000.0
		#print(deg)
		angle2goal = math.atan2(config.goalY-config.y,config.goalX-config.x)
		sensorIndex = int((angle2goal-msg.angle_min)/msg.angle_increment)
		config.laser_reso = deg
		for angle in self.myRange(0,deg-1,1):
			distance = msg.ranges[angle]
			# only record obstacles that are within 4 metres away
			if (distance < 5.0):
				scanTheta = -math.pi + angle * msg.angle_increment
				xb = distance * math.cos(scanTheta)
				yb = distance * math.sin(scanTheta)
				BP = np.array([xb,yb,0,1])

				ABT = np.array([[math.cos(config.th),-math.sin(config.th),0,config.x],
								[math.sin(config.th),math.cos(config.th),0,config.y],
								[0,0,1,0],
								[0,0,0,1]])

				AP = np.dot(ABT, BP)
				self.obst.add((AP[0],AP[1],distance))

	def assignObs2(self, msg, config):
		deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
		self.obst = set()   # reset the obstacle set to only keep visible objects
		self.sudden = list()
		self.suddeninrobot = list()
		self.bestDistinturn = 10000.0
		self.xxx=100
		localtestx = 0
		ABT = np.array([[math.cos(config.th),-math.sin(config.th),0,config.x],
						[math.sin(config.th),math.cos(config.th),0,config.y],
						[0,0,1,0],
						[0,0,0,1]])
		CP = np.array([config.goalX,config.goalY,0,1])
		ANTI_ABT = np.linalg.inv(ABT)
		DP = np.dot(ANTI_ABT,CP)
		angleinrobot = math.atan2(DP[1],DP[0])
		sensorIndex = int((angleinrobot-msg.angle_min)/msg.angle_increment)
		besti = -1
		bestj = -1
		#bestsudden = 0
		config.laser_reso = deg

		for angle in self.myRange(0,deg-1,1):
			distance = msg.ranges[angle]
			# only record obstacles that are within 4 metres away
			if (distance < 5.0):
				scanTheta = -math.pi + angle * msg.angle_increment
				xb = distance * math.cos(scanTheta)
				yb = distance * math.sin(scanTheta)
				BP = np.array([xb,yb,0,1])
				AP = np.dot(ABT, BP)

				self.obst.add((AP[0],AP[1]))
				if msg.ranges[angle-1] >= 5.0:
					self.sudden.append([AP[0],AP[1],scanTheta])
					self.suddeninrobot.append(np.dot(ANTI_ABT,[AP[0],AP[1],0,1]))
				#if (abs(msg.ranges[angle]-msg.ranges[angle-1]) > config.discthresh):
				#	self.sudden.append([AP[0],AP[1],scanTheta])
				#	self.suddeninrobot.append(np.dot(ANTI_ABT,[AP[0],AP[1],0,1]))

		if len(self.sudden) > 0:
			for i in range(len(self.sudden)):
				suddendist = math.sqrt((self.sudden[i][0]-config.goalX) ** 2 + (self.sudden[i][1]-config.goalY) ** 2)
				distance = math.sqrt((self.sudden[i][0]-config.x) ** 2 + (self.sudden[i][1]-config.y) ** 2)
				if suddendist < self.bestsudden:
					self.bestsudden = suddendist
				if (suddendist+distance) < self.bestDistinturn:
					self.bestDistinturn = suddendist + distance
					besti = i
			if self.bestDistinturn <= self.bestDist:
				self.bestDist = self.bestDistinturn
				self.state = 'goto'
			else:
				if math.sqrt((config.x-config.goalX)**2+(config.y-config.goalY)**2) <= self.bestsudden:
					self.state = 'goto'
				else:
					self.state = 'follow'

		if self.state == 'goto':
			if msg.ranges[sensorIndex] >= 5.0:
				config.localX = config.goalX
				config.localY = config.goalY
			else:
				if besti >= 0:
					config.localX = self.sudden[besti][0]
					config.localY = self.sudden[besti][1]

		if self.state == 'follow':
			for j in range(len(self.sudden)):
				if math.sqrt((self.sudden[j][0]-config.localX)**2+(self.sudden[j][1]-config.localY)**2)<self.xxx:
					self.xxx = math.sqrt((self.sudden[j][0]-config.localX)**2+(self.sudden[j][1]-config.localY)**2)
					bestj = j
			if bestj >= 0:
				config.localX = self.sudden[bestj][0]
				config.localY = self.sudden[bestj][1]
# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
	# motion model
	# x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
	x = [x[0],x[1],x[2],u[0],u[1]]
	DT = np.array([[1,0,0,dt*math.cos(x[2]),0],
					[0,1,0,dt*math.sin(x[2]),0],
					[0,0,1,0,dt],
					[0,0,0,1,0],
					[0,0,0,0,1]])
	x = np.dot(DT,x)
	return x
def linear_velocity_generator(jv,config):
	time = 0.1
	av_predict_list = []
	v_predict_list = []

	av_predict_list.append(config.av_all_list[-1])
	v_predict_list.append(config.v_all_list[-1])
	while time <= config.predict_time:
		if av_predict_list[-1]+jv*config.dt >= 0:
			av_predict_list.append(min(config.max_accel,av_predict_list[-1]+jv*config.dt))
		else:
			av_predict_list.append(max(-config.max_accel,av_predict_list[-1]+jv*config.dt))
		if v_predict_list[-1]+av_predict_list[-1]*config.dt >= 0:
			v_predict_list.append(min(config.max_speed,v_predict_list[-1]+av_predict_list[-1]*config.dt))
		else:
			v_predict_list.append(max(0,v_predict_list[-1]+av_predict_list[-1]*config.dt))
		time = time + config.dt
		#print(time)
	#print(v_predict_list)
	for i in range(len(v_predict_list)):
		if v_predict_list[i] == config.max_speed:
			av_predict_list[i] = 0

	return v_predict_list,av_predict_list[0]

def angular_velocity_generator(jw,config):
	time = 0.1
	aw_predict_list = []
	w_predict_list = []

	aw_predict_list.append(config.aw_all_list[-1])
	w_predict_list.append(config.w_all_list[-1])
	while time <= config.predict_time:
		if aw_predict_list[-1]+jw*config.dt >= 0:
			aw_predict_list.append(min(config.max_dyawrate,aw_predict_list[-1]+jw*config.dt))
		else:
			aw_predict_list.append(max(-config.max_dyawrate,aw_predict_list[-1]+jw*config.dt))
		if w_predict_list[-1]+aw_predict_list[-1]*config.dt >= 0:
			w_predict_list.append(min(config.max_yawrate,w_predict_list[-1]+aw_predict_list[-1]*config.dt))
		else:
			w_predict_list.append(max(-config.max_yawrate,w_predict_list[-1]+aw_predict_list[-1]*config.dt))
		time = time + config.dt
		#print(time)
	#print(v_predict_list)
	for i in range(len(w_predict_list)):
		if w_predict_list[i] == config.max_yawrate:
			aw_predict_list[i] = 0

	return w_predict_list,aw_predict_list[0]

# Determine the dynamic window from robot configurations
def calc_dynamic_window(x, config):
	def v_to_t(v):
		if v <= config.min_speed:
			t = 0
		elif v >= config.max_speed:
			t = config.acc_time
		else:
			t = v / config.max_accel
		return t
	def t_to_v(t):
		if t <= 0:
			v = config.min_speed
		elif t >= config.acc_time:
			v = config.max_speed
		else:
			v = config.max_accel * t
		return v
	t = v_to_t(x[3])
	t1 = t - config.dt
	t2 = t + config.dt

	s = 0.5 * config.max_accel * config.acc_time**2
	if math.sqrt((config.goalX-x[0])**2 + (config.goalY-x[1])**2) <= s:
		v2 = t_to_v(t1)
		v1 = t_to_v(t1)
	else:
		v2 = t_to_v(t2)
		v1 = t_to_v(t2)
	if config.dwa_mode == '2d':
		v2 = t_to_v(t2)
		v1 = t_to_v(t1)
	# Dynamic window from robot specification
	Vs = [config.min_speed, config.max_speed,
		  -config.max_yawrate, config.max_yawrate]

	# Dynamic window from motion model
	Vd = [v1,
		  v2,
		  x[4] - config.max_dyawrate * config.dt,
		  x[4] + config.max_dyawrate * config.dt]

	#  [vmin, vmax, yawrate min, yawrate max]
	dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
	#print(dw)
	return dw

def quadratic_window(x, config):
	def v_to_t(v):
		if v <= config.max_speed/2:
			t = math.sqrt(2*v*config.acc_time/config.max_accel)
		elif config.max_speed/2 < v <= config.max_speed:
			t = 2*config.acc_time - math.sqrt((config.max_speed-v)*(2*config.acc_time/config.max_accel))
		else:
			t = 2*config.acc_time
		return t
	def t_to_v(t):
		if 0 < t <= config.acc_time:
			v = config.max_accel*t**2/(2*config.acc_time)
		elif config.acc_time < t < 2*config.acc_time:
			#v = config.max_speed-((0.5*config.max_accel/config.acc_time)*((t-2*config.acc_time)**2))
			v = config.max_speed-config.max_accel*(t-2*config.acc_time)**2/(2*config.acc_time)
		elif t <= 0:
			v = 0
		elif t >= 2*config.acc_time:
			v = config.max_speed
		return v
	t = v_to_t(x[3])
	t1 = t - config.dt
	t2 = t + config.dt
	s =  config.max_accel * config.acc_time**2
	if math.sqrt((config.goalX-x[0])**2 + (config.goalY-x[1])**2) <= s:
		v2 = t_to_v(t1)
		v1 = t_to_v(t1)
	else:
		v2 = t_to_v(t2)
		v1 = t_to_v(t2)
	if config.dwa_mode == '2d':
		v2 = t_to_v(t2)
		v1 = t_to_v(t1)

	# Dynamic window from robot specification
	Vs = [config.min_speed, config.max_speed,
		  -config.max_yawrate, config.max_yawrate]

	# Dynamic window from motion model
	Vd = [v1,
		  v2,
		  x[4] - config.max_dyawrate * config.dt,
		  x[4] + config.max_dyawrate * config.dt]

	#  [vmin, vmax, yawrate min, yawrate max]
	dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
		  max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
	#print(dw)

	return dw

def sin_window(x,config):
	def v_to_t(v):
		if v <= config.min_speed:
			t = -0.25 * math.pi * config.acc_time
		elif v >= config.max_speed:
			t = 0.25 * math.pi * config.acc_time
		else:
			t = 0.5 * config.acc_time * math.asin((2*v/config.max_speed)-1)
		return t

	def t_to_v(t):
		if t < - 0.25 * math.pi * config.acc_time:
			v = config.min_speed
		elif t > 0.25 * math.pi * config.acc_time:
			v = config.max_speed
		else:
			v = 0.5 * config.max_speed * (1 + math.sin(2*t/config.acc_time))
		return v
	t = v_to_t(x[3])
	t1 = t - config.dt
	t2 = t + config.dt
	s = math.pi * config.max_speed * config.acc_time*0.25

	if math.sqrt((config.goalX-config.x)**2 + (config.goalY-config.y)**2) <= s:
		v2 = t_to_v(t1)
		v1 = t_to_v(t1)
	else:
		v2 = t_to_v(t2)
		v1 = t_to_v(t2)
	# Dynamic window from robot specification
	Vs = [config.min_speed, config.max_speed,
		  -config.max_yawrate, config.max_yawrate]

	# Dynamic window from motion model
	Vd = [v1,
		  v2,
		  x[4] - config.max_dyawrate * config.dt,
		  x[4] + config.max_dyawrate * config.dt]

	#  [vmin, vmax, yawrate min, yawrate max]
	dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
		  max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
	#print(dw)
	return dw

def sin2_window(x,config):
	cube = (lambda t: config.max_speed/2+config.max_speed/(2*math.pi)*(math.sin(math.pi*t/config.acc_time)+(math.pi*t/config.acc_time)))
	def v_to_t(v):
		if v <= 0:
			t = -config.acc_time
		elif v >= config.max_speed:
			t = config.acc_time
		else:
			t = inversefunc(cube,y_values = v)
		return t

	def t_to_v(t):
		if t <= -config.acc_time:
			v = 0
		elif t >= config.acc_time:
			v = config.max_speed
		else:
			v = 0.5 * config.max_speed+config.max_speed/(2*math.pi)*(math.sin(math.pi*t/config.acc_time)+(math.pi*t/config.acc_time))
		return v
	t = v_to_t(x[3])
	t1 = t - config.dt
	t2 = t + config.dt
	s =  config.max_speed * config.acc_time

	#print(s)

	if math.sqrt((20-config.x)**2 + (3-config.y)**2) <= s:
		v2 = t_to_v(t1)
		v1 = t_to_v(t1)
	else:
		v2 = t_to_v(t2)
		v1 = t_to_v(t2)

	if config.dwa_mode == '2d':
		v2 = t_to_v(t2)
		v1 = t_to_v(t1)

	# Dynamic window from robot specification
	Vs = [config.min_speed, config.max_speed,
		  -config.max_yawrate, config.max_yawrate]

	# Dynamic window from motion model
	Vd = [v1,
		  v2,
		  x[4] - config.max_dyawrate * config.dt,
		  x[4] + config.max_dyawrate * config.dt]

	#  [vmin, vmax, yawrate min, yawrate max]
	dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
		  max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
	print(dw)
	return dw

def sigmoid_window(x,config):
	def v_to_t(v):
		if v <= config.min_speed:
			t = -1.25 * config.acc_time
		elif v >= config.max_speed:
			t = 1.25 * config.acc_time
		else:
			t = -config.acc_time * math.log(config.max_speed / v - 1) * 0.25
		return t

	def t_to_v(t):
		if t < -1.25 * config.acc_time:
			v = config.min_speed
		elif t > 1.25 * config.acc_time:
			v = config.max_speed
		else:
			v = config.max_speed * (1 / (1 + (math.exp(-4 * t / config.acc_time))))
		return v
	t = v_to_t(x[3])
	t1 = t - config.dt
	t2 = t + config.dt

	s = config.max_speed * config.acc_time*1.25

	if math.sqrt((config.goalX-config.x)**2 + (config.goalY-config.y)**2) <= 6.94:
		v2 = t_to_v(t1)
		v1 = t_to_v(t1)
	else:
		v2 = t_to_v(t2)
		v1 = t_to_v(t2)
	# Dynamic window from robot specification
	Vs = [config.min_speed, config.max_speed,
		  -config.max_yawrate, config.max_yawrate]

	# Dynamic window from motion model
	Vd = [v1,
		  v2,
		  x[4] - config.max_dyawrate * config.dt,
		  x[4] + config.max_dyawrate * config.dt]

	#  [vmin, vmax, yawrate min, yawrate max]
	dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
		  max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

	#print(dw)

	return dw
# Calculate a trajectory sampled across a prediction time
def calc_trajectory(xinit, v_predict_list, w_predict_list, config):
	x = np.array(xinit)
	#print(x)
	traj = np.array(x)  # many motion models stored per trajectory
	time = 0
	#w_predict_list = [w]*8

	#while time <= config.predict_time:
	if type(w_predict_list) == list:
		#for v in v_predict_list:
		for i in range(len(v_predict_list)):
			#for w in w_predict_list:
			# store each motion model along a trajectory
			#x = motion(x, [v, w_predict_list[v_predict_list.index(v)]], config.dt)
			x = motion(x, [v_predict_list[i], w_predict_list[i]], config.dt)
			traj = np.vstack((traj, x))
			time = time + config.dt # next sample
	else:
		#for v in v_predict_list:
		for i in range(len(v_predict_list)):
			#for w in w_predict_list:
			# store each motion model along a trajectory
			x = motion(x, [v_predict_list[i], w_predict_list], config.dt)
			traj = np.vstack((traj, x))
			time = time + config.dt # next sample

	return traj

# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, obst):

	xinit = x[:]
	max_cost = 0
	state = 0
	max_u = u
	#deaccel_dist = 0.5 * config.max_accel*(config.v_all_list[-1]/config.max_accel)**2
	#deaccel_dist = 0.5 * config.max_accel*(config.max_speed/config.max_accel)**2*2.0
	deaccel_dist=2.5
	#print(deaccel_dist)
	to_goal_dist = math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2)
	#print(to_goal_dist)
	# evaluate all trajectory with sampled input in dynamic window
	for jv in np.arange(-config.max_vjerk, config.max_vjerk+0.5*config.vj_reso,config.vj_reso):
	#for v in np.arange(dw[0], dw[1]+0.5*config.v_reso,config.v_reso):
		#for jw in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
		for jw in np.arange(-config.max_wjerk, config.max_wjerk+0.5*config.wj_reso,config.wj_reso):
			v_predict_list,av = linear_velocity_generator(jv,config)
			w_predict_list,aw = angular_velocity_generator(jw,config)

			traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)# generate trajectory
			#print(jw,w_predict_list,traj)
			to_goal_cost = calc_to_goal_cost(traj, config)# score from end of trajectory to goal

			speed_cost = v_predict_list[-1]/config.max_speed
			#speed_cost = (v_predict_list[-1]-config.v_all_list[-1])/config.max_speed

			#jerk_cost = 1-(abs(j/config.max_jerk))

			ob_cost = calc_obstacle_cost(traj, obst, config)#score from end of trajectory to obstacle

			odom_angle_cost = calc_odom_angle_cost(traj,config)#准确度？

			#final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.001*jerk_cost)
			"""
			if to_goal_dist >= deaccel_dist:
				#jerk_cost = 1-(abs(j/config.max_jerk))
				speed_cost = traj[-1, 3]/config.max_speed
				if v_predict_list[1] <= 1.0:
					jerk_cost = 1-(abs(j/config.max_jerk))
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.001*jerk_cost)
				else:
					if j > 0:
						jerk_cost = -1
					else:
						jerk_cost = 1-(abs(j/config.max_jerk))
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.001*jerk_cost)
			else:
				jerk_cost = 1-(abs(j/config.max_jerk))
				speed_cost = 1-(traj[-1, 3]/config.max_speed)
				if v_predict_list[1] >= 1.0:
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.001*jerk_cost)
				else:
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.001*jerk_cost)
			"""

			#wjerk_cost = 1-abs(jw/config.max_wjerk)
			wjerk_cost = 1-abs((jw/config.max_wjerk)**1)
			#w_cost = 1-abs(w_predict_list[-1]/config.max_yawrate)
			#vjerk_cost = 1-(abs(jv/config.max_vjerk))

			#final_cost = (0.0*odom_angle_cost+2.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*vjerk_cost+0.01*wjerk_cost)

			if v_predict_list[1] < 1.0:
				#if j >= 0:
				#	jerk_cost = 1-abs(j/config.max_jerk)
				#else:
				#	jerk_cost = -1
				vjerk_cost = (jv/config.max_vjerk)

				if to_goal_dist >= deaccel_dist:
					#speed_cost = (v_predict_list[-1]-config.v_all_list[-1])/config.max_speed
					speed_cost = v_predict_list[-1]/config.max_speed
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*vjerk_cost+0.01*wjerk_cost)
				#print(j,w,traj[-1, 3],speed_cost,final_cost)
				else:
					speed_cost = 1-abs(v_predict_list[-1]/config.max_speed)
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +10.0*speed_cost+0.01*vjerk_cost+0.01*wjerk_cost)
			else:
				#if j >= 0:
				#	jerk_cost = -1
				#else:
				#	jerk_cost = 1-abs(j/config.max_jerk)
				vjerk_cost = -(jv/config.max_vjerk)
				if to_goal_dist >= deaccel_dist:
					#speed_cost = (v_predict_list[-1]-config.v_all_list[-1])/config.max_speed
					speed_cost = v_predict_list[-1]/config.max_speed
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*vjerk_cost+0.01*wjerk_cost)
				#print(j,w,traj[-1, 3],speed_cost,final_cost)
				else:
					speed_cost = 1-abs(v_predict_list[-1]/config.max_speed)
					final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +10.0*speed_cost+0.01*vjerk_cost+0.01*wjerk_cost)


				#print(traj[-1, 3],speed_cost,final_cost)
			#print(v_predict_list[0],w,speed_cost,final_cost)

			if final_cost >= max_cost:
				max_cost = final_cost
				max_u = [v_predict_list[1], w_predict_list[1],jv,jw]
				#print('***',max_u,to_goal_cost,speed_cost,vjerk_cost,wjerk_cost)
	#print('*****',max_u)
	if final_cost <= 0:
		state = -1
	return max_u,state
# Calculate obstacle cost inf: collision, 0:free

def calc_obstacle_cost(traj, obst, config):
	minr = 1000
	# Loop through every obstacle in set and calc Pythagorean distance
	# Use robot radius to determine if collision
	for ii in range(0, traj.shape[0]):
		for i in obst.copy():
			dx = traj[ii, 0] - i[0]
			dy = traj[ii, 1] - i[1]
			r = math.sqrt(dx**2 + dy**2)
			if r <= minr:
				minr = r
	if minr <= config.robot_radius:
		cost = -10000  # collision
	elif config.robot_radius< minr <= 1.2:
		cost = minr - 0.2
	else:
		cost = 1
	return cost

# Calculate goal cost via Pythagorean distance to robot
def calc_to_goal_cost(traj, config):
	# If-Statements to determine negative vs positive goal/trajectory position
	# traj[-1,0] is the last predicted X coord position on the trajectory
	total = math.sqrt((config.x-config.localX)**2 + (config.y-config.localY)**2)
	remain = math.sqrt((config.localX-traj[-1,0])**2 + (config.localY-traj[-1,1])**2)
	cost = 1 -(remain / total)
	return cost

def calc_odom_angle_cost(traj, config):
	# If-Statements to determine negative vs positive goal/trajectory position
	# traj[-1,0] is the last predicted X coord position on the trajectory
	angle_end1 = math.atan2(config.localY-traj[-1,1],config.localX-traj[-1,0])#-traj[-1,2]
	angle_end2 = traj[-1,2]
	cost = 1-abs((angle_end1-angle_end2)/math.pi)
	return cost

# Begin DWA calculations
def dwa_control(x, u, config,obst):

	# Dynamic Window control
	if config.mode == 'sin2':
		dw = sin2_window(x, config)
	if config.mode == 'equal':
		dw = calc_dynamic_window(x, config)
	if config.mode == 'quad':
		dw = quadratic_window(x, config)

	u,state = calc_final_input(x, u, dw, config, obst)
	return u,state

# Determine whether the robot has reached its goal
def atGoal(config, x):
	# check at goal
	if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) <= 0.2:
		return True
	return False

def threedgraph(odomx,odomy,vl,al,jl):
	#vl.append(0)
	#print(len(odomx),len(vl))
	#ax = plt.subplot(projection = "3d")
	ax = plt.axes(projection = "3d")
	tl = range(len(vl))
	max_range = np.array([np.ptp(odomx),np.ptp(odomy)]).max()/2.0
	midx = (max(tl)+min(tl))/2.0
	midy = (max(odomx)+min(odomx))/2.0
	midz = (max(odomy)+min(odomy))/2.0

	maxlist = [midy+max_range for index in range(len(vl))]
	zerolist = [0 for index in range(len(vl))]

	ax.plot3D(tl,maxlist,vl,"black",linestyle="-",label='Velocity')
	ax.plot3D(tl,maxlist,al,"red",linestyle="-",label='Acceleration')
	ax.plot3D(tl,maxlist,jl,"green",linestyle="-",label='Jerk')

	ax.plot3D(tl,odomx,odomy,"blue",linestyle='-',label='Path')
	ax.plot3D(zerolist,odomx,odomy,"orange",linestyle='-',label='Path')

	#z_major_locator = MultipleLocator(1)
	#y_major_locator = MultipleLocator(1)
	#ax.yaxis.set_major_locator(MultipleLocator(5.))
	#ax.zaxis.set_major_locator(MultipleLocator(5.))
	#ax.axis("equal")
	#ax.set_xlim([-22,8])
	#ax.set_ylim([0,10])
	ax.set_aspect("auto")


	ax.set_xlabel('Frame')
	ax.set_ylabel('x[m]')
	ax.set_zlabel('y[m]')
	#ax.set_box_aspect((None))
	#ax.auto_scale_xyz([0,150],[-22,5],[0,10])
	#ax.set(aspect=1.0/ax.get_date_ratio()*1)
	ax.set_xlim(0,None)
	ax.set_ylim(midy-max_range,midy+max_range)
	ax.set_zlim(midz-max_range,midz+max_range)

	ax.legend()
	plt.gca().view_init(26,-45)
	#plt.show()

def jerkxydgraph(config,odomx,odomy,jl):

	fig = plt.figure(figsize=(7,4),tight_layout = True)
	img = plt.imread(config.map)
	ax = fig.add_subplot(111)
	#clist = [[0,1,0,0.1],'red']
	clist = ['#C8C8C8','black']
	divider = make_axes_locatable(ax)
	cax = divider.append_axes('right',size='5%',pad=0.1)
	newcm = LinearSegmentedColormap.from_list('chaos',clist)
	#cbar = mpl.colorbar.ColorbarBase(ax,cmap=newcm,norm=mpl.colors.Normalize(vmin=0,vmax=1))
	#cbar.set_clim(0,1)

	jl2=[]
	for a in range(len(jl)):
		jl2.append(abs(jl[a]))
		#jl2.append(abs(jl[a]))
		#if abs(jl[a]) >= 1.0:#np.mean(jl)+1*np.var(jl):
			#jl2.append(np.mean(jl)+1*np.var(jl))
		#	jl2.append(1.0)
		#else :
		#	jl2.append(abs(jl[a]))

	mappable = ax.scatter(x=odomx,y=odomy,c=jl2,label='Velocity',s=0.5,cmap=newcm,vmin=0,vmax=1)
	ax.set_aspect("equal")
	ax.set_xlabel('x[m]')
	ax.set_ylabel('y[m]')
	ax.set(facecolor=[0.5,0.5,0.5])
	ax.imshow(img,extent=[-32,32,-18,18])
	pp = fig.colorbar(mappable,cax=cax,label='Jerk['+'m/$ \mathit{s}^{3}$'+']')
	#fig.savefig("lin35-1")
	plt.show()

def localminimum_xy_graph(config,odomx,odomy,jl,x0,y0,xg,yg):

	fig = plt.figure(figsize=(7,3),tight_layout = True)
	img = plt.imread(config.map)
	x_mesh,y_mesh,goal_potential = to_goal_potential(x0,y0,xg,yg)
	ob_potential = to_ob_potential(config.map)

	z_mesh = goal_potential + ob_potential
	for i in range(z_mesh.shape[0]):
		for j in range(z_mesh.shape[1]):
			if z_mesh[i,j] >1.0:
				z_mesh[i,j] = 1.0

	ax = fig.add_subplot(111)
	openx = []
	openy = []
	closedx = []
	closedy = []
	for i in range(len(jl)):
		if jl[i] == 0:
			openx.append(odomx[i])
			openy.append(odomy[i])
		else:
			closedx.append(odomx[i])
			closedy.append(odomy[i])
	ax.scatter(x=openx,y=openy,c='g',label='open',s=10)
	ax.scatter(x=closedx,y=closedy,c='r',label='local_minimum',s=10)
	ax.set_aspect("equal")
	ax.set_xlabel('x[m]')
	ax.set_ylabel('y[m]')
	ax.contour(x_mesh,y_mesh,z_mesh,5,cmap = plt.cm.Blues)
	ax.imshow(img,extent=[-32,32,-18,18])
	ax.legend()
	#fig.savefig("dwa3.5-1")
	plt.show()

def twodgraph(config,vl,al,jl):
	xl = range(len(vl))
	xl2 = range(len(al))
	xl3 = range(len(jl))

	fig = plt.figure(figsize=(7,3))
	ax_velocity = HostAxes(fig, [0.1, 0.15, 0.65, 0.8])
	ax_acceleration = ParasiteAxes(ax_velocity, sharex=ax_velocity)
	ax_jerk = ParasiteAxes(ax_velocity, sharex=ax_velocity)

	ax_velocity.parasites.append(ax_acceleration)
	ax_velocity.parasites.append(ax_jerk)

	ax_velocity.axis['right'].set_visible(False)
	ax_velocity.axis['top'].set_visible(False)
	ax_acceleration.axis['right'].set_visible(True)
	ax_acceleration.axis['right'].major_ticklabels.set_visible(True)
	ax_acceleration.axis['right'].label.set_visible(True)

	ax_velocity.set_ylabel('Velocity[m/s]')
	ax_velocity.set_xlabel('Frame')
	ax_acceleration.set_ylabel('Acceleration['+'m/$ \mathit{s}^{2}$'+']')
	ax_jerk.set_ylabel('Jerk['+'m/$ \mathit{s}^{3}$'+']')

	jerk_axisline = ax_jerk.get_grid_helper().new_fixed_axis
	ax_jerk.axis['right2'] = jerk_axisline(loc='right', axes=ax_jerk, offset=(50,0))

	fig.add_axes(ax_velocity)

	curve_velocity, = ax_velocity.plot(xl, vl, label="Velocity", color='black',linewidth=1)
	curve_acceleration, = ax_acceleration.plot(xl2, al, label="Acceleraion", color='black',linewidth=1,linestyle='dashed')
	curve_jerk, = ax_jerk.plot(xl3, jl, label="Jerk", color='black',linewidth=1,linestyle='dotted')

	ax_velocity.legend()
	ax_velocity.legend(loc='lower right')

	ax_acceleration.axis['right'].label.set_color('black')
	ax_jerk.axis['right2'].label.set_color('black')

	ax_acceleration.axis['right'].major_ticks.set_color('black')
	ax_jerk.axis['right2'].major_ticks.set_color('black')

	ax_acceleration.axis['right'].major_ticklabels.set_color('black')
	ax_jerk.axis['right2'].major_ticklabels.set_color('black')

	ax_acceleration.axis['right'].line.set_color('black')
	ax_jerk.axis['right2'].line.set_color('black')

	ax_velocity.set_ylim(0.0,config.max_speed+0.5)
	ax_acceleration.set_ylim(-config.max_accel*2,config.max_accel*2)
	ax_jerk.set_ylim(-config.max_vjerk*2,config.max_vjerk*2)
	#fig.savefig("lin35-2")
	plt.show()

def twodgraph3(config,vl,al,jl):
	xl = range(len(vl))
	xl2 = range(len(al))
	xl3 = range(len(jl))

	fig = plt.figure(figsize=(7,3))
	ax_velocity = HostAxes(fig, [0.1, 0.15, 0.65, 0.8])
	ax_acceleration = ParasiteAxes(ax_velocity, sharex=ax_velocity)
	ax_jerk = ParasiteAxes(ax_velocity, sharex=ax_velocity)

	ax_velocity.parasites.append(ax_acceleration)
	ax_velocity.parasites.append(ax_jerk)

	ax_velocity.axis['right'].set_visible(False)
	ax_velocity.axis['top'].set_visible(False)
	ax_acceleration.axis['right'].set_visible(True)
	ax_acceleration.axis['right'].major_ticklabels.set_visible(True)
	ax_acceleration.axis['right'].label.set_visible(True)

	ax_velocity.set_ylabel('Velocity[m/s]')
	ax_velocity.set_xlabel('Frame')
	ax_acceleration.set_ylabel('Acceleration['+'m/$ \mathit{s}^{2}$'+']')
	ax_jerk.set_ylabel('Jerk['+'m/$ \mathit{s}^{3}$'+']')

	jerk_axisline = ax_jerk.get_grid_helper().new_fixed_axis
	ax_jerk.axis['right2'] = jerk_axisline(loc='right', axes=ax_jerk, offset=(50,0))

	fig.add_axes(ax_velocity)

	curve_velocity, = ax_velocity.plot(xl, vl, label="Velocity", color='black',linewidth=1)
	curve_acceleration, = ax_acceleration.plot(xl2, al, label="Acceleraion", color='black',linewidth=1,linestyle='dashed')
	curve_jerk, = ax_jerk.plot(xl3, jl, label="Jerk", color='black',linewidth=1,linestyle='dotted')

	ax_velocity.legend()
	ax_velocity.legend(loc='lower right')

	ax_acceleration.axis['right'].label.set_color('black')
	ax_jerk.axis['right2'].label.set_color('black')

	ax_acceleration.axis['right'].major_ticks.set_color('black')
	ax_jerk.axis['right2'].major_ticks.set_color('black')

	ax_acceleration.axis['right'].major_ticklabels.set_color('black')
	ax_jerk.axis['right2'].major_ticklabels.set_color('black')

	ax_acceleration.axis['right'].line.set_color('black')
	ax_jerk.axis['right2'].line.set_color('black')

	ax_velocity.set_ylim(-config.max_yawrate-0.5,config.max_yawrate+0.5)
	ax_acceleration.set_ylim(-config.max_dyawrate,config.max_dyawrate)
	ax_jerk.set_ylim(-config.max_wjerk*2,config.max_wjerk*2)
	#fig.savefig("lin35-2")
	plt.show()

def twodgraph2(config,vl,al,jl):
	xl = range(len(vl))
	xl2 = range(len(al))
	xl3 = range(len(jl))

	fig = plt.figure(figsize=(7,3))
	ax_velocity = HostAxes(fig, [0.15, 0.15, 0.65, 0.8])
	ax_acceleration = ParasiteAxes(ax_velocity, sharex=ax_velocity)
	ax_jerk = ParasiteAxes(ax_velocity, sharex=ax_velocity)

	ax_velocity.parasites.append(ax_acceleration)
	ax_velocity.parasites.append(ax_jerk)

	ax_velocity.axis['right'].set_visible(False)
	ax_velocity.axis['top'].set_visible(False)
	ax_acceleration.axis['right'].set_visible(True)
	ax_acceleration.axis['right'].major_ticklabels.set_visible(True)
	ax_acceleration.axis['right'].label.set_visible(True)

	ax_velocity.set_ylabel('Velocity[m/s]')
	ax_velocity.set_xlabel('Frame')
	ax_acceleration.set_ylabel('Rightwheel[m/s]')
	ax_jerk.set_ylabel('Lefttwheel[m/s]')

	jerk_axisline = ax_jerk.get_grid_helper().new_fixed_axis
	ax_jerk.axis['right2'] = jerk_axisline(loc='right', axes=ax_jerk, offset=(40,0))

	fig.add_axes(ax_velocity)

	curve_velocity, = ax_velocity.plot(xl, vl, label="Velocity", color='black',linewidth=1)
	curve_acceleration, = ax_acceleration.plot(xl2, al, label="Rightwheel", color='black',linewidth=1,linestyle='dashed')
	curve_jerk, = ax_jerk.plot(xl3, jl, label="Lefttwheel", color='black',linewidth=1,linestyle='dotted')

	ax_velocity.legend()
	ax_velocity.legend(loc='lower right')

	ax_acceleration.axis['right'].label.set_color('black')
	ax_jerk.axis['right2'].label.set_color('black')

	ax_acceleration.axis['right'].major_ticks.set_color('black')
	ax_jerk.axis['right2'].major_ticks.set_color('black')

	ax_acceleration.axis['right'].major_ticklabels.set_color('black')
	ax_jerk.axis['right2'].major_ticklabels.set_color('black')

	ax_acceleration.axis['right'].line.set_color('black')
	ax_jerk.axis['right2'].line.set_color('black')

	ax_velocity.set_ylim(0.0,config.max_speed+0.5)
	ax_acceleration.set_ylim(0.0,config.max_speed+0.5)
	ax_jerk.set_ylim(0.0,config.max_speed+0.5)
	#fig.savefig("1")
	plt.show()

def to_goal_potential(x0,y0,xg,yg):
	x = np.linspace(-32,32,64)
	y = np.linspace(-18,18,36)
	total = math.sqrt((xg - x0)**2 + (yg - y0)**2)
	x_mesh,y_mesh = np.meshgrid(x,y,indexing='ij')
	z_mesh = (np.sqrt((x_mesh-xg)**2 + (y_mesh-yg)**2) / total)
	for i in range(z_mesh.shape[0]) :
		for j in range(z_mesh.shape[1]) :
			if z_mesh[i,j] >= 1.0:
				z_mesh[i,j] = 1.0
	return x_mesh,y_mesh,z_mesh

def to_ob_potential(source_path):
	x = np.linspace(-32,32,64)
	y = np.linspace(-18,18,36)
	x_mesh,y_mesh = np.meshgrid(x,y,indexing='ij')
	ob = []
	mapp = []
	image = Image.open(source_path).convert('1')
	image = image.resize((64,36))
	image = np.matrix(image,dtype='float')
	binary = np.reshape(image,(36,64))
	for i in range(binary.shape[1]):
		for j in range(binary.shape[0]):
			if binary[j,i] == 0:
				ob.append((j,i))
	for i in range(64):
		for j in range(36):
			mapp.append((j,i))
	z_mesh = distance.cdist(mapp,ob,'euclidean')
	z_mesh = np.min(z_mesh,axis=1)
	z_mesh = np.reshape(z_mesh,(64,36))
	z_mesh = np.fliplr(z_mesh)
	for i in range(z_mesh.shape[0]):
		for j in range(z_mesh.shape[1]):
			if z_mesh[i,j] >= 1.2:
				z_mesh[i,j] = 0
			elif z_mesh[i,j] <= 0.2:
				z_mesh[i,j] = 1.0
			else:
				z_mesh[i,j] = z_mesh[i,j] - 0.2
	return z_mesh
def localminimum_xy_graph_animation_both(config,odomx,odomy,odomth,localx,localy,jl,x0,y0,xg,yg):
	fig = plt.figure(figsize=(7,3),tight_layout = True)
	img = plt.imread(config.map)
	x_mesh,y_mesh,goal_potential = to_goal_potential(x0,y0,xg,yg)
	ob_potential = to_ob_potential(config.map)

	z_mesh = goal_potential + ob_potential
	for i in range(z_mesh.shape[0]):
		for j in range(z_mesh.shape[1]):
			if z_mesh[i,j] >1.0:
				z_mesh[i,j] = 1.0

	ax = fig.add_subplot(111)
	openx = []
	openy = []
	closedx = []
	closedy = []
	figManager = plt.get_current_fig_manager()
	figManager.resize(*figManager.window.maxsize())

	ax.set_aspect("equal")
	ax.set_xlabel('x[m]')
	ax.set_ylabel('y[m]')
	ax.contour(x_mesh,y_mesh,z_mesh,5,cmap = plt.cm.Blues)
	ax.imshow(img,extent=[-32,32,-18,18])
	ax.legend()
	for i in range(len(odomx)):
		if jl[i] == 0:
			ax.scatter(localx[i],localy[i],c='r',s=10)
			ax.arrow(odomx[i],odomy[i],1*math.cos(odomth[i]),1*math.sin(odomth[i]),head_length=0.2,head_width=0.2,color='g')
		else:
			ax.scatter(localx[i],localy[i],c='r',s=10)
			ax.arrow(odomx[i],odomy[i],1*math.cos(odomth[i]),1*math.sin(odomth[i]),head_length=0.2,head_width=0.2,color='r')
		plt.pause(0.01)

		#fig.savefig("outdoor+dwa+both"+str(i))
	plt.show()
def draw_traj(config):
	fig = plt.figure()
	ax = fig.add_subplot(111)
	av = 0.05#2
	aw = 5.24

	jv = 0.5
	jw = 1.31

	dw = [1.5-av*0.1,1.5+av*0.1,0-aw*0.1,0+aw*0.1]
	#dw =[1.475,1.525,-4.0,4.0]
	config.v_reso = av*0.1
	config.yawrate_reso = aw*0.1
	#config.predict_time = 1.6
	xinit = [0,0,0,1.0,0]
	config.v_all_list = [1.5]
	config.a_all_list = [0]
	jinduanx = []
	jinduany = []
	yuanduanx = []
	yuanduany = []
	jizhunx = []
	jizhuny = []
	#axins = ax.inset_axes((0.2,0.4,0.4,0.2))
	#axins = inset_axes(ax,width='20%',height='40%',loc='center',bbox_to_anchor=(0.5,0.1,1,1),bbox_transform=ax.transAxes)
	#config.max_accel = 0.04
	#for v in np.arange(dw[0], dw[1]+0.5*config.v_reso,config.v_reso):
	#for j in np.arange(-config.max_jerk, config.max_jerk+0.5*config.j_reso,config.j_reso):
	for jv in [-0.5,0,0.5]:
		#for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
		for jw in [-1.31,0,1.31]:
			config.predict_time=0.8
			v_predict_list,av = linear_velocity_generator(jv,config)
			w_predict_list,aw = angular_velocity_generator(jw,config)
			traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)
			print(v_predict_list[-1],w_predict_list[-1])
			if jv == -0.5:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			elif jv == 0:
				jizhunx.append(traj[-1:,0])
				jizhuny.append(traj[-1:,1])
			else:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
			plt.plot(traj[:,0],traj[:,1],c='r',linewidth=1)
	#config.predict_time=0.8
	for jv in [-0.5,0,0.5]:
		for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
		#for jw in [-1.31,0,1.31]:
			v_predict_list,av = linear_velocity_generator(jv,config)
			print(v_predict_list[-1],w)
			traj = calc_trajectory(xinit, v_predict_list, w, config)
			if jv == -0.5:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			elif jv == 0:
				jizhunx.append(traj[-1:,0])
				jizhuny.append(traj[-1:,1])
			else:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
			plt.plot(traj[:,0],traj[:,1],c='g',linewidth=1)
	#plt.plot(jizhuny,jizhunx,c='black',linewidth=1)
	#plt.plot(jinduany,jinduanx,c='r',linewidth=1,linestyle='dotted')
	#plt.plot(yuanduany,yuanduanx,c='r',linewidth=1,linestyle='dotted')
	#print((max(yuanduany)-max(jinduany))/2)
	#axins.plot(jizhuny,jizhunx,c='black',linewidth=1)
	#axins.plot(jinduany,jinduanx,c='r',linewidth=1,linestyle='dotted')
	#axins.plot(yuanduany,yuanduanx,c='r',linewidth=1,linestyle='dotted')
	#print(math.sqrt((jinduany[0]-yuanduany[0])**2+(jinduanx[0]-yuanduanx[0])**2)/2)
	jinduanx = []
	jinduany = []
	yuanduanx = []
	yuanduany = []
	#print(v_predict_list)
	for v in np.arange(dw[0], dw[1]+0.5*config.v_reso, config.v_reso):
	#for v in np.arange(dw[0], dw[1]+0.5*config.v_reso,config.v_reso):
		for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
			print(v,w)
			#print(v)
			v_predict_list = [v]*8
			v_predict_list.insert(0,1.0)
			traj = calc_trajectory(xinit, v_predict_list, w, config)
			if v == dw[0]:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			else:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
			plt.plot(traj[:,0],traj[:,1],c='b',linewidth=1)
	#plt.plot(jinduany,jinduanx,c='b',linewidth=1,linestyle='dashed')
	#plt.plot(yuanduany,yuanduanx,c='b',linewidth=1,linestyle='dashed')
	#axins.plot(jinduany,jinduanx,c='b',linewidth=1,linestyle='dashed')
	#axins.plot(yuanduany,yuanduanx,c='b',linewidth=1,linestyle='dashed')
	#print((max(yuanduany)-max(jinduany))/2)
	#print(math.sqrt((jinduany[0]-yuanduany[0])**2+(jinduanx[0]-yuanduanx[0])**2)/2)
	plt.plot(0,0,c='r',label='DWA-JJ')
	plt.plot(0,0,c='g',label='DWA-J')
	plt.plot(0,0,c='b',label='DWA')

	ax.set_aspect('equal')
	plt.xlabel("x[m]")
	plt.ylabel("y[m]")
	#mark_inset(ax,axins,loc1=4,loc2=1,fc='none',lw=1)
	#axins.set_ylim(1.28,1.42)
	#axins.set_xlim(-0.05,0.05)
	#plt.ylim(-0.25,1.5)
	#plt.xlim(-1.1,1.1)
	plt.legend()
	plt.show()

def main():
	print(__file__ + " start!!")
	profiler = Profiler()
	profiler.start()
	config = Config()
	obs = Obstacles()
	subOdom = rospy.Subscriber("base_pose_ground_truth", Odometry, config.assignOdomCoords)
	subLaser = rospy.Subscriber("/base_scan", LaserScan, obs.assignObs2, config)
	subCmd = rospy.Subscriber("/cmd_vel", Twist, config.callcmd)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	speed = Twist()
	# initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
	x = np.array([config.x, config.y, config.th, config.current_v, config.current_w])
	# initial linear and angular velocities
	u = np.array([0.0, 0.0])
	odomx = [config.x]
	odomy = [config.y]
	odomth = [config.th]
	localgoalx = [config.goalX]
	localgoaly = [config.goalY]
	al = [0] # acceleration list
	vl = [0] # velocity list
	vjl = [0] # jerk list
	wjl = [0]
	sl = [0]
	vright = [0]
	vleft = [0]
	t = 0
	while not rospy.is_shutdown():
		t=t+1
		#x = np.array([config.x, config.y, config.th, config.current_v, config.current_w])
		if (atGoal(config,x) == False) and (sl[-1]!=-1):#local minimum test use
		#if (atGoal(config,x) == False):#usual use
			u,state = dwa_control(x, u, config, obs.obst)
			x[0] = config.x
			x[1] = config.y
			x[2] = config.th
			x[3] = u[0]
			x[4] = u[1]
			if x[3] >= config.max_speed:
				speed.linear.x = config.max_speed
			else:
				speed.linear.x = x[3]
			speed.angular.z = x[4]
		else:
			# if at goal then stay there until new goal published
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			pub.publish(speed)
			break
		#plt.clf()
		#plt.scatter(config.x,config.y)
		#plt.pause(0.1)
		#plt.ioff()
		#fig = plt.figure(figsize=(7,3),tight_layout = True)
		#ax = fig.add_subplot(111)
		#img = plt.imread(config.map)
		#ax.legend()
		#plt.pause(0.01)
		#plt.show()
		#print(config.state)
		vright.append((2*speed.linear.x+0.35*speed.angular.z)/2)
		vleft.append((2*speed.linear.x-0.35*speed.angular.z)/2)
		#sl.append(state)
		config.v_all_list.append(speed.linear.x)#velocity
		config.av_all_list.append((config.v_all_list[-1]-config.v_all_list[-2])/config.dt)#acceleration
		config.w_all_list.append(speed.angular.z)#velocity
		config.aw_all_list.append((config.w_all_list[-1]-config.w_all_list[-2])/config.dt)#acceleration

		#config.a_all_list.append(u[3])
		#print(u[2])
		vjl.append(u[2])
		wjl.append(u[3])
		#jl.append((config.v_all_list[-1]-config.v_all_list[-2])/config.dt)
		#jl.append((config.a_all_list[-1]-config.a_all_list[-2])/config.dt)#jerk
		odomx.append(config.x)
		odomy.append(config.y)
		odomth.append(config.th)
		#localgoalx.append(config.localX)
		#localgoaly.append(config.localY)
		pub.publish(speed)
		config.r.sleep()
		#time.sleep(0.1)
	#print(config.local_minimum)
	#print(max(config.v_all_list))
	#print(max(config.a_all_list))
	print(max(vjl))
	print(t,config.predict_time,config.laser_reso)
	twodgraph(config,config.v_all_list,config.av_all_list,vjl)
	twodgraph3(config,config.w_all_list,config.aw_all_list,wjl)

	twodgraph2(config,config.v_all_list,vleft,vright)
	#threedgraph(odomx,odomy,vl,al,jl)
	jerkxydgraph(config,odomx,odomy,vjl)
	#localminimum_xy_graph(config,odomx,odomy,sl,config.startX,config.startY,config.goalX,config.goalY)
	#localminimum_xy_graph_animation_both(config,odomx,odomy,odomth,localgoalx,localgoaly,sl,config.startX,config.startY,config.goalX,config.goalY)

	profiler.stop()
	year=time.localtime().tm_year
	month=time.localtime().tm_mon
	day=time.localtime().tm_mday
	hour=time.localtime().tm_hour
	minute=time.localtime().tm_min
	second=time.localtime().tm_sec
	date = str(month)+'_'+str(day)+'_'+str(hour)+'_'+str(minute)+'_'+str(second)
	f = open(str(t)+"_"+str(config.predict_time)+"_"+str(config.laser_reso)+"_"+date+'.txt','w')

	#profiler.print(file=f)

	profiler.print()


if __name__ == '__main__':
	rospy.init_node('dwa')
	main()
	#config = Config()
	#draw_traj(config)
