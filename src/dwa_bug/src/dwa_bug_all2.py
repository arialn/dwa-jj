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
import os
import rospy
import math
import numpy as np
import pandas as pd
import datetime
import time
import matplotlib as mpl
import mpl_toolkits.mplot3d
import sympy
import mpmath as mp
import random
import sys

from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
from scipy.optimize import newton,root,curve_fit
from scipy.interpolate import interp1d,PchipInterpolator
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression,Perceptron
from sklearn.metrics import mean_squared_error,r2_score
from sklearn.model_selection import train_test_split

from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap,LinearSegmentedColormap
from matplotlib.ticker import MultipleLocator
#from sympy import *
from pathsolver import solver
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from mpl_toolkits.axes_grid1 import make_axes_locatable
from mpl_toolkits.axes_grid1.inset_locator import mark_inset,inset_axes
#from pynverse import inversefunc
from pyinstrument import Profiler
import PIL.Image as Image



class Config():
	# simulation parameters
	def __init__(self):
		self.dt = 0.05 # [s]
		self.predict_time = 2.0

		# robot parameter
		self.max_speed = 2.0#1.5  # [m/s]
		self.min_speed = 0.0  # [m/s]
		self.max_yawrate = 6.28  # [rad/s]360 degree

		self.max_accel = 1.0#2.0  # [m/ss]
		self.max_dyawrate = 5.24  # [rad/ss]300 degree

		self.max_vjerk = 0.5  # [m/sss]
		self.max_wjerk = 1.31  # [m/sss]

		self.v_reso = self.max_accel*self.dt*0.5#0.1
		self.yawrate_reso = self.max_dyawrate*self.dt*0.5#0.2 # [rad/s]

		self.av_reso = self.max_accel/2
		self.aw_reso = self.max_dyawrate/2

		self.jv_reso = self.max_vjerk*0.5
		self.jw_reso = self.max_wjerk*0.5

		self.robot_radius = 0.2 # [m]
		self.sensor_range = 5.0

		self.goalX = 18.0
		self.goalY = 0.0
		self.startX = -18.0
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

		self.error = []

		self.v_all_list = [0.0]
		self.w_all_list = [0.0]
		self.av_all_list = [0.0]
		self.aw_all_list = [0.0]
		self.cal_time1 = []
		self.cal_time2 = []
		self.cal_time3 = []

		self.ob_nums = []
		self.path = []
		self.remain_path = self.path

		self.stop_dist = 0.5

		self.v_weight = 0.1
		self.j_weight = 0.1

		self.type = "a" #dwa a aa j jj
		if self.type == "jj":
			self.predict_time = 2.0
		self.state = "goto"
		self.laser_reso = []
		self.map = '../world/1181.png'

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
		self.current_v = msg.twist.twist.linear.x
		self.current_w = msg.twist.twist.angular.z

class Obstacles():
	def __init__(self):
		# Set of coordinates of obstacles in view
		#global path
		#self.active_from = 'f1_form'
		self.obst = set()
		self.obst_rc = set()
		self.bestDist = 10000.0
		self.bestsudden = 10000.0
		self.state='goto'
		self.obst_nums = []

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
		angle2goal = math.atan2(config.goalY-config.y,config.goalX-config.x)
		sensorIndex = int((angle2goal-msg.angle_min)/msg.angle_increment)
		config.laser_reso = deg
		for angle in self.myRange(0,deg-1,1):
			distance = msg.ranges[angle]
			# only record obstacles that are within 4 metres away
			if (distance < config.sensor_range):
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
		self.obst_rc = set()
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
			#print('min_dist = ',min(msg.ranges))
			# only record obstacles that are within 4 metres away
			if (distance < config.sensor_range):
				scanTheta = -math.pi + angle * msg.angle_increment
				xb = distance * math.cos(scanTheta)#obstacle in robot coordinate
				yb = distance * math.sin(scanTheta)#obstacle in robot coordinate
				BP = np.array([xb,yb,0,1])
				AP = np.dot(ABT, BP)#obstacle in world coordinate

				self.obst.add((AP[0],AP[1]))
				self.obst_rc.add((xb,yb))
				if msg.ranges[angle-1] >= 5.0:
					self.sudden.append([AP[0],AP[1],scanTheta])
					self.suddeninrobot.append(np.dot(ANTI_ABT,[AP[0],AP[1],0,1]))
				#if (abs(msg.ranges[angle]-msg.ranges[angle-1]) > config.discthresh):
				#   self.sudden.append([AP[0],AP[1],scanTheta])
				#   self.suddeninrobot.append(np.dot(ANTI_ABT,[AP[0],AP[1],0,1]))

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

	def assignObs3(self, msg, config):

		deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
		self.obst = set()   # reset the obstacle set to only keep visible objects
		self.obst_rc = set()
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
		i=0
		#bestsudden = 0
		config.laser_reso = deg
		del_list = []
		for angle in self.myRange(0,deg-1,1):
		#for angle in self.myRange(int(deg*0.25),int(deg*0.75),1):
			distance = msg.ranges[angle]
			if (distance < config.sensor_range):
				i=i+1
				scanTheta = -math.pi + angle * msg.angle_increment
				xb = distance * math.cos(scanTheta)#obstacle in robot coordinate
				yb = distance * math.sin(scanTheta)#obstacle in robot coordinate
				BP = np.array([xb,yb,0,1])
				AP = np.dot(ABT, BP)#obstacle in world coordinate
				self.obst.add((AP[0],AP[1]))
				self.obst_rc.add((xb,yb))
		self.obst_nums.append(i)
		if isinstance(config.path,list) == True:
			if len(config.path) !=0:
				if config.type == "j" or config.type == "a"or config.type == "a2":
					for i in range(len(config.path)-1,-1,-1):
						if (math.sqrt((config.path[i][0]-config.x)**2+(config.path[i][1]-config.y)**2)) < config.sensor_range:
							config.path.remove(config.path[i])
				if config.type =="dwa":
					for i in config.path:
						if (math.sqrt((i[0]-config.x)**2+(i[1]-config.y)**2)) < config.sensor_range:
							config.path.remove(i)

				if len(config.path) == 0:
					config.localX = config.goalX
					config.localY = config.goalY
				else:
					config.localX = config.path[0][0]
					config.localY = config.path[0][1]
			else:
				config.localX = config.goalX
				config.localY = config.goalY
		#print(config.path)
		#print("===================")


# Model to determine the expected position of the robot after moving along trajectory
def line(route):
	xc=[]
	yc=[]
	for i in (range(0,len(route))):
		x=route[i][0]
		y=route[i][1]
		xc.append(x)
		yc.append(y)
	return xc,yc

def get_png(config):
	img=cv2.imread(config.map,0)
	x,y = img.shape[0:2]
	img_one_of_four = cv2.resize(img,(int(y/10),int(x/10)))
	return img_one_of_four

def ast(config):
	#global obj, solve
	solve=solver()
	fig,ax=plt.subplots()
	#grid=obj.returnGrid()
	img = get_png(config)
	block_img = ~img
	grid = block_img
	#print(grid[2200,2000])  #1
	print(grid.shape)
	img_h, img_w = grid.shape
	#ax.imshow(grid,cmap=plt.cm.Spectral)
	#plt.show()
	dx = img_w*0.5
	dy = img_h*0.5
	ds = 0.1
	st = (config.startX,config.startY)
	ed = (config.goalX,config.goalY)
	start=(int((st[1]/ds)+dy),int((st[0]/ds)+dx))
	#start =(500,1500)
	print("start point:{}".format(start))
	#end=(124,340)#end point:(2480, 4405)  No path
	end = (int((ed[1]/ds)+dy),int((ed[0]/ds)+dx))
	print("end point:{}".format(end))

	route=solve.astar(start,end,grid)
	if(route==False):
		print("No path")
		return 0
	route+=[start]
	route=route[::-1]

	path=[]
	for i in route:
		px = (dy-i[0])*ds
		py = (i[1]-dx)*ds
		path.append((py,px))
	print('=================')
	xc,yc=line(route)
	fig,ax=plt.subplots()
	#ax.imshow(grid,cmap=plt.cm.Spectral)
	#ax.plot(yc,xc,color="black")
	#ax.scatter(start[1],start[0])
	#ax.scatter(end[1],end[0])
	#plt.show()
	#path = []
	return path

def motion(x, u, dt):
	# x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
	x = [x[0],x[1],x[2],u[0],u[1]]
	DT = np.array([[1,0,0,dt*math.cos(x[2]),0],
					[0,1,0,dt*math.sin(x[2]),0],
					[0,0,1,0,dt],
					[0,0,0,1,0],
					[0,0,0,0,1]])
	x = np.dot(DT,x)
	return x

def acceleration_to_velocity(a,config,mode):
	time = config.dt
	predict_list = []
	if mode =='av':
		predict_list.append(config.v_all_list[-1])
		while time <= config.predict_time:
			if predict_list[-1]+a*config.dt >= 0:
				predict_list.append(min(config.max_speed,predict_list[-1]+a*config.dt))
			else:
				predict_list.append(max(0,predict_list[-1]+a*config.dt))
			time = time + config.dt
	elif mode =='aw':
		predict_list.append(config.w_all_list[-1])
		while time <= config.predict_time:
			if predict_list[-1]+a*config.dt >= 0:
				predict_list.append(min(config.max_yawrate,w_predict_list[-1]+aw*config.dt))
			else:
				predict_list.append(max(-config.max_yawrate,w_predict_list[-1]+aw*config.dt))
			time = time + config.dt
	del(predict_list[0])
	return predict_list

def j_linear_velocity_generator(jv,config):
	time = config.dt
	av_predict_list = []
	v_predict_list = []
	av_predict_list.append(config.av_all_list[-1])
	v_predict_list.append(config.v_all_list[-1])

	while time < config.predict_time:
		if av_predict_list[-1]+jv*config.dt >= 0:
			av_predict_list.append(min(config.max_accel,av_predict_list[-1]+jv*config.dt))
		else:
			av_predict_list.append(max(-config.max_accel,av_predict_list[-1]+jv*config.dt))
		time = time + config.dt
	del(av_predict_list[0])

	for i in range(len(av_predict_list)):
		if v_predict_list[-1]+av_predict_list[i]*config.dt >= 0:
			v_predict_list.append(min(config.max_speed,v_predict_list[-1]+av_predict_list[i]*config.dt))
		else:
			v_predict_list.append(max(0,v_predict_list[-1]+av_predict_list[i]*config.dt))
	del(v_predict_list[0])

	return v_predict_list,av_predict_list

def j_angular_velocity_generator(jw,config):
	time = config.dt
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
	#for i in range(len(w_predict_list)):
	#   if w_predict_list[i] == config.max_yawrate:
	#       aw_predict_list[i] = 0
	del(w_predict_list[0])
	return w_predict_list

# Determine the dynamic window from robot configurations
def calc_dynamic_window(x, config):
	v1 = x[3] - config.max_accel * config.dt
	v2 = x[3] + config.max_accel * config.dt
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
	return dw

# Calculate a trajectory sampled across a prediction time
def calc_trajectory(xinit, v_predict_list, w_predict_list, config):
	x = np.array(xinit)
	traj = np.array(x)  # many motion models stored per trajectory
	time = config.dt

	for i in range(len(v_predict_list)):
		x = motion(x, [v_predict_list[i], w_predict_list[i]], config.dt)
		traj = np.vstack((traj, x))
		time = time + config.dt # next sample

	return traj

# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, obst,obst_rc,t):

	xinit = x[:]
	max_cost = -100000000
	state = 0
	max_u = u
	to_goal_dist = math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2)
	max_v_predict_list = []

	# evaluate all trajectory with sampled input in dynamic window
	if config.type == "dwa":
		deaccel_dist = 0.7
		line = 0
		for v in np.arange(dw[0], dw[1]+0.5*config.v_reso,config.v_reso):
			for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
				v_predict_list = [v]*int(config.predict_time/config.dt)
				w_predict_list = [w]*int(config.predict_time/config.dt)
				traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)
				#to_goal_cost,stop_mark,indexremain,remain_end = calc_to_goal_cost(traj, config)
				to_goal_cost = calc_to_goal_cost(traj, config)
				ob_cost,_ = calc_obstacle_cost(traj, obst, config)
				odom_angle_cost = calc_odom_angle_cost(traj,config)
				add_cost = 0
				speed_cost = abs(v/config.max_speed)

				final_cost = (1.0*add_cost+0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +config.v_weight*speed_cost)
				if final_cost > max_cost:
					max_cost = final_cost
					max_u = [v, w]
		if max_cost <= 0:
			state = -1
		return max_u,state,line

	elif config.type == "a":
		per_pair1=[]
		per_pair2=[]
		for av in np.arange(-config.max_accel, config.max_accel+0.5*config.av_reso,config.av_reso):
			for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
				v_predict_list = acceleration_to_velocity(av,config,'av')
				w_predict_list = [w]*int(config.predict_time/config.dt)
				######################################################
				time_s1 = time.perf_counter()
				traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)
				to_goal_cost1 = calc_to_goal_cost(traj, config)
				ob_cost1,_ = calc_obstacle_cost(traj, obst, config)
				time_e1 = time.perf_counter()
				######################################################
				time_s2 = time.perf_counter()
				ob_cost3,_,xl,yl,s1= calc_obstacle_cost4(config, obst_rc,av,v_predict_list,w)
				to_goal_cost2 = calc_to_goal_cost2(config,xl,yl)
				time_e2 = time.perf_counter()
				per_pair1.append(time_e1-time_s1)
				per_pair2.append(time_e2-time_s2)
				######################################################
				odom_angle_cost = 0#calc_odom_angle_cost(traj,config)
				speed_cost = np.mean(v_predict_list)/config.max_speed
				final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost1+2.0*ob_cost1 +config.v_weight*speed_cost)
				#final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost2+2.0*ob_cost3 +config.v_weight*speed_cost)
				#config.error.append(ob_cost3-ob_cost1)
				#if ob_cost3-ob_cost1>1:
				#	print(v_predict_list,av,w)
				if final_cost > max_cost:
					max_cost = final_cost
					max_u = [v_predict_list[0], w,av]
		#config.cal_time1.append(sum(per_pair1))
		config.cal_time1.append(sum(per_pair1))
		config.cal_time2.append(sum(per_pair2))
		if max_cost <= 0:
			state = -1
		line = 0
		return max_u,state,line
	elif config.type == "aa":
		for av in np.arange(-config.max_accel, config.max_accel+0.5*config.av_reso,config.av_reso):
			for aw in np.arange(-config.max_dyawrate, config.max_dyawrate+0.5*config.aw_reso, config.aw_reso):
				v_predict_list = acceleration_to_velocity(av,config,'av')
				w_predict_list = acceleration_to_velocity(aw,config,'aw')
				traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)
				to_goal_cost = calc_to_goal_cost(traj, config)
				ob_cost = calc_obstacle_cost(traj, obst, config)
				odom_angle_cost = calc_odom_angle_cost(traj,config)
				aw_cost = 1-abs(aw/config.max_dyawrate)
				if v_predict_list[-1] < config.max_speed/2:
					av_cost = (av/config.max_accel)
					if to_goal_dist >= deaccel_dist:
						speed_cost = v_predict_list[-1]/config.max_speed
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*av_cost+0.01*aw_cost)
					else:
						state = 1
						speed_cost = 1-abs(v_predict_list[-1]/config.max_speed)
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +10.0*speed_cost+0.01*av_cost+0.01*aw_cost)
				else:
					av_cost = -(av/config.max_accel)
					if to_goal_dist >= deaccel_dist:
						speed_cost = v_predict_list[-1]/config.max_speed
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*av_cost+0.01*aw_cost)
					else:
						state = 1
						speed_cost = 1-abs(v_predict_list[-1]/config.max_speed)
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +10.0*speed_cost+0.01*av_cost+0.01*aw_cost)
				if final_cost >= max_cost:
					max_cost = final_cost
					max_u = [v_predict_list[0], w_predict_list[0],av,aw]
		if final_cost <= 0:
			state = -1
		return max_u,state

	elif config.type == "j":
		line = 0
		deaccel_dist = 5.0

		for jv in np.arange(-config.max_vjerk, config.max_vjerk+0.5*config.jv_reso,config.jv_reso):
			for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
				line = line+1
				v_predict_list,a_predict_list = j_linear_velocity_generator(jv,config)
				traj = calc_trajectory(xinit, v_predict_list, w, config)# generate trajectory
				#to_goal_cost,stop_mark,indexremain,remain_end = calc_to_goal_cost(traj, config)
				to_goal_cost = calc_to_goal_cost(traj, config)
				ob_cost,_ = calc_obstacle_cost(traj, obst, config)
				odom_angle_cost = calc_odom_angle_cost(traj,config)
				add_cost = 0
				al=[]
				ssl=[]
				#stm = 0
				for i in range(len(v_predict_list)-1):
					al.append((v_predict_list[i+1]-v_predict_list[i])/config.dt)
				#if stop_mark == 1:
				#   pass
					#al.append(0)

				for i in range(len(al)-1):
					ssl.append(abs(al[i+1]-al[i])/config.dt)

				#speed_cost = v_predict_list[-1]/config.max_speed
				speed_cost = (sum(v_predict_list)/len(v_predict_list))/config.max_speed

				jv_cost = 1-abs(round(max(ssl),1)/config.max_vjerk)
				#jv_cost2 = 1-abs(round(min(ssl),1)/config.max_vjerk)
				#jv_cost = max(jv_cost1,jv_cost2)
				if jv_cost < 0:
					jv_cost = -100000000
					line = line-1
				#if to_goal_dist >= deaccel_dist:
				#   speed_cost = speed_cost
				#   final_cost = (1.0*add_cost+0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*jv_cost)
				#else:
				#   speed_cost = abs(1-speed_cost)
				#   final_cost = (1.0*add_cost+0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +10.0*speed_cost+0.01*jv_cost)
				final_cost = (1.0*add_cost+0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +config.v_weight*speed_cost+config.j_weight*jv_cost)

				if final_cost >= max_cost:
					max_cost = final_cost
					max_u = [v_predict_list[0], w,jv]
					max_v_predict_list = v_predict_list
					#stm = stop_mark
					#if v_predict_list[-1]<= 0.05*config.max_speed and stop_mark ==1:
					#   end_mark = 0
						#end_mark = 0

					#else:
					#   end_mark = 0
		if max_cost <= 0:
			#print('minus')
			state = -1
		#print("remain_path",line,"abandon_path",25-line)
		#return max_u,state,stm,line,max_v_predict_list
		return max_u,state,line

	elif config.type == "jj":
		#config.predict_time = 2.0
		deaccel_dist = 2.5
		for jv in np.arange(-config.max_vjerk, config.max_vjerk+0.5*config.jv_reso,config.jv_reso):
			for jw in np.arange(-config.max_wjerk, config.max_wjerk+0.5*config.jw_reso,config.jw_reso):
				v_predict_list = j_linear_velocity_generator(jv,config)
				w_predict_list = j_angular_velocity_generator(jw,config)
				traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)# generate trajectory
				to_goal_cost = calc_to_goal_cost(traj, config)
				ob_cost = calc_obstacle_cost(traj, obst, config)
				odom_angle_cost = calc_odom_angle_cost(traj,config)
				jw_cost = 1-abs(jw/config.max_wjerk)
				if v_predict_list[-1] < config.max_speed/2:
					jv_cost = (jv/config.max_vjerk)
					if to_goal_dist >= deaccel_dist:
						speed_cost = v_predict_list[-1]/config.max_speed
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*jv_cost+0.01*jw_cost)
					else:
						state = 1
						speed_cost = 1-abs(v_predict_list[-1]/config.max_speed)
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +10.0*speed_cost+0.01*jv_cost+0.01*jw_cost)
				else:
					jv_cost = -(jv/config.max_vjerk)
					if to_goal_dist >= deaccel_dist:
						speed_cost = v_predict_list[-1]/config.max_speed
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +0.1*speed_cost+0.01*jv_cost+0.01*jw_cost)
					else:
						state = 1
						speed_cost = 1-abs(v_predict_list[-1]/config.max_speed)
						final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +10.0*speed_cost+0.01*jv_cost+0.01*jw_cost)
				if final_cost >= max_cost:
					max_cost = final_cost
					max_u = [v_predict_list[0], w_predict_list[0],jv,jw]
		if final_cost <= 0:
			state = -1
		return max_u,state
# Calculate obstacle cost inf: collision, 0:free

def calc_obstacle_cost(traj, obst, config):
	minr = 1000
	# Loop through every obstacle in set and calc Pythagorean distance
	# Use robot radius to determine if collision
	path = traj[:,0:2]
	obst = list(obst)
	if len(obst) == 0:
		minr = 1000
	else:
		obst = np.array(obst)
		dist_matrix = distance.cdist(path,obst,metric="euclidean")
		minr = np.min(dist_matrix)
	if minr <= config.robot_radius:
		cost = -10000  # collision
	elif config.robot_radius< minr <= 1.2:
		cost = minr - 0.2
	else:
		cost = 1
	return cost,minr

def calc_obstacle_cost2(config,obst_rc,av,w):
	minr = 1000
	v0 = config.v_all_list[-1]
	w0 = w
	def ff_(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			if 0 <=t<=acct:
				ff_ = (v0+av*t)*math.cos(w0*t)
			elif acct<t<=config.predict_time:
				ff_ = config.max_speed*math.cos(w0*t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			if 0 <=t<=acct:
				ff_ = (v0+av*t)*math.cos(w0*t)
			elif acct<t<=config.predict_time:
				ff_ = 0
			else:
				print('out of range')
		else:
			ff_ = v0*math.cos(w0*t)
		return ff_
	def gg_(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			if 0 <=t<=acct:
				gg_ = (v0+av*t)*math.sin(w0*t)
			elif acct<t<=config.predict_time:
				gg_ = config.max_speed*math.sin(w0*t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			if 0 <=t<=acct:
				gg_ = (v0+av*t)*math.sin(w0*t)
			elif acct<t<=config.predict_time:
				gg_ = 0
			else:
				print('out of range')
		else:
			gg_ = v0*math.sin(w0*t)
		return gg_
	def ff(t):
		#print(t)
		if av > 0:
			acct = (config.max_speed-v0)/av
			x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
			dx1 = x0 -0
			if 0 <=t<=acct:
				f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
			elif acct <t<=config.predict_time:
				x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
				x2 = config.max_speed*math.sin(w0*acct)/w0
				dx2 = x2-x1
				f = config.max_speed*math.sin(w0*t)/w0-dx2
			else:
				print('av > 0')
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
			dx1 = x0 -0
			if 0 <=t<=acct:
				f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
			elif acct <t<=config.predict_time:
				f = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
			else:
				print(v0,vn,av)
				print('t=',t)
				print('av < 0')
				print('out of range')
				print('==============')
		else:
			x0 = (v0*math.sin(w0*0)/w0)
			dx1 = x0 - 0
			f = (v0*math.sin(w0*t)/w0)-dx1
		return f
	def gg(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
			dy1 = y0 -0
			if 0 <=t<=acct:
				g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
			elif acct <t<=config.predict_time:
				y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
				y2 = -config.max_speed*math.cos(w0*acct)/w0
				dy2 = y2-y1
				g = -config.max_speed*math.cos(w0*t)/w0-dy2
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
			dy1 = y0 -0
			if 0 <=t<=acct:
				g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
			elif acct <t<=config.predict_time:
				g = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
			else:
				print('out of range')
		else:
			y0 = -(v0*math.cos(w0*0)/w0)
			dy1 = y0 - 0
			g = -(v0*math.cos(w0*t)/w0)-dy1
		return g

	vn = v0 + av*config.predict_time
	#wn = w0 + aw*config.predict_time
	if vn >= config.max_speed:
		vn = config.max_speed
	elif vn <= 0:
		vn =0

	if w0 != 0:
		xend = ff(config.predict_time)
		yend = gg(config.predict_time)
		#print(xend,yend)
		if av !=0 :
			midtime = abs((math.sqrt((v0**2+vn**2)*0.5)-v0)/av)
		else:
			midtime = config.predict_time*0.5
		#print(v0,vn,midtime)
		xmid = ff(midtime)
		ymid = gg(midtime)

		dx21 = (v0/w0)*math.sin(w0*config.predict_time)-xend
		dy21 = (v0/w0)*(1-math.cos(w0*config.predict_time))-yend
		dx31 = (vn/w0)*math.sin(w0*config.predict_time)-xend
		dy31 = (vn/w0)*(1-math.cos(w0*config.predict_time))-yend

		dx41 = (v0/w0)*math.sin(w0*midtime)-xmid
		dy41 = (v0/w0)*(1-math.cos(w0*midtime))-ymid
		dx51 = (vn/w0)*math.sin(w0*midtime)-xmid
		dy51 = (vn/w0)*(1-math.cos(w0*midtime))-ymid

		c2 = [0,v0/w0]
		c3 = [0,vn/w0]
		c4 = [0-dx21,v0/w0-dy21]
		c5 = [0-dx31,vn/w0-dy31]
		c6 = [0-dx41,v0/w0-dy41]
		c7 = [0-dx51,vn/w0-dy51]
		if ff_(config.predict_time) != 0:
			k = gg_(config.predict_time)/ff_(config.predict_time)
			k = -1/k
		else:
			k = 1
		b = yend-k*xend
		anglemax = math.atan2(yend-b,xend)
		for i in obst_rc.copy():
			angle = math.atan2(i[1]-b,i[0])
			if (w0 > 0 and -0.5*math.pi < angle < anglemax) or (w0 < 0 and anglemax < angle < 0.5*math.pi) :
				r2 = math.sqrt((i[0]-c2[0])**2+(i[1]-c2[1])**2)
				r3 = math.sqrt((i[0]-c3[0])**2+(i[1]-c3[1])**2)
				r4 = math.sqrt((i[0]-c4[0])**2+(i[1]-c4[1])**2)
				r5 = math.sqrt((i[0]-c5[0])**2+(i[1]-c5[1])**2)
				r6 = math.sqrt((i[0]-c6[0])**2+(i[1]-c6[1])**2)
				r7 = math.sqrt((i[0]-c7[0])**2+(i[1]-c7[1])**2)
				if r3-(vn/abs(w0))>0 and r5-(vn/abs(w0))>0 and r7-(vn/abs(w0))>0 :
					if av > 0:
						#dist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						dist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
					else:
						#dist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						dist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				elif r2-(v0/abs(w0))<0 and r4-(v0/abs(w0))<0 and r4-(v0/abs(w0))<0 :
					if av > 0:
						#dist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						dist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
					else:
						#dist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						dist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
				else:
					dist = 0
			else:
				dist1 = math.sqrt((i[0]-xend)**2+(i[1]-yend)**2)
				dist2 = math.sqrt((i[0])**2+(i[1])**2)
				dist = min(dist1,dist2)
			if dist <= minr:
				minr = dist
	else:
		if av > 0:
			acct = (config.max_speed-v0)/av
			xend = (v0*acct+0.5*av*acct**2)+config.max_speed*(config.predict_time-acct)
		elif av < 0:
			acct = abs(v0/av)
			if acct > config.predict_time:
				xend = (v0*config.predict_time+0.5*av*config.predict_time**2)
			else:
				xend = (v0*acct+0.5*av*acct**2)
		else:
			xend = v0*config.predict_time
		yend = 0
		for i in obst_rc.copy():
			if 0 <= i[0] <= xend:
				dist = abs(i[1])
			else:
				dist1 = math.sqrt((i[0]-xend)**2+(i[1]-0)**2)
				dist2 = math.sqrt((i[0])**2+(i[1])**2)
				dist = min(dist1,dist2)
			if dist <= minr:
				minr = dist
	print("o2=",minr)
	if minr <= config.robot_radius:
		cost = -10000  # collision
	elif config.robot_radius< minr <= 1.2:
		cost = minr - 0.2
	else:
		cost = 1
	return cost,minr,xend,yend,vn

def calc_obstacle_cost3(config,obst_rc,av,w):
	minr = 1000
	v0 = config.v_all_list[-1]
	w0 = w
	def ff_(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			if 0 <=t<=acct:
				ff_ = (v0+av*t)*math.cos(w0*t)
			elif acct<t<=config.predict_time:
				ff_ = config.max_speed*math.cos(w0*t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			if 0 <=t<=acct:
				ff_ = (v0+av*t)*math.cos(w0*t)
			elif acct<t<=config.predict_time:
				ff_ = 0
			else:
				print('out of range')
		else:
			ff_ = v0*math.cos(w0*t)
		return ff_
	def gg_(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			if 0 <=t<=acct:
				gg_ = (v0+av*t)*math.sin(w0*t)
			elif acct<t<=config.predict_time:
				gg_ = config.max_speed*math.sin(w0*t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			if 0 <=t<=acct:
				gg_ = (v0+av*t)*math.sin(w0*t)
			elif acct<t<=config.predict_time:
				gg_ = 0
			else:
				print('out of range')
		else:
			gg_ = v0*math.sin(w0*t)
		return gg_
	def ff(t):
		#print(t)
		if av > 0:
			acct = (config.max_speed-v0)/av
			x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
			dx1 = x0 -0
			if 0 <=t<=acct:
				f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
			elif acct <t<=config.predict_time:
				x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
				x2 = config.max_speed*math.sin(w0*acct)/w0
				dx2 = x2-x1
				f = config.max_speed*math.sin(w0*t)/w0-dx2
			else:
				print('av > 0')
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
			dx1 = x0 -0
			if 0 <=t<=acct:
				f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
			elif acct <t<=config.predict_time:
				f = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
			else:
				print(v0,vn,av)
				print('t=',t)
				print('av < 0')
				print('out of range')
				print('==============')
		else:
			x0 = (v0*math.sin(w0*0)/w0)
			dx1 = x0 - 0
			f = (v0*math.sin(w0*t)/w0)-dx1
		return f
	def gg(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
			dy1 = y0 -0
			if 0 <=t<=acct:
				g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
			elif acct <t<=config.predict_time:
				y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
				y2 = -config.max_speed*math.cos(w0*acct)/w0
				dy2 = y2-y1
				g = -config.max_speed*math.cos(w0*t)/w0-dy2
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
			dy1 = y0 -0
			if 0 <=t<=acct:
				g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
			elif acct <t<=config.predict_time:
				g = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
			else:
				print('out of range')
		else:
			y0 = -(v0*math.cos(w0*0)/w0)
			dy1 = y0 - 0
			g = -(v0*math.cos(w0*t)/w0)-dy1
		return g

	vn = v0 + av*config.predict_time
	if vn >= config.max_speed:
		vn = config.max_speed
	elif vn <= 0:
		vn =0

	if w0 != 0:
		xend = ff(config.predict_time)
		yend = gg(config.predict_time)
		if av !=0 :
			midtime = abs((math.sqrt((v0**2+vn**2)*0.5)-v0)/av)
		else:
			midtime = config.predict_time*0.5

		xmid = ff(midtime)
		ymid = gg(midtime)
		dx21 = (v0/w0)*math.sin(w0*config.predict_time)-xend
		dy21 = (v0/w0)*(1-math.cos(w0*config.predict_time))-yend
		dx31 = (vn/w0)*math.sin(w0*config.predict_time)-xend
		dy31 = (vn/w0)*(1-math.cos(w0*config.predict_time))-yend

		dx41 = (v0/w0)*math.sin(w0*midtime)-xmid
		dy41 = (v0/w0)*(1-math.cos(w0*midtime))-ymid
		dx51 = (vn/w0)*math.sin(w0*midtime)-xmid
		dy51 = (vn/w0)*(1-math.cos(w0*midtime))-ymid

		inside_circle_center = np.array([[0,v0/w0],#2,4,6
										#[0-dx21,v0/w0-dy21],
										[0-dx41,v0/w0-dy41]])
		outside_circle_center=np.array([[0,vn/w0],#3,5,7
										#[0-dx31,vn/w0-dy31],
										[0-dx51,vn/w0-dy51]])

		if ff_(config.predict_time) != 0:
			k = gg_(config.predict_time)/ff_(config.predict_time)
			k = -1/k
		else:
			k = 1
		b = yend-k*xend
		anglemax = math.atan2(yend-b,xend)

		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
		else:
			obst_rc_array = np.array(obst_rc_list)
			angel_matrix = np.arctan2(obst_rc_array[:,1]-b,obst_rc_array[:,0])
			angel_matrix2 = angel_matrix.reshape(-1,1)
			obst_rc_array = np.hstack((obst_rc_array,angel_matrix2))
			if w0 > 0:
				inrange_condition = (-0.5*math.pi<obst_rc_array[:,2])&(obst_rc_array[:,2]<anglemax)
				obst_inrange = obst_rc_array[inrange_condition]
				obst_outrange = obst_rc_array[~inrange_condition]
				dist_to_outside_center = distance.cdist(outside_circle_center,obst_inrange[:,0:2],metric="euclidean")
				dist_to_inside_center = distance.cdist(inside_circle_center,obst_inrange[:,0:2],metric="euclidean")
				if np.any(obst_inrange):
					if av > 0:
						mt1 = np.absolute(dist_to_outside_center - (vn/abs(w0)))
						mt2 = np.absolute(dist_to_inside_center - (v0/abs(w0)))
						c1 = mt1.min(0)>0
						c2 = mt2.max(0)<0
						mt = np.where(c1,0.5*mt1.max(0)+0.5*mt2.min(0),
										np.where(c2,0.5*mt1.min(0)+0.5*mt2.max(0),0))
						dist_a = np.min(mt)
					else:
						mt1 = np.absolute(dist_to_outside_center - (vn/abs(w0)))
						mt2 = np.absolute(dist_to_inside_center - (v0/abs(w0)))
						c1 = mt1.min(0)>0
						c2 = mt2.max(0)<0
						mt = np.where(c1,0.5*mt1.min(0)+0.5*mt2.max(0),
										np.where(c2,0.5*mt1.max(0)+0.5*mt2.min(0),0))
						dist_a = np.min(mt)
				else:
					dist_a = 1000
				if np.any(obst_outrange):
					dist1 = np.min(distance.cdist(np.array([[xend,yend]]),obst_outrange[:,0:2],metric="euclidean"))
					dist2 = np.min(distance.cdist(np.array([[0,0]]),obst_outrange[:,0:2],metric="euclidean"))
					dist_b = min(dist1,dist2)
				else:
					dist_b = 1000
				dist = min(dist_a,dist_b)
			elif w0 <0:
				inrange_condition = (anglemax<obst_rc_array[:,2])&(obst_rc_array[:,2]<0.5*math.pi)
				obst_inrange = obst_rc_array[inrange_condition]
				obst_outrange = obst_rc_array[~inrange_condition]
				dist_to_outside_center = distance.cdist(outside_circle_center,obst_inrange[:,0:2],metric="euclidean")
				dist_to_inside_center = distance.cdist(inside_circle_center,obst_inrange[:,0:2],metric="euclidean")
				if np.any(obst_inrange):
					if av > 0:
						mt1 = np.absolute(dist_to_outside_center - (vn/abs(w0)))
						mt2 = np.absolute(dist_to_inside_center - (v0/abs(w0)))
						c1 = mt1.min(0)>0
						c2 = mt2.max(0)<0
						mt = np.where(c1,0.5*mt1.max(0)+0.5*mt2.min(0),
										np.where(c2,0.5*mt1.min(0)+0.5*mt2.max(0),0))
						dist_a = np.min(mt)
					else:
						mt1 = np.absolute(dist_to_outside_center - (vn/abs(w0)))
						mt2 = np.absolute(dist_to_inside_center - (v0/abs(w0)))
						c1 = mt1.min(0)>0
						c2 = mt2.max(0)<0
						mt = np.where(c1,0.5*mt1.min(0)+0.5*mt2.max(0),
										np.where(c2,0.5*mt1.max(0)+0.5*mt2.min(0),0))
						dist_a = np.min(mt)
				else:
					dist_a = 1000
				if np.any(obst_outrange):
					dist1 = np.min(distance.cdist(np.array([[xend,yend]]),obst_outrange[:,0:2],metric="euclidean"))
					dist2 = np.min(distance.cdist(np.array([[0,0]]),obst_outrange[:,0:2],metric="euclidean"))
					dist_b = min(dist1,dist2)
				else:
					dist_b = 1000
				dist = min(dist_a,dist_b)
		if dist <= minr:
			minr = dist
		if minr <= config.robot_radius:
			cost = -10000  # collision
		elif config.robot_radius< minr <= 1.2:
			cost = minr - 0.2
		else:
			cost = 1
	else:
		if av > 0:
			acct = (config.max_speed-v0)/av
			xend = (v0*acct+0.5*av*acct**2)+config.max_speed*(config.predict_time-acct)
		elif av < 0:
			acct = abs(v0/av)
			if acct > config.predict_time:
				xend = (v0*config.predict_time+0.5*av*config.predict_time**2)
			else:
				xend = (v0*acct+0.5*av*acct**2)
		else:
			xend = v0*config.predict_time
		yend = 0
		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
		else:
			obst_rc_array = np.array(obst_rc_list)
			inrange_condition = (0<obst_rc_array[:,0])&(obst_rc_array[:,0]<xend)
			inrange_matrix = obst_rc_array[inrange_condition]
			outrange_matrix = obst_rc_array[~inrange_condition]
			if np.any(inrange_matrix):
				dist_a = np.min(np.absolute(inrange_matrix[:,1]))
			else:
				dist_a = 1000
			if np.any(outrange_matrix):
				dist_b1 = np.min(distance.cdist(np.array([[0,0]]),outrange_matrix,metric="euclidean"))
				dist_b2 = np.min(distance.cdist(np.array([[xend,yend]]),outrange_matrix,metric="euclidean"))
				dist_b = min(dist_b1,dist_b2)
			else:
				dist_b = 1000
			dist = min(dist_a,dist_b)
		if dist <= minr:
			minr = dist
		if minr <= config.robot_radius:
			cost = -10000  # collision
		elif config.robot_radius< minr <= 1.2:
			cost = minr - 0.2
		else:
			cost = 1
	return cost,minr,xend,yend,vn

def calc_obstacle_cost4(config,obst_rc,av,v_predict_list,w):
	minr = 1000

	vn = v_predict_list[-1]
	v0 = config.v_all_list[-1]
	w0 = w
	def ff_(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			if 0 <=t<=acct:
				ff_ = (v0+av*t)*math.cos(w0*t)
			elif acct<t<=config.predict_time:
				ff_ = config.max_speed*math.cos(w0*t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			if 0 <=t<=acct:
				ff_ = (v0+av*t)*math.cos(w0*t)
			elif acct<t<=config.predict_time:
				ff_ = 0
			else:
				print('out of range')
		else:
			ff_ = v0*math.cos(w0*t)
		return ff_
	def gg_(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			if 0 <=t<=acct:
				gg_ = (v0+av*t)*math.sin(w0*t)
			elif acct<t<=config.predict_time:
				gg_ = config.max_speed*math.sin(w0*t)
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			if 0 <=t<=acct:
				gg_ = (v0+av*t)*math.sin(w0*t)
			elif acct<t<=config.predict_time:
				gg_ = 0
			else:
				print('out of range')
		else:
			gg_ = v0*math.sin(w0*t)
		return gg_
	def ff(t):
		#print(t)
		if av > 0:
			acct = (config.max_speed-v0)/av
			x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
			dx1 = x0 -0
			if 0 <=t<=acct:
				f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
			elif acct <t<=config.predict_time:
				x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
				x2 = config.max_speed*math.sin(w0*acct)/w0
				dx2 = x2-x1
				f = config.max_speed*math.sin(w0*t)/w0-dx2
			else:
				print('av > 0')
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
			dx1 = x0 -0
			if 0 <=t<=acct:
				f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
			elif acct <t<=config.predict_time:
				f = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
			else:
				print(v0,vn,av)
				print('t=',t)
				print('av < 0')
				print('out of range')
				print('==============')
		else:
			x0 = (v0*math.sin(w0*0)/w0)
			dx1 = x0 - 0
			f = (v0*math.sin(w0*t)/w0)-dx1
		return f
	def gg(t):
		if av > 0:
			acct = (config.max_speed-v0)/av
			y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
			dy1 = y0 -0
			if 0 <=t<=acct:
				g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
			elif acct <t<=config.predict_time:
				y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
				y2 = -config.max_speed*math.cos(w0*acct)/w0
				dy2 = y2-y1
				g = -config.max_speed*math.cos(w0*t)/w0-dy2
			else:
				print('out of range')
		elif av < 0:
			acct = abs(v0/av)
			y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
			dy1 = y0 -0
			if 0 <=t<=acct:
				g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
			elif acct <t<=config.predict_time:
				g = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
			else:
				print('out of range')
		else:
			y0 = -(v0*math.cos(w0*0)/w0)
			dy1 = y0 - 0
			g = -(v0*math.cos(w0*t)/w0)-dy1
		return g

	if w0 != 0:
		xend = ff(config.predict_time)
		yend = gg(config.predict_time)
		if av !=0 :
			midtime = abs((math.sqrt((v0**2+vn**2)*0.5)-v0)/av)
		else:
			midtime = config.predict_time*0.5

		xmid = ff(midtime)
		ymid = gg(midtime)
		dx21 = (v0/w0)*math.sin(w0*config.predict_time)-xend
		dy21 = (v0/w0)*(1-math.cos(w0*config.predict_time))-yend
		dx31 = (vn/w0)*math.sin(w0*config.predict_time)-xend
		dy31 = (vn/w0)*(1-math.cos(w0*config.predict_time))-yend

		dx41 = (v0/w0)*math.sin(w0*midtime)-xmid
		dy41 = (v0/w0)*(1-math.cos(w0*midtime))-ymid
		dx51 = (vn/w0)*math.sin(w0*midtime)-xmid
		dy51 = (vn/w0)*(1-math.cos(w0*midtime))-ymid

		inside_circle_center = np.array([[0,v0/w0],#2,4,6
										#[0-dx21,v0/w0-dy21],
										[0-dx41,v0/w0-dy41]])
		outside_circle_center=np.array([[0,vn/w0],#3,5,7
										#[0-dx31,vn/w0-dy31],
										[0-dx51,vn/w0-dy51]])

		if ff_(config.predict_time) != 0:
			k = gg_(config.predict_time)/ff_(config.predict_time)
			k = -1/k
		else:
			k = 1
		b = yend-k*xend
		anglemax = math.atan2(yend-b,xend)

		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
			obst_inrange=np.array([0])
		else:
			obst_rc_array = np.array(obst_rc_list)
			angel_matrix = np.arctan2(obst_rc_array[:,1]-b,obst_rc_array[:,0])
			angel_matrix2 = angel_matrix.reshape(-1,1)
			obst_rc_array = np.hstack((obst_rc_array,angel_matrix2))
			if w0 > 0:
				inrange_condition = (-0.5*math.pi<obst_rc_array[:,2])&(obst_rc_array[:,2]<anglemax)
			else:
				inrange_condition = (anglemax<obst_rc_array[:,2])&(obst_rc_array[:,2]<0.5*math.pi)

			obst_inrange = obst_rc_array[inrange_condition]
			obst_outrange = obst_rc_array[~inrange_condition]
			dist_to_outside_center = distance.cdist(outside_circle_center,obst_inrange[:,0:2],metric="euclidean")
			dist_to_inside_center = distance.cdist(inside_circle_center,obst_inrange[:,0:2],metric="euclidean")
			if np.any(obst_inrange):
				mt1 = np.absolute(dist_to_outside_center - (vn/abs(w0)))
				mt2 = np.absolute(dist_to_inside_center - (v0/abs(w0)))
				c1 = mt1.min(0)>0
				c2 = mt2.max(0)<0
				if av > 0:
					mt = np.where(c1,0.5*mt1.max(0)+0.5*mt2.min(0),
									np.where(c2,0.5*mt1.min(0)+0.5*mt2.max(0),0))
				else:
					mt = np.where(c1,0.5*mt1.min(0)+0.5*mt2.max(0),
									np.where(c2,0.5*mt1.max(0)+0.5*mt2.min(0),0))
				dist_a = np.min(mt)
			else:
				dist_a = 1000

			if np.any(obst_outrange):
				dist_b1 = np.min(distance.cdist(np.array([[xend,yend]]),obst_outrange[:,0:2],metric="euclidean"))
				dist_b2 = np.min(distance.cdist(np.array([[0,0]]),obst_outrange[:,0:2],metric="euclidean"))
				dist_b = min(dist_b1,dist_b2)
			else:
				dist_b = 1000
			dist = min(dist_a,dist_b)

	#	if dist <= minr:
	#		minr = dist
	#	if minr <= config.robot_radius:
	#		cost = -10000  # collision
	#	elif config.robot_radius< minr <= 1.2:
	#		cost = minr - 0.2
	#	else:
	#		cost = 1
	else:
		if av > 0:
			acct = (config.max_speed-v0)/av
			xend = (v0*acct+0.5*av*acct**2)+config.max_speed*(config.predict_time-acct)
		elif av < 0:
			acct = abs(v0/av)
			if acct > config.predict_time:
				xend = (v0*config.predict_time+0.5*av*config.predict_time**2)
			else:
				xend = (v0*acct+0.5*av*acct**2)
		else:
			xend = v0*config.predict_time
		yend = 0
		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
			obst_inrange=np.array([0])
		else:
			obst_rc_array = np.array(obst_rc_list)
			inrange_condition = (0<obst_rc_array[:,0])&(obst_rc_array[:,0]<xend)
			obst_inrange = obst_rc_array[inrange_condition]
			obst_outrange = obst_rc_array[~inrange_condition]
			if np.any(obst_inrange):
				dist_a = np.min(np.absolute(obst_inrange[:,1]))
			else:
				dist_a = 1000
			if np.any(obst_outrange):
				dist_b1 = np.min(distance.cdist(np.array([[0,0]]),obst_outrange,metric="euclidean"))
				dist_b2 = np.min(distance.cdist(np.array([[xend,yend]]),obst_outrange,metric="euclidean"))
				dist_b = min(dist_b1,dist_b2)
			else:
				dist_b = 1000
			dist = min(dist_a,dist_b)
	if dist <= minr:
		minr = dist
	if minr <= config.robot_radius:
		cost = -10000  # collision
	elif config.robot_radius< minr <= 1.2:
		cost = minr - 0.2
	else:
		cost = 1

	return cost,minr,xend,yend,obst_inrange.shape[0]


# Calculate goal cost via Pythagorean distance to robot
def calc_to_goal_cost(traj, config):
	# If-Statements to determine negative vs positive goal/trajectory position
	# traj[-1,0] is the last predicted X coord position on the trajectory
	#total = math.sqrt((config.x-config.localX)**2 + (config.y-config.localY)**2)
	remain_end = math.sqrt((config.localX-traj[-1,0])**2 + (config.localY-traj[-1,1])**2)
	#remain = (np.sqrt((config.localX-traj[:,0])**2 + (config.localY-traj[:,1])**2))
	#minremain = min(remain)
	#global_remain = (np.sqrt((config.goalX-traj[:,0])**2 + (config.goalY-traj[:,1])**2))
	#global_remain_end = math.sqrt((config.goalX-traj[-1,0])**2 + (config.goalY-traj[-1,1])**2)
	#stop_mark_list = global_remain <= config.stop_dist

	#indexremain = np.argmin(remain)

	#if global_remain_end <= config.stop_dist:
	#	stop_mark = 1
	#	cost = 1 -(global_remain_end / 30)
	#else:
	#	stop_mark = 0
	#if remain_end >5:
	#   cost = 0
	#else:
	cost = 1 -(remain_end / 5)
	return cost

def calc_to_goal_cost2(config,xl,yl):
	ABT = np.array([[math.cos(config.th),-math.sin(config.th),0,config.x],
					[math.sin(config.th),math.cos(config.th),0,config.y],
					[0,0,1,0],
					[0,0,0,1]])
	BP = [xl,yl,0,1]

	CP = np.dot(ABT,BP)
	#total = math.sqrt((config.x-config.localX)**2 + (config.y-config.localY)**2)
	remain = math.sqrt((config.localX-CP[0])**2 + (config.localY-CP[1])**2)
	cost = 1 -(remain / 5)

	return cost

def calc_odom_angle_cost(traj, config):
	#function in dwa paper
	angle_end1 = math.atan2(config.localY-traj[-1,1],config.localX-traj[-1,0])#-traj[-1,2]
	angle_end2 = traj[-1,2]
	cost = 1-abs((angle_end1-angle_end2)/math.pi)
	return cost

def calc_traj_angle_cost(traj, config):
	angle_start = traj[0,2]
	angle_end = traj[-1,2]
	cost = 1-abs((angle_end-angle_start)/math.pi)
	return cost


# Begin DWA calculations
def dwa_control(x, u, config,obst,obst_rc,t):
	# Dynamic Window control
	dw = calc_dynamic_window(x, config)
	u,state,line = calc_final_input(x, u, dw, config, obst,obst_rc,t)
	return u,state,line
# Determine whether the robot has reached its goal
def atGoal(config, x):
	# check at goal
	if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) <= config.stop_dist:
		if x[3] <= 0.1*config.max_speed:
			return True
	return False

def path_jerk_graph(config,odomx,odomy,jl):
	#print(odomx,odomy,jl)
	fig = plt.figure(figsize=(8,3.0),tight_layout = True)
	img = plt.imread(config.map)
	ax = fig.add_subplot(111)
	clist = [[0,1,0,0.1],'red']
	#clist = ['#C8C8C8','black']
	divider = make_axes_locatable(ax)
	cax = divider.append_axes('right',size='5%',pad=0.1)
	newcm = LinearSegmentedColormap.from_list('chaos',clist)

	jl2=[]
	sl = []
	for a in range(len(jl)):
		#if abs(jl[a]) >=1:
		#   jl[a] = 0.5
		jl2.append(abs(jl[a]))
	for i in range(len(odomx)):
		if i % 50 == 0:

			sl.append(30)
		else:
			sl.append(5)

	mappable = ax.scatter(x=odomx,y=odomy,c=jl2,label='Velocity',s=sl,cmap=newcm,vmin=0,vmax=0.1)
	#for i in range(len(odomx)):
	#   if i //50 == 0:
	#       ax.plot(odomx[i],odomy[i],c=jl2[i],label='Velocity',s=20,cmap=newcm,vmin=0,vmax=1)
	ax.set_aspect("equal")
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_xlim(-22.025*1.0,22.025*1.0)
	ax.set_ylim(-12.4*0.7,12.4*0.7)
	#ax.set_xlim(-32*0.8,32*0.8)
	#ax.set_ylim(-18*0.7,18*0.7)

	ax.set(facecolor=[0.5,0.5,0.5])
	ax.imshow(img,extent=[-22.025,22.025,-12.4,12.4])
	#ax.imshow(img,extent=[-32,32,-18,18])

	pp = fig.colorbar(mappable,cax=cax,label='Jerk ['+'m/$ \mathit{s}^{3}$'+']')
	time_day = time.strftime("%Y%m%d",time.localtime())
	path = '../results/{}/'.format(time_day)
	plt.savefig(path+str(config.type)+'-path'+'fig-time{}.png'.format(time.strftime("%Y%m%d-%H%M", time.localtime())), dpi=400)

def vaj_graph(config,vl,al,jl,mode):
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14
	#xl = range(len(vl))
	#xl2 = range(len(al))
	#xl3 = range(len(jl))
	xl = np.arange(0,len(vl))*0.1
	xl2 = np.arange(0,len(al))*0.1
	xl3 = np.arange(0,len(jl))*0.1

	fig = plt.figure(figsize=(8,3))
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

	ax_velocity.set_ylabel('Velocity [m/s]')
	ax_velocity.set_xlabel('Time [s]')
	ax_acceleration.set_ylabel('Acceleration ['+'m/$ \mathit{s}^{2}$'+']')
	ax_jerk.set_ylabel('Jerk ['+'m/$ \mathit{s}^{3}$'+']')

	jerk_axisline = ax_jerk.get_grid_helper().new_fixed_axis
	ax_jerk.axis['right2'] = jerk_axisline(loc='right', axes=ax_jerk, offset=(60,0))

	fig.add_axes(ax_velocity)

	curve_velocity, = ax_velocity.plot(xl, vl, label="Velocity", color='black',linewidth=1,linestyle='dashed')
	curve_acceleration, = ax_acceleration.plot(xl2, al, label="Acceleraion", color='black',linewidth=1,linestyle='dotted')
	curve_jerk, = ax_jerk.plot(xl3, jl, label="Jerk", color='black',linewidth=1)

	ax_velocity.legend()
	ax_velocity.legend(loc='upper right')

	ax_acceleration.axis['right'].label.set_color('black')
	ax_jerk.axis['right2'].label.set_color('black')

	ax_acceleration.axis['right'].major_ticks.set_color('black')
	ax_jerk.axis['right2'].major_ticks.set_color('black')

	ax_acceleration.axis['right'].major_ticklabels.set_color('black')
	ax_jerk.axis['right2'].major_ticklabels.set_color('black')

	ax_acceleration.axis['right'].line.set_color('black')
	ax_jerk.axis['right2'].line.set_color('black')

	if mode == "linear":
		ax_velocity.set_ylim(0.0,config.max_speed+0.5)
		ax_acceleration.set_ylim(-config.max_accel*2,config.max_accel*2)
		if config.type == "dwa" or config.type == "a" or config.type == "aa":
			ax_jerk.set_ylim(-config.max_accel*20,config.max_accel*20)
		else:
			ax_jerk.set_ylim(-20.0,20.0)
		time_day = time.strftime("%Y%m%d",time.localtime())
		path = '../results/{}/'.format(time_day)
		if not os.path.exists(path):
			os.makedirs(path)
		plt.savefig(path+str(config.type)+'-linear'+'fig-time{}.png'.format(time.strftime("%Y%m%d-%H%M", time.localtime())), dpi=400)
	elif mode == "angular":
		ax_velocity.set_ylim(-config.max_yawrate-0.5,config.max_yawrate+0.5)
		ax_acceleration.set_ylim(-config.max_dyawrate,config.max_dyawrate)
		if config.type == "dwa" or config.type == "a" or config.type == "aa"or config.type == "j":
			ax_jerk.set_ylim(-config.max_dyawrate*20,config.max_dyawrate*20)
		elif config.type == "jj":
			ax_jerk.set_ylim(-config.max_wjerk-0.5,config.max_wjerk+0.5)
	elif mode == "wheel":
		ax_velocity.set_ylim(0.0,config.max_speed+0.5)
		ax_acceleration.set_ylim(0.0,config.max_speed+0.5)
		ax_jerk.set_ylim(0.0,config.max_speed+0.5)
	elif mode == "line":
		ax_velocity.set_ylim(0.0,25)
		ax_acceleration.set_ylim(0.0,0)
		ax_jerk.set_ylim(0.0,0)
	elif mode == "time":
		ax_velocity.set_ylim(0.0,500)
		ax_acceleration.set_ylim(0.0,0.1)
		ax_jerk.set_ylim(0.0,0)
	plt.show()
	#plt.savefig('mode'+'fig-time{}.png'.format(time.strftime("%Y%m%d-%H%M", time.localtime())), dpi=400)

def time_graph(config,vl,al):
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14
	#xl = range(len(vl))
	#xl2 = range(len(al))
	#xl3 = range(len(jl))
	xl = np.arange(0,len(vl))*config.dt
	xl2 = np.arange(0,len(al))*config.dt

	fig = plt.figure(figsize=(8,3))
	ax_velocity = HostAxes(fig, [0.1, 0.15, 0.65, 0.8])
	ax_acceleration = ParasiteAxes(ax_velocity, sharex=ax_velocity)

	ax_velocity.parasites.append(ax_acceleration)

	ax_velocity.axis['right'].set_visible(False)
	ax_velocity.axis['top'].set_visible(False)
	ax_acceleration.axis['right'].set_visible(True)
	ax_acceleration.axis['right'].major_ticklabels.set_visible(True)
	ax_acceleration.axis['right'].label.set_visible(True)

	ax_velocity.set_ylabel('Obst_nums')
	ax_velocity.set_xlabel('Time [s]')
	ax_acceleration.set_ylabel('DWA time [s]')

	fig.add_axes(ax_velocity)

	curve_acceleration, = ax_acceleration.plot(xl2, al, label="DWA_time", color='red',linewidth=1,linestyle='solid')
	curve_velocity, = ax_velocity.plot(xl, vl, label="Obst_nums", color='black',linewidth=1,linestyle='solid')

	ax_velocity.legend()
	ax_velocity.legend(loc='upper right')

	ax_acceleration.axis['right'].label.set_color('black')
	ax_acceleration.axis['right'].major_ticks.set_color('black')
	ax_acceleration.axis['right'].major_ticklabels.set_color('black')
	ax_acceleration.axis['right'].line.set_color('black')

	ax_velocity.set_ylim(0.0,500)
	ax_acceleration.set_ylim(0,0.1)

	plt.show()

def draw_traj(config):
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14

	fig = plt.figure()
	fig.tight_layout()

	ax = fig.add_subplot(111)
	ax.set_aspect(1)

	config.jv = 0.5
	config.jw = 1.31

	config.av = 1.0#0.025#jv*0.1
	config.aw = 5.24#0.0655#jw*0.1


	#av = 0.05#2
	#aw = 0.131#5.24

	#jv = 0.5
	#jw = 1.31

	dw = [1.0-config.av*0.1,1.0+config.av*0.1,0-config.aw*0.1,0+config.aw*0.1]
	print(dw)

	#dw =[1.475,1.525,-4.0,4.0]
	config.v_reso = config.av*config.dt
	config.yawrate_reso = config.aw*0.1*0.5
	print(config.v_reso,config.yawrate_reso)

	config.av_reso = config.av
	config.aw_reso = config.aw

	config.jv_reso = config.jv
	config.jw_reso = config.jw


	#config.predict_time = 1.6
	xinit = [0,0,0,1.0,0]
	config.v_all_list = [1.0]
	config.a_all_list = [0.0]
	jinduanx = []
	jinduany = []
	yuanduanx = []
	yuanduany = []
	jizhunx = []
	jizhuny = []
	config.predict_time = 1.0
	for v in np.arange(dw[0], dw[1]+0.5*config.v_reso, config.v_reso):
		for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
			#print(v,w)
			traj = calc_trajectory(xinit, v, w, config)
			#print(traj)
			if v == dw[0]:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			elif v == dw[1]:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
	plt.plot(jinduanx,jinduany,c='b',linewidth=1,linestyle='dashed',label='Origianl method')
	plt.plot(yuanduanx,yuanduany,c='b',linewidth=1,linestyle='dashed')
	#plt.show()
	#plt.plot(traj[:,0],traj[:,1],c='black',linewidth=1)
	#plt.xlim(-0.1,1.0)
	#plt.ylim(-0.2,0.2)
	#plt.show()
	jinduanx = []
	jinduany = []
	yuanduanx = []
	yuanduany = []
	jizhunx = []
	jizhuny = []

	for av in np.arange(-config.av, config.av+0.5*config.av_reso, config.av_reso):
		for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
			v_predict_list = a_linear_velocity_generator(av,config)
			#print(av,w)
			#w = 0.524
			traj = calc_trajectory(xinit, v_predict_list, w, config)
			#print(traj)
			if v == dw[0]:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			else:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
			#if av < 0:
				#plt.plot(traj[:,0],traj[:,1],c='blue',linewidth=1)
			#elif av == 0:
				#plt.plot(traj[:,0],traj[:,1],c='red',linewidth=1)
			#else:
				#plt.plot(traj[:,0],traj[:,1],c='black',linewidth=1)
	#plt.plot(traj[:,0],traj[:,1],c='blue',linewidth=1,label='Curvature decrease')
	#plt.plot(traj[:,0],traj[:,1],c='red',linewidth=1,label='Curvature constant')
	#plt.plot(traj[:,0],traj[:,1],c='black',linewidth=1,label='Curvature increase')
	jinduanx = []
	jinduany = []
	yuanduanx = []
	yuanduany = []
	jizhunx = []
	jizhuny = []
	#plt.plot(traj[:,0],traj[:,1],c='blue',linewidth=1)
	#fig = plt.figure()
	#ax = fig.add_subplot(111)
	#ax.set_aspect("equal")
	for av in np.arange(-config.av, config.av+0.5*config.av_reso, config.av_reso):
		for aw in np.arange(-config.aw,config.aw+0.5*config.aw_reso, config.aw_reso):
			v_predict_list = a_linear_velocity_generator(av,config)
			w_predict_list = a_angular_velocity_generator(aw,config)
			#print(av,aw)
			traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)
			if v == dw[0]:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			else:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
			#plt.plot(traj[:,0],traj[:,1],c='green',linewidth=1)
	jinduanx = []
	jinduany = []
	yuanduanx = []
	yuanduany = []
	jizhunx = []
	jizhuny = []
	for jv in np.arange(-config.jv, config.jv+0.5*config.jv_reso, config.jv_reso):
		for w in np.arange(dw[2], dw[3]+0.5*config.yawrate_reso, config.yawrate_reso):
			v_predict_list,_ = j_linear_velocity_generator(jv,config)
			#print(jv,w)
			#print(v_predict_list)
			traj = calc_trajectory(xinit, v_predict_list, w, config)
			#print(traj)
			if jv == -0.5:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			elif jv == 0.5:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
	plt.plot(jinduanx,jinduany,c='r',linewidth=1,label='Proposed method')
	plt.plot(yuanduanx,yuanduany,c='r',linewidth=1)
	#plt.xlim(-0.3,0.3)
	plt.xlabel('x [m]')
	plt.ylabel('y [m]')
	plt.legend()

	plt.show()
			#print(max(traj[:,0]))
			#plt.plot(traj[:,0],traj[:,1],c='red',linewidth=1)
	#plt.xlim(-0.1,1.0)
	#plt.ylim(-0.2,0.2)
	#plt.show()

	for jv in np.arange(-config.jv, config.jv+0.5*config.jv_reso, config.jv_reso):
		for jw in np.arange(-config.jw,config.jw+0.5*config.jw_reso, config.jw_reso):
			v_predict_list = j_linear_velocity_generator(jv,config)
			w_predict_list = j_angular_velocity_generator(jw,config)
			#print(w_predict_list)
			traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)

			if v == dw[0]:
				jinduanx.append(traj[-1:,0])
				jinduany.append(traj[-1:,1])
			else:
				yuanduanx.append(traj[-1:,0])
				yuanduany.append(traj[-1:,1])
			plt.plot(traj[:,0],traj[:,1],c='pink',linewidth=1)
	#axins = ax.inset_axes((0.2,0.4,0.4,0.2))
	#axins = inset_axes(ax,width='20%',height='40%',loc='center',bbox_to_anchor=(0.5,0.1,1,1),bbox_transform=ax.transAxes)
	#config.max_accel = 0.04
	#for v in np.arange(dw[0], dw[1]+0.5*config.v_reso,config.v_reso):
	#for j in np.arange(-config.max_jerk, config.max_jerk+0.5*config.j_reso,config.j_reso):

	#plt.plot(jizhuny,jizhunx,c='black',linewidth=1)
	#plt.plot(jinduany,jinduanx,c='r',linewidth=1,linestyle='dotted')
	#plt.plot(yuanduany,yuanduanx,c='r',linewidth=1,linestyle='dotted')
	#print((max(yuanduany)-max(jinduany))/2)
	#axins.plot(jizhuny,jizhunx,c='black',linewidth=1)
	#axins.plot(jinduany,jinduanx,c='r',linewidth=1,linestyle='dotted')
	#axins.plot(yuanduany,yuanduanx,c='r',linewidth=1,linestyle='dotted')
	#print(math.sqrt((jinduany[0]-yuanduany[0])**2+(jinduanx[0]-yuanduanx[0])**2)/2)
	#jinduanx = []
	#jinduany = []
	#yuanduanx = []
	#yuanduany = []
	print(jinduanx)
	#plt.plot(jinduanx,jinduanx,c='b',linewidth=1,linestyle='dashed')
	#plt.plot(jinduany,jinduanx,c='b',linewidth=1,linestyle='dashed')
	#plt.plot(yuanduany,yuanduanx,c='b',linewidth=1,linestyle='dashed')
	#axins.plot(jinduany,jinduanx,c='b',linewidth=1,linestyle='dashed')
	#axins.plot(yuanduany,yuanduanx,c='b',linewidth=1,linestyle='dashed')
	#print((max(yuanduany)-max(jinduany))/2)
	#print(math.sqrt((jinduany[0]-yuanduany[0])**2+(jinduanx[0]-yuanduanx[0])**2)/2)
	#plt.plot(0,0,c='black',label='DWA')
	#plt.plot(0,0,c='blue',label='DWA-A')
	#plt.plot(0,0,c='green',label='DWA-AA')
	#plt.plot(0,0,c='red',label='DWA-J')
	#plt.plot(0,0,c='pink',label='DWA-JJ')


	#ax.set_aspect('equal')
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
	config = Config()
	config.path = ast(config)

	obs = Obstacles()
	subOdom = rospy.Subscriber("base_pose_ground_truth", Odometry, config.assignOdomCoords)
	subLaser = rospy.Subscriber("/base_scan", LaserScan, obs.assignObs3, config)
	#subCmd = rospy.Subscriber("/cmd_vel", Twist, config.callcmd)
	subCmd = rospy.Subscriber("odom", Odometry, config.callcmd)
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
	vjl = [0] # jerk list
	vjl_r = [0]
	wjl = [0]
	sl = [0]
	vright = [0]
	vleft = [0]
	t = 0
	td= []
	profiler = Profiler()
	profiler.start()
	end_mark = 0
	vi = 1
	ap=[]
	bp=[]
	cp=[]
	line_list = []
	while not rospy.is_shutdown():
		t=t+1
		if t >= 800:
			sl.append(-1)
		if (atGoal(config,x) == False) and t<500:
			#dwa_s = time.perf_counter()
			u,state,line= dwa_control(x, u, config, obs.obst,obs.obst_rc,t)
			#dwa_e = time.perf_counter()
			#config.cal_time.append(dwa_e-dwa_s)
			x= [config.x,config.y,config.th,u[0],u[1]]

			if x[3] >= config.max_speed:
				speed.linear.x = config.max_speed
			else:
				speed.linear.x = x[3]
			speed.angular.z = x[4]
		else:
			print(config.v_all_list[-1])
			print(config.w_all_list[-1])
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			pub.publish(speed)
			break
		vright.append((2*speed.linear.x+0.35*speed.angular.z)/2)
		vleft.append((2*speed.linear.x-0.35*speed.angular.z)/2)
		sl.append(state)

		if config.type == "dwa":
			config.v_all_list.append(speed.linear.x)#velocity
			config.av_all_list.append((config.v_all_list[-1]-config.v_all_list[-2])/config.dt)#acceleration
			config.w_all_list.append(speed.angular.z)#velocity
			config.aw_all_list.append((config.w_all_list[-1]-config.w_all_list[-2])/config.dt)#acceleration
			vjl.append((config.av_all_list[-1]-config.av_all_list[-2])/config.dt)
			wjl.append((config.aw_all_list[-1]-config.aw_all_list[-2])/config.dt)
		elif config.type == "a":
			config.v_all_list.append(speed.linear.x)#velocity
			config.av_all_list.append(u[2])#acceleration
			config.w_all_list.append(speed.angular.z)#velocity
			config.aw_all_list.append((config.w_all_list[-1]-config.w_all_list[-2])/config.dt)#acceleration
			#vjl.append((config.av_all_list[-1]-config.av_all_list[-2])/config.dt)
			#vjl.append(dwa_time)
			wjl.append((config.aw_all_list[-1]-config.aw_all_list[-2])/config.dt)
		elif config.type == "aa":
			config.v_all_list.append(speed.linear.x)#velocity
			config.av_all_list.append(u[2])#acceleration
			config.w_all_list.append(speed.angular.z)#velocity
			config.aw_all_list.append(u[3])#acceleration
			vjl.append((config.av_all_list[-1]-config.av_all_list[-2])/config.dt)
			wjl.append((config.aw_all_list[-1]-config.aw_all_list[-2])/config.dt)
		elif config.type == "j":
			config.v_all_list.append(speed.linear.x)#velocity
			config.av_all_list.append((config.v_all_list[-1]-config.v_all_list[-2])/config.dt)#acceleration
			config.w_all_list.append(speed.angular.z)#velocity
			config.aw_all_list.append((config.w_all_list[-1]-config.w_all_list[-2])/config.dt)#acceleration
			vjl.append(u[2])
			vjl_r.append((config.av_all_list[-1]-config.av_all_list[-2])/config.dt)
			wjl.append((config.aw_all_list[-1]-config.aw_all_list[-2])/config.dt)
		elif config.type == "jj":
			config.v_all_list.append(speed.linear.x)#velocity
			config.av_all_list.append((config.v_all_list[-1]-config.v_all_list[-2])/config.dt)#acceleration
			config.w_all_list.append(speed.angular.z)#velocity
			config.aw_all_list.append((config.w_all_list[-1]-config.w_all_list[-2])/config.dt)#acceleration
			vjl.append(u[2])
			wjl.append(u[3])
		line_list.append(25-line)
		odomx.append(config.x)
		odomy.append(config.y)
		if config.x <= -15:
			ap.append(config.x)
		elif -15 < config.x <= 15:
			bp.append(config.x)
		else:
			cp.append(config.x)
		odomth.append(config.th)
		pub.publish(speed)
		config.r.sleep()
	#print(sum(config.cal_time))
	profiler.stop()



	print("obs_nums=",len(obs.obst_nums))
	#print("cal_time=",len(config.cal_time))
	#del config.cal_time1[-1]
	#del config.cal_time2[-1]
	#x = list(np.arange(0,len(obs.obst_nums)-1))
	#fig = plt.figure()
	#plt.scatter(obs.obst_nums[:len(x)],config.cal_time1[:len(x)],color='blue',label='Np.No')
	#plt.scatter(obs.obst_nums[:len(x)],config.cal_time2[:len(x)],color='red',label='No')
	#plt.xlabel('Obst nums')
	#plt.ylabel('time [m]')
	#plt.legend()
	#plt.show()
	#fig = plt.figure()
	#plt.scatter(obs.obst_nums[:len(x)],config.cal_time3[:len(x)],color='black',label='Np+Np.No')
	#plt.scatter(obs.obst_nums[:len(x)],config.cal_time2[:len(x)],color='red',label='No')
	#plt.xlabel('Obst nums')
	#plt.ylabel('time [m]')
	#plt.legend()
	#plt.show()
	print(td)
	print(max(config.v_all_list))
	print(len(ap),len(bp),len(cp))
	#if 2.0 in config.v_all_list:
	#print(config.v_all_list.index(max(config.v_all_list)),t)
	#   print('avg velocity =',sum(config.v_all_list[config.v_all_list.index(2.0):td[0]])/len(config.v_all_list[config.v_all_list.index(2.0):td[0]]))
	print('maximum jerk =',max(vjl_r),min(vjl_r))
	print('avg speed =',np.mean(config.v_all_list))
	print('inrange obst nums =',np.mean(config.ob_nums))
	print(t,config.predict_time,config.laser_reso)
	m = 1
	#vaj_graph(config,line_list,[0]*len(line_list),[0]*len(line_list),"line")

	#vaj_graph(config,config.v_all_list[:-m],config.av_all_list[:-m],vjl[:-m],"linear")
	#vaj_graph(config,config.v_all_list[:-m],config.av_all_list[:-m],vjl_r[:-m],"linear")
	#vaj_graph(config,config.w_all_list[:-m],config.aw_all_list[:-m],wjl[:-m],"angular")
	#vaj_graph(config,config.v_all_list[:-m],vleft[:-m],vright[:-m],"wheel")
	#time_graph(config,obs.obst_nums[:len(x)],config.cal_time[:len(x)])
	#path_jerk_graph(config,odomx[:-m],odomy[:-m],vjl[:-m])
	#path_jerk_graph(config,odomx[:-m],odomy[:-m],vjl_r[:-m])
	#fig = plt.figure()
	#x = list(np.arange(0,len(config.cal_time2)))
	#plt.scatter(x,config.cal_time2)
	#plt.show()
	time_day = time.strftime("%Y%m%d",time.localtime())
	path = '../results/{}/'.format(time_day)
	f = open(path+config.type+'*10.txt',"a")
	#f = open(path+'env2_'+config.type+'_original_eff_0.02.txt',"a")
	#f.write(str(config.type)+','+str(config.j_weight)+','+'{}'.format(time.strftime("%Y%m%d-%H%M", time.localtime())))
	f.write('\n')
	f.write(str(config.dt)+','+'360')
	f.write('\n')
	f.write(str(len(config.cal_time1))+',')
	f.write(str(np.mean(config.cal_time1))+',')
	f.write(str(np.std(config.cal_time1))+',')
	f.write(str(np.mean(config.cal_time2[0:int(len(config.cal_time2)*0.5)]))+',')
	f.write(str(np.std(config.cal_time2[0:int(len(config.cal_time2)*0.5)]))+',')
	f.write('\n')
	#profiler.print(file=f)
	f.close()
	#profiler.output_text(unicode=True,color=True)
	profiler.print()

def cal_dist(config,v0,w0,av,aw,obst):
	mindist = 10000
	x = sympy.Symbol('x')
	t = sympy.Symbol('t')
	#v = v0+av*t
	#w = w0+aw*t
	v = v0 + av*t
	w = w0 + aw*t

	theta = w0*t+0.5*aw*t**2

	f_ = (v*sympy.cos(theta))
	g_ = (v*sympy.sin(theta))

	f = sympy.integrate(f_,t)
	g = sympy.integrate(g_,t)

	f = sympy.integrate(f_,t)
	g = sympy.integrate(g_,t)+(v0/w0)
	#print(f_)
	#print(g_)
	#print(f)
	#print(g)

	if w0 != 0 and v0 != 0:
		#f = sympy.integrate(f_,t)-(v0/w0)
		#g = sympy.integrate(g_,t)+(v0/w0)


		#print(g)
		for i in obst.copy():
			#print(v,w)
			h = f_*(i[0]-(f))+g_*(i[1]-(g))
			#print(h)
			t0 = sympy.nsolve(h,t,0)
			#t0 = sympy.solveset(h,t,sympy.Interval(0,3.14))
			#t0 = list(t0.args[0])
			#print(t0)
			#t0 = t0 % math.pi
			#t0 = sympy.nsolve(h,t,0,verify=False)
			#print(sympy.nsolve(h,t,0,verify=False))

			if t0 >= 0 and t0 <= config.predict_time:
				xfoot = f.evalf(subs={t:t0})
				yfoot = g.evalf(subs={t:t0})
				dist = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
				#print(f,g)
				#print(t0)
				#print(math.sin(t0),1-math.cos(t0))
				#print(xfoot,yfoot)
				#print(dist)
				#print('*************')
				if dist <= mindist:
					mindist = dist
			else:
				startdist = math.sqrt((i[0]-0)**2+(i[1]-0)**2)
				enddist = math.sqrt((i[0]-f.evalf(subs={t:config.predict_time}))**2+(i[1]-g.evalf(subs={t:config.predict_time}))**2)
				dist = min(startdist,enddist)
				if dist <= mindist:
					mindist = dist

		if mindist <= config.robot_radius:
			cost = -10000  # collision
		elif config.robot_radius< mindist <= 1.2:
			cost = mindist - 0.2
		else:
			cost = 1
		return cost,mindist,f.evalf(subs={t:config.predict_time}),g.evalf(subs={t:config.predict_time})
	elif w0 == 0 and v0 != 0:
		g = 0
		for i in obst.copy():
			h = f_*(i[0]-(f))+g_*(i[1]-(g))
			t0 = sympy.nsolve(h,t,0,verify=False)
			#print(t0)
			if t0 >= 0 and t0 <= config.predict_time:
				xfoot = f.evalf(subs={t:t0})
				yfoot = 0
				dist = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
				if dist <= mindist:
					mindist = dist
			else:
				startdist = math.sqrt((i[0]-0)**2+(i[1]-0)**2)
				enddist = math.sqrt((i[0]-f.evalf(subs={t:config.predict_time}))**2+(i[1]-0)**2)
				dist = min(startdist,enddist)
				if dist <= mindist:
					mindist = dist
		if mindist <= config.robot_radius:
			cost = -10000  # collision
		elif config.robot_radius< mindist <= 1.2:
			cost = mindist - 0.2
		else:
			cost = 1
		return cost,mindist,f.evalf(subs={t:config.predict_time}),0

	else:
	#elif w0 != 0 and v0 == 0:
		for i in obst.copy():
			dist = math.sqrt((i[0]-0)**2+(i[1]-0)**2)
			if dist <= mindist:
				mindist = dist

		if mindist <= config.robot_radius:
			cost = -10000  # collision
		elif config.robot_radius< mindist <= 1.2:
			cost = mindist - 0.2
		else:
			cost = 1
		return cost,mindist,0,0

	#h = f_*(x0-(f))+g_*(y0-(g))


	"""
	xl=[]
	yl=[]
	for t0 in range(0,8):
		xl.append(f.evalf(subs={t:t0*0.1}))
		yl.append(g.evalf(subs={t:t0*0.1}))
	plt.plot(xl,yl)
	plt.show()
	"""
	#print('newmethod',mindist

def compare_dist():
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14
	xinit = [0,0,0,0,0]
	config = Config()
	obst=list()
	config.predict_time = 1.0
	dtlist=[]
	distlist1=[]
	distlist21=[]
	distlist22=[]
	distlist23=[]
	distlist24=[]
	distlist3=[]
	distlist4=[]

	d3max = []
	d3min = []
	v0=1.0
	av=2.0
	w0=1.0
	aw=0
	vn = v0+av*config.predict_time
	#if acct <= config.predict_time:
	if vn >= config.max_speed:
		vn = config.max_speed
	elif vn <= 0:
		vn = 0
	else:
		vn = v0+av*config.predict_time

	for i in np.arange(0.2,3.0,0.1):
		#x = (1.5)*math.sin(i)
		#y = (1.5)*(1-math.cos(i))-(0.5)
		x = (1.5)*math.sin(i)
		y = (1.5)*(1-math.cos(i))-1.5
		#y = 1.85-y
		#x = 1.5
		#y = -1.5 + i*3
		obst.append((x,y))

	x = sympy.Symbol('x')
	t = sympy.Symbol('t')
	if av > 0:
		acct1 = (config.max_speed-v0)/av
		v = sympy.Piecewise((v0 + av*t,t<=acct1),(config.max_speed,t>acct1))
	elif av < 0:
		acct1 = abs(v0/av)
		v = sympy.Piecewise((v0 + av*t,t<=acct1),(0,t>acct1))
	else:
		v = v0
	#w = w0 + aw*t
	theta = w0*t
	f_ = (v*sympy.cos(theta))
	g_ = (v*sympy.sin(theta))
	f = (av*sympy.cos(w0*t)+w0*(v0+av*t)*sympy.sin(w0*t))/(w0**2)
	g = (av*sympy.sin(w0*t)-w0*(v0+av*t)*sympy.cos(w0*t))/(w0**2)
	dx = f.evalf(subs={t:0})-0
	dy = g.evalf(subs={t:0})-0
	f1 = f-dx
	g1 = g-dy
	f2 = config.max_speed*sympy.sin(w0*t)/w0
	g2 = -config.max_speed*sympy.cos(w0*t)/w0
	f3 = 0
	g3 = 0
	if av > 0:
		acct1 = (config.max_speed-v0)/av
		x1 = f1.evalf(subs={t:acct1})
		y1 = g1.evalf(subs={t:acct1})
		x2 = f2.evalf(subs={t:acct1})
		y2 = g2.evalf(subs={t:acct1})
		dx1 = x2-x1
		dy1 = y2-y1
		f = sympy.Piecewise((f1,t<=acct1)
							,(f2-dx1,t>acct1))
		g = sympy.Piecewise((g1,t<=acct1)
							,(g2-dy1,t>acct1))
	elif av < 0:
		acct1 = abs(v0/av)
		x1 = f1.evalf(subs={t:acct1})
		y1 = g1.evalf(subs={t:acct1})
		x3 = 0
		y3 = 0
		dx2 = x3-x1
		dy2 = y3-y1
		f = sympy.Piecewise((f1,t<=acct1)
							,(f3-dx2,t>acct1))
		g = sympy.Piecewise((g1,t<=acct1)
							,(g3-dy2,t>acct1))
	else:
		f = v0*sympy.sin(w0*t)/w0-dx
		g = -v0*sympy.cos(w0*t)/w0-dy
	#print(f)
	#print(g)
	def ff(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_speed-v0)/av
				x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
				dx1 = x0 -0
				if 0 <=t<=acct:
					f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
				elif acct <t<=config.predict_time:
					x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
					x2 = config.max_speed*math.sin(w0*acct)/w0
					dx2 = x2-x1
					f = config.max_speed*math.sin(w0*t)/w0-dx2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0/av)
				x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
				dx1 = x0 -0
				if 0 <=t<=acct:
					f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
				elif acct <t<=config.predict_time:
					x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
					x2 = 0
					dx2 = x2-x1
					f = 0-dx2
			else:
				x0 = v0*math.sin(w0*0)/w0
				dx1 = x0 - 0
				f = v0*math.sin(w0*t)/w0-dx1
		else:
			if av > 0:
				acct = (config.max_speed-v0)/av
				if 0 <=t<=acct:
					f = v0*t+0.5*av*t**2
				elif acct <t<=config.predict_time:
					f = (v0*acct+0.5*av*acct**2)+(config.predict_time-acct)+config.max_speed
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0/av)
				if 0 <=t<=acct:
					f = v0*t+0.5*av*t**2
				elif acct <t<=config.predict_time:
					f = v0*acct+0.5*av*acct**2
				else:
					print('out of range')
			else:
				f = v0*t
		return f
	def gg(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_speed-v0)/av
				y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
				dy1 = y0 -0
				if 0 <=t<=acct:
					g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
				elif acct <t<=config.predict_time:
					y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
					y2 = -config.max_speed*math.cos(w0*acct)/w0
					dy2 = y2-y1
					g = -config.max_speed*math.cos(w0*t)/w0-dy2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0/av)
				y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
				dy1 = y0 -0
				if 0 <=t<=acct:
					g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
				elif acct <t<=config.predict_time:
					y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
					y2 = 0
					dy2 = y2-y1
					g = 0-dy2
			else:
				y0 = -v0*sympy.cos(w0*0)/w0
				dy1 = y0 - 0
				g = -v0*sympy.cos(w0*t)/w0-dy1
		else:
			g = 0
		return g
	traj5x = []
	traj5y = []
	config.dt = config.predict_time/10
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj1 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/20
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj2 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/50
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj3 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/100
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj4 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/100
	for i in np.arange(0,config.predict_time,config.dt):
		#traj5x.append(f.evalf(subs={t:i}))
		#traj5y.append(g.evalf(subs={t:i}))
		traj5x.append(ff(i))
		traj5y.append(gg(i))
	fig = plt.figure()
	#fig = plt.figure(figsize=(8,3))
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")
	plt.plot(traj1[:,0],traj1[:,1],c='blue',label='predict path time inverse 0.1 [s]')
	plt.plot(traj2[:,0],traj2[:,1],label='0.05')
	plt.plot(traj3[:,0],traj3[:,1],label='0.02')
	#plt.plot(traj4[:,0],traj4[:,1],c='red',label='predict path time inverse 0.01[s]')
	plt.plot(traj5x,traj5y,c='black',label='real path')
	plt.xlabel('x [m]')
	plt.ylabel('y [m]')
	#plt.xlim([None,1.0])
	#plt.ylim([None,1.2])
	plt.legend()
	plt.show()

	traj1x=[]
	traj1y=[]
	traj2x=[]
	traj2y=[]
	traj3x=[]
	traj3y=[]
	traj8x=[]
	traj8y=[]
	#print(f)
	#print(g)
	#if w0 != 0:
	#for i in np.arange(0,config.predict_time,config.dt):
	for i in np.arange(0,config.predict_time,config.dt):
		#traj1x.append(f.evalf(subs={t:i}))
		#traj1y.append(g.evalf(subs={t:i}))
		traj1x.append(ff(i))
		traj1y.append(gg(i))
		if w0 != 0:
			traj2x.append((v0/w0)*math.sin(w0*i))
			traj3x.append((vn/w0)*math.sin(w0*i))
			traj2y.append((v0/w0)*(1-math.cos(w0*i)))
			traj3y.append((vn/w0)*(1-math.cos(w0*i)))
		else:
			traj2x.append(v0*i)
			traj3x.append(vn*i)
			traj2y.append(0)
			traj3y.append(0)
	#av = 0.5
	#for a in np.arange(0,config.predict_time,config.dt):
	#   traj8x.append(ff(a))
	#   traj8y.append(gg(a))
	#av = 1.0
	print("traj1x=",len(traj1x))
	if w0 != 0:
		k = g_.evalf(subs={t:config.predict_time})/f_.evalf(subs={t:config.predict_time})
		k = -1/k
	else:
		k = 0
	print(k)
	#y = k*x+b
	b = traj1y[-1]-k*traj1x[-1]
	print(b)
	anglemax = math.atan2(traj1y[-1]-b,traj1x[-1]-0)
	print(anglemax)
	print(math.degrees(anglemax))
	dx21 = traj2x[-1]-traj1x[-1]
	dy21 = traj2y[-1]-traj1y[-1]
	dx31 = traj3x[-1]-traj1x[-1]
	dy31 = traj3y[-1]-traj1y[-1]
	if av != 0:
		n = len(traj1x)*(math.sqrt((v0**2+vn**2)*0.5)-v0)/av
	else:
		n = 0.5*config.predict_time
	print("n=",n)
	print("len n=",len(traj2x))
	kn = g_.evalf(subs={t:(n/len(traj1x))})/f_.evalf(subs={t:(n/len(traj1x))})
	print('kn=',math.degrees(kn))
	nn = kn*config.dt*len(traj1x)/w0
	print('kn/w0=',len(traj1x)*kn/w0)
	dx41 = traj2x[int(n)]-traj1x[int(n)]
	dy41 = traj2y[int(n)]-traj1y[int(n)]
	dx51 = traj3x[int(n)]-traj1x[int(n)]
	dy51 = traj3y[int(n)]-traj1y[int(n)]
	traj4x = np.array(traj2x)-dx21
	traj4y = np.array(traj2y)-dy21

	traj5x = np.array(traj3x)-dx31
	traj5y = np.array(traj3y)-dy31

	traj6x = np.array(traj2x)-dx41
	traj6y = np.array(traj2y)-dy41

	traj7x = np.array(traj3x)-dx51
	traj7y = np.array(traj3y)-dy51

	#fig = plt.figure()
	fig = plt.figure(figsize=(5.25,5.25))
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")


	plt.plot(traj2x,traj2y,label='Inside circle',c='blue',linestyle='solid')
	plt.plot(traj4x,traj4y,c='blue',linestyle='solid')
	plt.plot(traj6x,traj6y,c='blue',linestyle='solid')
	plt.plot(traj3x,traj3y,label='Outside circle',c='red',linestyle='solid')
	plt.plot(traj5x,traj5y,c='red',linestyle='solid')
	plt.plot(traj7x,traj7y,c='red',linestyle='solid')
	#plt.plot(traj8x,traj8y,label='a = 0.5'+'m/$ \mathit{s}^{2}$',c='black',linestyle='dotted')
	plt.plot(traj1x,traj1y,label='Path',c='black',linestyle='solid')
	#plt.plot((0,traj1x[-1]),(b,traj1y[-1]))
	#plt.plot((0,0),(b,0))
	if w0 != 0:
		c2 = [0,v0/w0]
		c3 = [0,vn/w0]
		c4 = [0-dx21,v0/w0-dy21]
		c5 = [0-dx31,vn/w0-dy31]
		c6 = [0-dx41,v0/w0-dy41]
		c7 = [0-dx51,vn/w0-dy51]
		obx = []
		oby = []
		r2l = []
		r3l = []
		r4l = []
		r5l = []
		o21_error=[]
		o22_error=[]
		o23_error=[]
		o24_error=[]
		p_error=[]
		distl = []
		tl = []

		for i in obst:
			f_ = sympy.diff(f,t)
			g_ = sympy.diff(g,t)
			h = f_*(i[0]-(f))+g_*(i[1]-(g))
			t0 = sympy.nsolve(h,t,[0,1])
			time_s = time.time()
			if 0<=t0<=1:
				xfoot = f.evalf(subs={t:t0})
				yfoot = g.evalf(subs={t:t0})
				dist = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
			else:
				xfoot = f.evalf(subs={t:config.predict_time})
				yfoot = g.evalf(subs={t:config.predict_time})
				dist1 = math.sqrt((i[0])**2+(i[1])**2)
				dist2 = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
				dist = min(dist1,dist2)
			time_e = time.time()
			tl.append(time_e-time_s)
			#print(time_e-time_s)
			distl.append(dist)
		print(np.mean(tl))
		print("****************")

		for i in obst:
			xend = ff(config.predict_time)
			yend = gg(config.predict_time)
			minr1 = 1000
			minr2 = 1000
			minr3 = 1000
			minr4 = 1000
			print('===========')
			print('obstacle',obst.index(i),i[0],i[1])
			obx.append(i[0])
			oby.append(i[1])
			r2 = math.sqrt((i[0]-c2[0])**2+(i[1]-c2[1])**2)
			r3 = math.sqrt((i[0]-c3[0])**2+(i[1]-c3[1])**2)
			r4 = math.sqrt((i[0]-c4[0])**2+(i[1]-c4[1])**2)
			r5 = math.sqrt((i[0]-c5[0])**2+(i[1]-c5[1])**2)
			r6 = math.sqrt((i[0]-c6[0])**2+(i[1]-c6[1])**2)
			r7 = math.sqrt((i[0]-c7[0])**2+(i[1]-c7[1])**2)
			#r2 = r4
			#r3 = r5
			#r4 = r6
			#r5 = r7
			print(r2,r3,r4,r5,r6,r7)
			#print(r2-(v0/w0))
			#print(abs(r2-(v0/w0)))
			print(abs(r2-(v0/w0)),abs(r3-(vn/w0)),abs(r4-(v0/w0)),abs(r5-(vn/w0)),abs(r6-(v0/w0)),abs(r7-(vn/w0)))
			angle = math.atan2(i[1]-b,i[0]-0)
			print(math.degrees(angle))
			#avg_r1 = (abs(r2-(v0/w0))+abs(r3-(vn/w0)))*0.5
			#avg_r2 = (abs(r4-(v0/w0))+abs(r5-(vn/w0)))*0.5
			#avg_r3 = (abs(r6-(v0/w0))+abs(r7-(vn/w0)))*0.5
			print(max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))
			print(min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0)))))
			angle = math.atan2(i[1]-b,i[0])
			if (w0 > 0 and -0.5*math.pi < angle < anglemax) or (w0 < 0 and anglemax < angle < 0.5*math.pi) :
				if r3-(vn/abs(w0))>0 and r5-(vn/abs(w0))>0 and r7-(vn/abs(w0))>0 :
					print('outside')
					if av > 0:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
					else:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				elif r2-(v0/abs(w0))<0 and r4-(v0/abs(w0))<0 and r6-(v0/abs(w0))<0:
				#else:
					print('inside')
					if av > 0:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

					else:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				else:
					pdist = 0.5*(max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+0.5*(min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0)))))
			else:
				dist1 = math.sqrt((i[0]-xend)**2+(i[1]-yend)**2)
				dist2 = math.sqrt((i[0])**2+(i[1])**2)
				pdist = min(dist1,dist2)

			#end = v0*config.predict_time+0.5*av*config.predict_time**2
			#end = 0
			#dist1 = math.sqrt((i[0]-xend)**2+(i[1]-yend)**2)
			#dist2 = math.sqrt((i[0])**2+(i[1])**2)
			#pdist = min(dist1,dist2)

			for ii in range(0, traj1.shape[0]):
				dx = traj1[ii, 0] - i[0]
				dy = traj1[ii, 1] - i[1]
				r1 = math.sqrt(dx**2 + dy**2)
				if r1 <= minr1:
					minr1 = r1
			for ii in range(0, traj2.shape[0]):
				dx = traj2[ii, 0] - i[0]
				dy = traj2[ii, 1] - i[1]
				r2 = math.sqrt(dx**2 + dy**2)
				if r2 <= minr2:
					minr2 = r2
			for ii in range(0, traj3.shape[0]):
				dx = traj3[ii, 0] - i[0]
				dy = traj3[ii, 1] - i[1]
				r3 = math.sqrt(dx**2 + dy**2)
				if r3 <= minr3:
					minr3 = r3
			for ii in range(0, traj4.shape[0]):
				dx = traj4[ii, 0] - i[0]
				dy = traj4[ii, 1] - i[1]
				r4 = math.sqrt(dx**2 + dy**2)
				#print(r4)
				if r4 <= minr4:
					minr4 = r4

			#f_ = sympy.diff(f,t)
			#g_ = sympy.diff(g,t)
			#h = f_*(i[0]-(f))+g_*(i[1]-(g))
			#t0 = sympy.nsolve(h,t,[0,1])

			#if 0<=t0<=1:
			#   xfoot = f.evalf(subs={t:t0})
			#   yfoot = g.evalf(subs={t:t0})
			#   dist = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
			#else:
			#   xfoot = f.evalf(subs={t:config.predict_time})
			#   yfoot = g.evalf(subs={t:config.predict_time})
			#   dist1 = math.sqrt((i[0])**2+(i[1])**2)
			#   dist2 = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
			#   dist = min(dist1,dist2)
			#print(dist)


			distlist1.append(distl[obst.index(i)])
			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)
			distlist24.append(minr4)
			distlist3.append(pdist)
			#distlist4.append(pdist2)
			d3min.append(max(r3-(vn/w0),r5-(vn/w0)))
			d3max.append(min(r2-(v0/w0),r4-(v0/w0)))
			print('true dist =',dist)
			print('pdist = ',pdist)
			print('odist = ',minr1)
			error21 = (minr1-dist)
			error22 = (minr2-dist)
			error23 = (minr3-dist)
			error24 = (minr4-dist)
			error3 = (pdist-dist)
			o21_error.append(abs(error21))
			o22_error.append(abs(error22))
			o23_error.append(abs(error23))
			o24_error.append(abs(error24))
			p_error.append(abs(error3))
	else:
		obx = []
		oby = []
		xend = ff(config.predict_time)
		for i in obst:
			print('===========')
			print('obstacle',i[0],i[1])
			obx.append(i[0])
			oby.append(i[1])
			minr1 = 1000
			minr2 = 1000
			minr3 = 1000
			minr4 = 1000
			if 0 <= i[0]<= xend:
				pdist = i[1]
			else:
				dist1 = math.sqrt((i[0]-xend)**2+(i[1]-0)**2)
				dist2 = math.sqrt((i[0])**2+(i[1])**2)
				pdist = min(dist1,dist2)
			for ii in range(0, traj1.shape[0]):
				dx = traj1[ii, 0] - i[0]
				dy = traj1[ii, 1] - i[1]
				r1 = math.sqrt(dx**2 + dy**2)
				if r1 <= minr1:
					minr1 = r1
			for ii in range(0, traj2.shape[0]):
				dx = traj2[ii, 0] - i[0]
				dy = traj2[ii, 1] - i[1]
				r2 = math.sqrt(dx**2 + dy**2)
				if r2 <= minr2:
					minr2 = r2
			for ii in range(0, traj3.shape[0]):
				dx = traj3[ii, 0] - i[0]
				dy = traj3[ii, 1] - i[1]
				r3 = math.sqrt(dx**2 + dy**2)
				if r3 <= minr3:
					minr3 = r3

			for ii in range(0, traj4.shape[0]):
				dx = traj4[ii, 0] - i[0]
				dy = traj4[ii, 1] - i[1]
				r4 = math.sqrt(dx**2 + dy**2)
				#print(r4)
				if r4 <= minr4:
					minr4 = r4
			#h = f_*(list(obst)[0][0]-(f))+g_*(list(obst)[0][1]-(g))

			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)
			distlist24.append(minr4)
			distlist3.append(pdist)
			#distlist4.append(pdist2)
			#d3min.append(max(r3-(vn/w0),r5-(vn/w0)))
			#d3max.append(min(r2-(v0/w0),r4-(v0/w0)))
			#print('true dist =',dist)
			print('pdist = ',pdist)
			print('odist = ',minr1)
			#error21 = (minr1-dist)
			#error22 = (minr2-dist)
			#error23 = (minr3-dist)
			#error24 = (minr4-dist)
			#error3 = (pdist-dist)
			#o21_error.append(abs(error21))
			#o22_error.append(abs(error22))
			#o23_error.append(abs(error23))
			#o24_error.append(abs(error24))
			#p_error.append(abs(error3))

	print("********************************")

	#print('original max error = ',max(o_error))
	#print('original min error = ',min(o_error))
	#print('original avg error = ',sum(o_error)/len(o_error))
	#print('proposed max error = ',max(p_error))
	#print('proposed min error = ',min(p_error))
	#print('proposed avg error = ',sum(p_error)/len(p_error))
	#plt.scatter(obx,oby,c='black')
	plt.xlabel("x [m]")
	plt.ylabel("y [m]")
	plt.xlim([min(traj5x)-0.1,max(traj3x)+0.1])
	plt.ylim([min(traj5y)-0.1,max(traj3y)+0.1])

	plt.legend()
	plt.tight_layout()
	plt.show()

	plt.subplots(tight_layout=True,figsize=(7,4),dpi=100)
	mse21 = 0
	mse22 = 0
	mse23 = 0
	mse24 = 0
	mse3 = 0
	for i in range(len(distlist1)):
		mse21=mse21+(distlist21[i]-distlist1[i])**2
		mse22=mse22+(distlist22[i]-distlist1[i])**2
		mse23=mse23+(distlist23[i]-distlist1[i])**2
		mse24=mse24+(distlist24[i]-distlist1[i])**2
		mse3=mse3+(distlist3[i]-distlist1[i])**2
	mse21 = math.sqrt(mse21/len(distlist21))
	mse22 = math.sqrt(mse22/len(distlist22))
	mse23 = math.sqrt(mse23/len(distlist23))
	mse24 = math.sqrt(mse24/len(distlist24))
	mse3 = math.sqrt(mse3/len(distlist3))
	#print(sum(distlist21)/len(distlist21),
	#   sum(distlist22)/len(distlist22),
	#   sum(distlist23)/len(distlist23),
	#   sum(distlist24)/len(distlist24),
	#   sum(distlist3)/len(distlist3))
	#print(mse21,mse22,mse23,mse24,mse3)
	#print(max(o21_error),max(o22_error),max(o23_error),max(o24_error),max(p_error))
	#print(min(o21_error),min(o22_error),min(o23_error),min(o24_error),min(p_error))
	print('rmse')
	for i in [mse21,mse22,mse23,mse24,mse3]:
		print(format(i,'.5f'))
	print('max_error')
	for i in [max(o21_error),max(o22_error),max(o23_error),max(o24_error),max(p_error)]:
		print(format(i,'.5f'))
	print('min_error')
	for i in [min(o21_error),min(o22_error),min(o23_error),min(o24_error),min(p_error)]:
		print(format(i,'.5f'))
	plt.scatter(np.arange(0,len(distlist1)),distlist1,label='Baseline',c='black',marker='o')
	plt.plot(np.arange(0,len(distlist1)),distlist1,c='black')
	plt.scatter(np.arange(0,len(distlist21)),distlist21,label='Original Δt=100ms',c='blue',marker='v')
	plt.plot(np.arange(0,len(distlist21)),distlist21,c='blue')
	plt.scatter(np.arange(0,len(distlist22)),distlist22,label='Original Δt=50 ms')
	plt.plot(np.arange(0,len(distlist22)),distlist22)
	plt.scatter(np.arange(0,len(distlist23)),distlist23,label='Original Δt=20 ms')
	plt.plot(np.arange(0,len(distlist23)),distlist23)
	#plt.scatter(np.arange(0,len(distlist24)),distlist24,label='Original 0.01',marker=',')
	#plt.plot(np.arange(0,len(distlist24)),distlist24)
	plt.scatter(np.arange(0,len(distlist3)),distlist3,label='Proposed',c='red',marker='^')
	plt.plot(np.arange(0,len(distlist3)),distlist3,c='red')
	#plt.scatter(np.arange(0,len(distlist4)),distlist4,label='proposed_avg')
	#plt.plot(np.arange(0,len(distlist4)),distlist4)
	#plt.scatter(np.arange(0,len(d3max)),d3max,label='proposed+')
	#plt.plot(np.arange(0,len(d3max)),d3max)
	#plt.scatter(np.arange(0,len(d3min)),d3min,label='proposed-')
	#plt.plot(np.arange(0,len(d3min)),d3min)
	plt.legend()
	#plt.ylim(0.445,0.505)
	x_major_locator=MultipleLocator(5)
	#y_major_locator=MultipleLocator(0.005)
	ax=plt.gca()
	ax.xaxis.set_major_locator(x_major_locator)
	#ax.yaxis.set_major_locator(y_major_locator)
	plt.xlim(-0.5,27.5)
	plt.ylim(0.0,1.6)
	plt.xlabel("Obstacle Number")
	plt.ylabel("Distance [m]")
	plt.show()
	"""
	plt.scatter(np.arange(0,10),distlist1,label='baseline',c='black')
	plt.scatter(np.arange(0,10),d3max,label='d3max',c='blue')
	plt.scatter(np.arange(0,10),d3min,label='d3min',c='red')
	plt.legend()
	plt.ylim(0.48,0.51)
	x_major_locator=MultipleLocator(1)
	ax=plt.gca()
	ax.xaxis.set_major_locator(x_major_locator)
	plt.xlabel("Obstacle Number")
	plt.ylabel("Distance [m]")
	plt.show()
	"""

def compare_dist2():
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14
	xinit = [0,0,0,0,0]
	config = Config()
	obst=list()
	config.predict_time = 1.0
	dtlist=[]
	distlist1=[]
	distlist21=[]
	distlist22=[]
	distlist23=[]
	distlist24=[]
	distlist3=[]
	distlist4=[]

	d3max = []
	d3min = []
	v0=1.0
	av=1.0
	w0=1.0
	aw=0
	vn = v0+av*config.predict_time
	#if acct <= config.predict_time:
	if vn >= config.max_speed:
		vn = config.max_speed
	elif vn <= 0:
		vn = 0
	else:
		vn = v0+av*config.predict_time

	for i in np.arange(0.2,3.0,0.1):
		#x = (1.5)*math.sin(i)
		#y = (1.5)*(1-math.cos(i))-(0.5)
		x = (1.5)*math.sin(i)
		y = (1.5)*(1-math.cos(i))-1.5
		#y = 1.85-y
		#x = 1.5
		#y = -1.5 + i*3
		obst.append((x,y))

	x = sympy.Symbol('x')
	t = sympy.Symbol('t')
	if av > 0:
		acct1 = (config.max_speed-v0)/av
		v = sympy.Piecewise((v0 + av*t,t<=acct1),(config.max_speed,t>acct1))
	elif av < 0:
		acct1 = abs(v0/av)
		v = sympy.Piecewise((v0 + av*t,t<=acct1),(0,t>acct1))
	else:
		v = v0
	#w = w0 + aw*t
	theta = w0*t
	f_ = (v*sympy.cos(theta))
	g_ = (v*sympy.sin(theta))
	f = (av*sympy.cos(w0*t)+w0*(v0+av*t)*sympy.sin(w0*t))/(w0**2)
	g = (av*sympy.sin(w0*t)-w0*(v0+av*t)*sympy.cos(w0*t))/(w0**2)
	dx = f.evalf(subs={t:0})-0
	dy = g.evalf(subs={t:0})-0
	f1 = f-dx
	g1 = g-dy
	f2 = config.max_speed*sympy.sin(w0*t)/w0
	g2 = -config.max_speed*sympy.cos(w0*t)/w0
	f3 = 0
	g3 = 0
	if av > 0:
		acct1 = (config.max_speed-v0)/av
		x1 = f1.evalf(subs={t:acct1})
		y1 = g1.evalf(subs={t:acct1})
		x2 = f2.evalf(subs={t:acct1})
		y2 = g2.evalf(subs={t:acct1})
		dx1 = x2-x1
		dy1 = y2-y1
		f = sympy.Piecewise((f1,t<=acct1)
							,(f2-dx1,t>acct1))
		g = sympy.Piecewise((g1,t<=acct1)
							,(g2-dy1,t>acct1))
	elif av < 0:
		acct1 = abs(v0/av)
		x1 = f1.evalf(subs={t:acct1})
		y1 = g1.evalf(subs={t:acct1})
		x3 = 0
		y3 = 0
		dx2 = x3-x1
		dy2 = y3-y1
		f = sympy.Piecewise((f1,t<=acct1)
							,(f3-dx2,t>acct1))
		g = sympy.Piecewise((g1,t<=acct1)
							,(g3-dy2,t>acct1))
	else:
		f = v0*sympy.sin(w0*t)/w0-dx
		g = -v0*sympy.cos(w0*t)/w0-dy
	#print(f)
	#print(g)
	def ff(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_speed-v0)/av
				x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
				dx1 = x0 -0
				if 0 <=t<=acct:
					f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
				elif acct <t<=config.predict_time:
					x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
					x2 = config.max_speed*math.sin(w0*acct)/w0
					dx2 = x2-x1
					f = config.max_speed*math.sin(w0*t)/w0-dx2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0/av)
				x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
				dx1 = x0 -0
				if 0 <=t<=acct:
					f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
				elif acct <t<=config.predict_time:
					x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
					x2 = 0
					dx2 = x2-x1
					f = 0-dx2
			else:
				x0 = v0*math.sin(w0*0)/w0
				dx1 = x0 - 0
				f = v0*math.sin(w0*t)/w0-dx1
		else:
			if av > 0:
				acct = (config.max_speed-v0)/av
				if 0 <=t<=acct:
					f = v0*t+0.5*av*t**2
				elif acct <t<=config.predict_time:
					f = (v0*acct+0.5*av*acct**2)+(config.predict_time-acct)+config.max_speed
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0/av)
				if 0 <=t<=acct:
					f = v0*t+0.5*av*t**2
				elif acct <t<=config.predict_time:
					f = v0*acct+0.5*av*acct**2
				else:
					print('out of range')
			else:
				f = v0*t
		return f
	def gg(t):
		if w0 != 0:
			if av > 0:
				acct = (config.max_speed-v0)/av
				y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
				dy1 = y0 -0
				if 0 <=t<=acct:
					g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
				elif acct <t<=config.predict_time:
					y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
					y2 = -config.max_speed*math.cos(w0*acct)/w0
					dy2 = y2-y1
					g = -config.max_speed*math.cos(w0*t)/w0-dy2
				else:
					print('out of range')
			elif av < 0:
				acct = abs(v0/av)
				y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
				dy1 = y0 -0
				if 0 <=t<=acct:
					g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
				elif acct <t<=config.predict_time:
					y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
					y2 = 0
					dy2 = y2-y1
					g = 0-dy2
			else:
				y0 = -v0*sympy.cos(w0*0)/w0
				dy1 = y0 - 0
				g = -v0*sympy.cos(w0*t)/w0-dy1
		else:
			g = 0
		return g
	traj5x = []
	traj5y = []
	config.dt = config.predict_time/10
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj1 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/20
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj2 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/50
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj3 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/100
	v_predict_list=[v0]
	for i in np.arange(0,config.predict_time,config.dt):
		vnext = v_predict_list[-1]+av*config.dt
		if vnext >= config.max_speed:
			vnext = config.max_speed
		elif vnext <= 0:
			vnext = 0
		v_predict_list.append(vnext)
	traj4 = calc_trajectory(xinit, v_predict_list, w0, config)

	config.dt = config.predict_time/100
	for i in np.arange(0,config.predict_time,config.dt):
		#traj5x.append(f.evalf(subs={t:i}))
		#traj5y.append(g.evalf(subs={t:i}))
		traj5x.append(ff(i))
		traj5y.append(gg(i))
	fig = plt.figure()
	#fig = plt.figure(figsize=(8,3))
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")
	plt.plot(traj1[:,0],traj1[:,1],c='blue',label='predict path time inverse 0.1 [s]')
	plt.plot(traj2[:,0],traj2[:,1],label='0.05')
	plt.plot(traj3[:,0],traj3[:,1],label='0.02')
	plt.plot(traj4[:,0],traj4[:,1],c='red',label='predict path time inverse 0.01[s]')
	plt.plot(traj5x,traj5y,c='black',label='real path')
	plt.xlabel('x [m]')
	plt.ylabel('y [m]')
	#plt.xlim([None,1.0])
	#plt.ylim([None,1.2])
	plt.legend()
	plt.show()

	traj1x=[]
	traj1y=[]
	traj2x=[]
	traj2y=[]
	traj3x=[]
	traj3y=[]
	traj8x=[]
	traj8y=[]
	#print(f)
	#print(g)
	#if w0 != 0:
	#for i in np.arange(0,config.predict_time,config.dt):
	for i in np.arange(0,config.predict_time,config.dt):
		#traj1x.append(f.evalf(subs={t:i}))
		#traj1y.append(g.evalf(subs={t:i}))
		traj1x.append(ff(i))
		traj1y.append(gg(i))
		if w0 != 0:
			traj2x.append((v0/w0)*math.sin(w0*i))
			traj3x.append((vn/w0)*math.sin(w0*i))
			traj2y.append((v0/w0)*(1-math.cos(w0*i)))
			traj3y.append((vn/w0)*(1-math.cos(w0*i)))
		else:
			traj2x.append(v0*i)
			traj3x.append(vn*i)
			traj2y.append(0)
			traj3y.append(0)
	#av = 0.5
	#for a in np.arange(0,config.predict_time,config.dt):
	#   traj8x.append(ff(a))
	#   traj8y.append(gg(a))
	#av = 1.0
	print("traj1x=",len(traj1x))
	if w0 != 0:
		k = g_.evalf(subs={t:config.predict_time})/f_.evalf(subs={t:config.predict_time})
		k = -1/k
	else:
		k = 0
	print(k)
	#y = k*x+b
	b = traj1y[-1]-k*traj1x[-1]
	print(b)
	anglemax = math.atan2(traj1y[-1]-b,traj1x[-1]-0)
	print(anglemax)
	print(math.degrees(anglemax))
	dx21 = traj2x[-1]-traj1x[-1]
	dy21 = traj2y[-1]-traj1y[-1]
	dx31 = traj3x[-1]-traj1x[-1]
	dy31 = traj3y[-1]-traj1y[-1]
	if av != 0:
		n = len(traj1x)*(math.sqrt((v0**2+vn**2)*0.5)-v0)/av
	else:
		n = 0.5*config.predict_time
	print("n=",n)
	print("len n=",len(traj2x))
	kn = g_.evalf(subs={t:(n/len(traj1x))})/f_.evalf(subs={t:(n/len(traj1x))})
	print('kn=',math.degrees(kn))
	nn = kn*config.dt*len(traj1x)/w0
	print('kn/w0=',len(traj1x)*kn/w0)
	dx41 = traj2x[int(n)]-traj1x[int(n)]
	dy41 = traj2y[int(n)]-traj1y[int(n)]
	dx51 = traj3x[int(n)]-traj1x[int(n)]
	dy51 = traj3y[int(n)]-traj1y[int(n)]
	traj4x = np.array(traj2x)-dx21
	traj4y = np.array(traj2y)-dy21

	traj5x = np.array(traj3x)-dx31
	traj5y = np.array(traj3y)-dy31

	traj6x = np.array(traj2x)-dx41
	traj6y = np.array(traj2y)-dy41

	traj7x = np.array(traj3x)-dx51
	traj7y = np.array(traj3y)-dy51

	#fig = plt.figure()
	fig = plt.figure(figsize=(5.25,5.25))
	ax = fig.add_subplot(111)
	ax.set_aspect("equal")


	plt.plot(traj2x,traj2y,label='Inside circle',c='blue',linestyle='solid')
	plt.plot(traj4x,traj4y,c='blue',linestyle='solid')
	plt.plot(traj6x,traj6y,c='blue',linestyle='solid')
	plt.plot(traj3x,traj3y,label='Outside circle',c='red',linestyle='solid')
	plt.plot(traj5x,traj5y,c='red',linestyle='solid')
	plt.plot(traj7x,traj7y,c='red',linestyle='solid')
	#plt.plot(traj8x,traj8y,label='a = 0.5'+'m/$ \mathit{s}^{2}$',c='black',linestyle='dotted')
	plt.plot(traj1x,traj1y,label='Path',c='black',linestyle='solid')
	#plt.plot((0,traj1x[-1]),(b,traj1y[-1]))
	#plt.plot((0,0),(b,0))
	if w0 != 0:
		c2 = [0,v0/w0]
		c3 = [0,vn/w0]
		c4 = [0-dx21,v0/w0-dy21]
		c5 = [0-dx31,vn/w0-dy31]
		c6 = [0-dx41,v0/w0-dy41]
		c7 = [0-dx51,vn/w0-dy51]
		obx = []
		oby = []
		r2l = []
		r3l = []
		r4l = []
		r5l = []
		o21_error=[]
		o22_error=[]
		o23_error=[]
		o24_error=[]
		p_error=[]
		for i in obst:
			xend = ff(config.predict_time)
			yend = gg(config.predict_time)
			minr1 = 1000
			minr2 = 1000
			minr3 = 1000
			minr4 = 1000
			print('===========')
			print('obstacle',obst.index(i),i[0],i[1])
			obx.append(i[0])
			oby.append(i[1])
			r2 = math.sqrt((i[0]-c2[0])**2+(i[1]-c2[1])**2)
			r3 = math.sqrt((i[0]-c3[0])**2+(i[1]-c3[1])**2)
			r4 = math.sqrt((i[0]-c4[0])**2+(i[1]-c4[1])**2)
			r5 = math.sqrt((i[0]-c5[0])**2+(i[1]-c5[1])**2)
			r6 = math.sqrt((i[0]-c6[0])**2+(i[1]-c6[1])**2)
			r7 = math.sqrt((i[0]-c7[0])**2+(i[1]-c7[1])**2)
			print(r2,r3,r4,r5,r6,r7)
			#print(r2-(v0/w0))
			#print(abs(r2-(v0/w0)))
			print(abs(r2-(v0/w0)),abs(r3-(vn/w0)),abs(r4-(v0/w0)),abs(r5-(vn/w0)),abs(r6-(v0/w0)),abs(r7-(vn/w0)))
			angle = math.atan2(i[1]-b,i[0]-0)
			print(math.degrees(angle))
			#avg_r1 = (abs(r2-(v0/w0))+abs(r3-(vn/w0)))*0.5
			#avg_r2 = (abs(r4-(v0/w0))+abs(r5-(vn/w0)))*0.5
			#avg_r3 = (abs(r6-(v0/w0))+abs(r7-(vn/w0)))*0.5
			print(max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))
			print(min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0)))))
			angle = math.atan2(i[1]-b,i[0])
			if (w0 > 0 and -0.5*math.pi < angle < anglemax) or (w0 < 0 and anglemax < angle < 0.5*math.pi) :
				if r3-(vn/abs(w0))>0 and r5-(vn/abs(w0))>0 and r7-(vn/abs(w0))>0 :
					print('outside')
					if av > 0:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
					else:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				elif r2-(v0/abs(w0))<0 and r4-(v0/abs(w0))<0 and r6-(v0/abs(w0))<0:
				#else:
					print('inside')
					if av > 0:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*min(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*max(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

					else:
						pdist = ((((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+(1-((angle+0.5*math.pi)/(anglemax+0.5*math.pi)))*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))
						#pdist = (0.5)*max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0))))+(0.5)*min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0))))

				else:
					pdist = 0.5*(max(abs(r3-(vn/abs(w0))),abs(r5-(vn/abs(w0))),abs(r7-(vn/abs(w0)))))+0.5*(min(abs(r2-(v0/abs(w0))),abs(r4-(v0/abs(w0))),abs(r6-(v0/abs(w0)))))
			else:
				dist1 = math.sqrt((i[0]-xend)**2+(i[1]-yend)**2)
				dist2 = math.sqrt((i[0])**2+(i[1])**2)
				pdist = min(dist1,dist2)

			#end = v0*config.predict_time+0.5*av*config.predict_time**2
			#end = 0
			#dist1 = math.sqrt((i[0]-xend)**2+(i[1]-yend)**2)
			#dist2 = math.sqrt((i[0])**2+(i[1])**2)
			#pdist = min(dist1,dist2)

			for ii in range(0, traj1.shape[0]):
				dx = traj1[ii, 0] - i[0]
				dy = traj1[ii, 1] - i[1]
				r1 = math.sqrt(dx**2 + dy**2)
				if r1 <= minr1:
					minr1 = r1
			for ii in range(0, traj2.shape[0]):
				dx = traj2[ii, 0] - i[0]
				dy = traj2[ii, 1] - i[1]
				r2 = math.sqrt(dx**2 + dy**2)
				if r2 <= minr2:
					minr2 = r2
			for ii in range(0, traj3.shape[0]):
				dx = traj3[ii, 0] - i[0]
				dy = traj3[ii, 1] - i[1]
				r3 = math.sqrt(dx**2 + dy**2)
				if r3 <= minr3:
					minr3 = r3

			for ii in range(0, traj4.shape[0]):
				dx = traj4[ii, 0] - i[0]
				dy = traj4[ii, 1] - i[1]
				r4 = math.sqrt(dx**2 + dy**2)
				#print(r4)
				if r4 <= minr4:
					minr4 = r4
			#h = f_*(list(obst)[0][0]-(f))+g_*(list(obst)[0][1]-(g))
			f_ = sympy.diff(f,t)
			g_ = sympy.diff(g,t)
			h = f_*(i[0]-(f))+g_*(i[1]-(g))
			t0 = sympy.nsolve(h,t,[0,1])
			if 0<=t0<=1:
				xfoot = f.evalf(subs={t:t0})
				yfoot = g.evalf(subs={t:t0})
				dist = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
				#dist = math.sqrt((list(obst)[0][0]-xfoot)**2+(list(obst)[0][1]-yfoot)**2)
			else:
				#xfoot1 = f.evalf(subs={t:0})
				#yfoot1 = g.evalf(subs={t:0})
				xfoot = f.evalf(subs={t:config.predict_time})
				yfoot = g.evalf(subs={t:config.predict_time})
				dist1 = math.sqrt((i[0])**2+(i[1])**2)
				dist2 = math.sqrt((i[0]-xfoot)**2+(i[1]-yfoot)**2)
				dist = min(dist1,dist2)
			distlist1.append(dist)
			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)
			distlist24.append(minr4)
			distlist3.append(pdist)
			#distlist4.append(pdist2)
			d3min.append(max(r3-(vn/w0),r5-(vn/w0)))
			d3max.append(min(r2-(v0/w0),r4-(v0/w0)))
			print('true dist =',dist)
			print('pdist = ',pdist)
			print('odist = ',minr1)
			error21 = (minr1-dist)
			error22 = (minr2-dist)
			error23 = (minr3-dist)
			error24 = (minr4-dist)
			error3 = (pdist-dist)
			o21_error.append(abs(error21))
			o22_error.append(abs(error22))
			o23_error.append(abs(error23))
			o24_error.append(abs(error24))
			p_error.append(abs(error3))
	else:
		obx = []
		oby = []
		xend = ff(config.predict_time)
		for i in obst:
			print('===========')
			print('obstacle',i[0],i[1])
			obx.append(i[0])
			oby.append(i[1])
			minr1 = 1000
			minr2 = 1000
			minr3 = 1000
			minr4 = 1000
			if 0 <= i[0]<= xend:
				pdist = i[1]
			else:
				dist1 = math.sqrt((i[0]-xend)**2+(i[1]-0)**2)
				dist2 = math.sqrt((i[0])**2+(i[1])**2)
				pdist = min(dist1,dist2)
			for ii in range(0, traj1.shape[0]):
				dx = traj1[ii, 0] - i[0]
				dy = traj1[ii, 1] - i[1]
				r1 = math.sqrt(dx**2 + dy**2)
				if r1 <= minr1:
					minr1 = r1
			for ii in range(0, traj2.shape[0]):
				dx = traj2[ii, 0] - i[0]
				dy = traj2[ii, 1] - i[1]
				r2 = math.sqrt(dx**2 + dy**2)
				if r2 <= minr2:
					minr2 = r2
			for ii in range(0, traj3.shape[0]):
				dx = traj3[ii, 0] - i[0]
				dy = traj3[ii, 1] - i[1]
				r3 = math.sqrt(dx**2 + dy**2)
				if r3 <= minr3:
					minr3 = r3

			for ii in range(0, traj4.shape[0]):
				dx = traj4[ii, 0] - i[0]
				dy = traj4[ii, 1] - i[1]
				r4 = math.sqrt(dx**2 + dy**2)
				#print(r4)
				if r4 <= minr4:
					minr4 = r4
			#h = f_*(list(obst)[0][0]-(f))+g_*(list(obst)[0][1]-(g))

			distlist21.append(minr1)
			distlist22.append(minr2)
			distlist23.append(minr3)
			distlist24.append(minr4)
			distlist3.append(pdist)
			#distlist4.append(pdist2)
			#d3min.append(max(r3-(vn/w0),r5-(vn/w0)))
			#d3max.append(min(r2-(v0/w0),r4-(v0/w0)))
			#print('true dist =',dist)
			print('pdist = ',pdist)
			print('odist = ',minr1)
			#error21 = (minr1-dist)
			#error22 = (minr2-dist)
			#error23 = (minr3-dist)
			#error24 = (minr4-dist)
			#error3 = (pdist-dist)
			#o21_error.append(abs(error21))
			#o22_error.append(abs(error22))
			#o23_error.append(abs(error23))
			#o24_error.append(abs(error24))
			#p_error.append(abs(error3))

	print("********************************")

	#print('original max error = ',max(o_error))
	#print('original min error = ',min(o_error))
	#print('original avg error = ',sum(o_error)/len(o_error))
	#print('proposed max error = ',max(p_error))
	#print('proposed min error = ',min(p_error))
	#print('proposed avg error = ',sum(p_error)/len(p_error))
	#plt.scatter(obx,oby,c='black')
	plt.xlabel("x [m]")
	plt.ylabel("y [m]")
	plt.xlim([min(traj5x)-0.1,max(traj3x)+0.1])
	plt.ylim([min(traj5y)-0.1,max(traj3y)+0.1])

	plt.legend()
	plt.tight_layout()
	plt.show()

	plt.subplots(tight_layout=True)
	mse21 = 0
	mse22 = 0
	mse23 = 0
	mse24 = 0
	mse3 = 0
	for i in range(len(distlist1)):
		mse21=mse21+(distlist21[i]-distlist1[i])**2
		mse22=mse22+(distlist22[i]-distlist1[i])**2
		mse23=mse23+(distlist23[i]-distlist1[i])**2
		mse24=mse24+(distlist24[i]-distlist1[i])**2
		mse3=mse3+(distlist3[i]-distlist1[i])**2
	mse21 = math.sqrt(mse21/len(distlist21))
	mse22 = math.sqrt(mse22/len(distlist22))
	mse23 = math.sqrt(mse23/len(distlist23))
	mse24 = math.sqrt(mse24/len(distlist24))
	mse3 = math.sqrt(mse3/len(distlist3))
	#print(sum(distlist21)/len(distlist21),
	#   sum(distlist22)/len(distlist22),
	#   sum(distlist23)/len(distlist23),
	#   sum(distlist24)/len(distlist24),
	#   sum(distlist3)/len(distlist3))
	#print(mse21,mse22,mse23,mse24,mse3)
	#print(max(o21_error),max(o22_error),max(o23_error),max(o24_error),max(p_error))
	#print(min(o21_error),min(o22_error),min(o23_error),min(o24_error),min(p_error))
	print('rmse')
	for i in [mse21,mse22,mse23,mse24,mse3]:
		print(format(i,'.5f'))
	print('max_error')
	for i in [max(o21_error),max(o22_error),max(o23_error),max(o24_error),max(p_error)]:
		print(format(i,'.5f'))
	print('min_error')
	for i in [min(o21_error),min(o22_error),min(o23_error),min(o24_error),min(p_error)]:
		print(format(i,'.5f'))
	plt.scatter(np.arange(0,len(distlist1)),distlist1,label='Baseline',c='black',marker='o')
	plt.plot(np.arange(0,len(distlist1)),distlist1,c='black')
	plt.scatter(np.arange(0,len(distlist21)),distlist21,label='Original 0.1',c='blue',marker='v')
	plt.plot(np.arange(0,len(distlist21)),distlist21,c='blue')
	plt.scatter(np.arange(0,len(distlist22)),distlist22,label='Original 0.05')
	plt.plot(np.arange(0,len(distlist22)),distlist22)
	plt.scatter(np.arange(0,len(distlist23)),distlist23,label='Original 0.02')
	plt.plot(np.arange(0,len(distlist23)),distlist23)
	plt.scatter(np.arange(0,len(distlist24)),distlist24,label='Original 0.01',marker=',')
	plt.plot(np.arange(0,len(distlist24)),distlist24)
	plt.scatter(np.arange(0,len(distlist3)),distlist3,label='Proposed',c='red',marker='^')
	plt.plot(np.arange(0,len(distlist3)),distlist3,c='red')
	#plt.scatter(np.arange(0,len(distlist4)),distlist4,label='proposed_avg')
	#plt.plot(np.arange(0,len(distlist4)),distlist4)
	#plt.scatter(np.arange(0,len(d3max)),d3max,label='proposed+')
	#plt.plot(np.arange(0,len(d3max)),d3max)
	#plt.scatter(np.arange(0,len(d3min)),d3min,label='proposed-')
	#plt.plot(np.arange(0,len(d3min)),d3min)
	plt.legend()
	#plt.ylim(0.445,0.505)
	x_major_locator=MultipleLocator(5)
	#y_major_locator=MultipleLocator(0.005)
	ax=plt.gca()
	ax.xaxis.set_major_locator(x_major_locator)
	#ax.yaxis.set_major_locator(y_major_locator)
	plt.xlim(-0.5,27.5)
	plt.ylim(0.0,1.6)
	plt.xlabel("Obstacle Number")
	plt.ylabel("Distance [m]")
	plt.show()
	"""
	plt.scatter(np.arange(0,10),distlist1,label='baseline',c='black')
	plt.scatter(np.arange(0,10),d3max,label='d3max',c='blue')
	plt.scatter(np.arange(0,10),d3min,label='d3min',c='red')
	plt.legend()
	plt.ylim(0.48,0.51)
	x_major_locator=MultipleLocator(1)
	ax=plt.gca()
	ax.xaxis.set_major_locator(x_major_locator)
	plt.xlabel("Obstacle Number")
	plt.ylabel("Distance [m]")
	plt.show()
	"""
def bargraph():
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14

	#name_list = ['0.1','0.05','0.02','0.01','Proposed method']
	name_list = ['0.0','0.5','1.0']
	rmse01 = np.array([0.045,0.04387,0.02448])
	rmse005 = np.array([0.02287,0.02278,0.01364])
	rmse002 = np.array([0.00923,0.0093,0.00636])
	rmse001 = np.array([0.00463,0.00468,0.00336])
	rmsep = np.array([0.0,0.0006,0.00149])

	max01 = np.array([0.08662,0.10848,0.03857])
	max005 = np.array([0.04352,0.0546,0.02163])
	max002 = np.array([0.01746,0.02192,0.01686])
	max001 = np.array([0.00874,0.01097,0.00931])
	maxp = np.array([0,0.00196,0.00467])

	min01 = np.array([0.00085,0.00255,0.00245])
	min005 = np.array([0.00128,0.00005,0.00069])
	min002 = np.array([0.00072,0.00035,0.00059])
	min001 = np.array([0.0004,0.00023,0.00044])
	minp = np.array([0,0,0])

	x = np.arange(len(name_list))
	width = 0.1
	fig, ax = plt.subplots()
	rects1 = ax.bar(x - width*2, rmse01, width, label='Origianl method 0.1')
	rects2 = ax.bar(x - width+0.01, rmse005, width, label='Origianl method 0.05')
	rects3 = ax.bar(x + 0.02, rmse002, width, label='Origianl method 0.02')
	rects4 = ax.bar(x + width+ 0.03, rmse001, width, label='Origianl method 0.01')
	rects5 = ax.bar(x + width*2 + 0.04, rmsep, width, label='Proposed method')
	ax.set_ylabel('RMSE')
	ax.set_xlabel('Acceleration['+'m/$ \mathit{s}^{2}$'+']')
	#ax.set_title('这里是标题')
	ax.set_xticks(x)
	ax.set_xticklabels(name_list)
	ax.legend()
	fig.tight_layout()
	plt.show()
	fig, ax = plt.subplots()
	rects1 = ax.bar(x - width*2, max01, width, label='Origianl method 0.1')
	rects2 = ax.bar(x - width+0.01, max005, width, label='Origianl method 0.05')
	rects3 = ax.bar(x + 0.02, max002, width, label='Origianl method 0.02')
	rects4 = ax.bar(x + width+ 0.03, max001, width, label='Origianl method 0.01')
	rects5 = ax.bar(x + width*2 + 0.04, maxp, width, label='Proposed method')
	ax.set_ylabel('Maximum error [m]')
	ax.set_xlabel('Acceleration['+'m/$ \mathit{s}^{2}$'+']')
	#ax.set_title('这里是标题')
	ax.set_xticks(x)
	ax.set_xticklabels(name_list)
	ax.legend()
	fig.tight_layout()
	plt.show()
	fig, ax = plt.subplots(figsize=(5.25,5.25))
	rects1 = ax.bar(x - width*2, min01, width, label='Origianl method 0.1')
	rects2 = ax.bar(x - width+0.01, min005, width, label='Origianl method 0.05')
	rects3 = ax.bar(x + 0.02, min002, width, label='Origianl method 0.02')
	rects4 = ax.bar(x + width+ 0.03, min001, width, label='Origianl method 0.01')
	rects5 = ax.bar(x + width*2 + 0.04, minp, width, label='Proposed method')
	ax.set_ylabel('Minimum error [m]')
	ax.set_xlabel('Acceleration['+'m/$ \mathit{s}^{2}$'+']')
	#ax.set_title('这里是标题')
	ax.set_xticks(x)
	ax.set_xticklabels(name_list)
	ax.legend()
	fig.tight_layout()
	plt.show()
	fig, ax = plt.subplots(figsize=(7,3))
	plt.bar('s',0.07416*1000)#,color="red")
	plt.bar('m',0.01153*1000)#,color="red")
	plt.bar('e',0.08226*1000)#,color="red")
	plt.bar('s+m',0.00755*1000)#,color="red")
	plt.bar('s+e',0.00625*1000)#,color="red")
	plt.bar('m+e',0.01058*1000)#,color="red")
	plt.bar('s+m+e',0.00149*1000)#,color="red")
	#plt.bar('0.1',0.02448*1000,color="black")
	#plt.bar('0.05',0.01364*1000,color="black")
	#plt.bar('0.02',0.00636*1000,color="black")
	#plt.xlabel('Method of circle generation')
	plt.ylabel('RMSE [mm]')
	fig.tight_layout()
	plt.show()
	fig, ax = plt.subplots(figsize=(7,3))
	plt.bar('s',0.03888*1000)#,color="red")
	plt.bar('m',0.03626*1000)#,color="red")
	plt.bar('e',0.0042*1000)#,color="red")
	plt.bar('s+m',0.00241*1000)#,color="red")
	plt.bar('s+e',0.00245*1000)#,color="red")
	plt.bar('m+e',0.00345*1000)#,color="red")
	plt.bar('s+m+e',0.0006*1000)#,color="red")
	#plt.bar('0.1',0.04387*1000,color="black")
	#plt.bar('0.05',0.02278*1000,color="black")
	#plt.bar('0.02',0.0093*1000,color="black")
	#plt.xlabel('Method of circle generation')
	plt.ylabel('RMSE [mm]')
	fig.tight_layout()
	plt.show()
	fig, ax = plt.subplots()
	plt.bar('s',0.21531)
	plt.bar('m',0.03965)
	plt.bar('g',0.23989)
	plt.bar('s+m',0.03651)
	plt.bar('s+g',0.01706)
	plt.bar('m+g',0.03965)
	plt.bar('s+m+g',0.00467)
	plt.xlabel('Method of circle generation')
	plt.ylabel('Maximum error [m]')
	fig.tight_layout()
	plt.show()

def func2(x,a,b,c):
	return a*np.log(x+b)+c

def linegraph():
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14
	plt.subplots(figsize=(7,5))
	#x = np.array([0.1,0.05,0.02])*1000
	#y1 = np.array([0.009269,0.01691,0.040201])*1000
	#y2 = np.array([0.005795,0.005795,0.005795])*1000
	#y3 = np.array([0.54,0.54,0.54])*1000

	#x = np.array([0.009269,0.01691,0.040201])*1000
	#y = np.array([0.02448,0.01364,0.00636])*1000
	"""
	x = np.array([0.0,0.5,1.0])
	y_o1 = np.array([45.0,43.9,24.5])
	y_o2 = np.array([22.9,22.8,13.6])
	y_o3 = np.array([9.2,9.3,6.4])

	y_p1 = np.array([0.0,38.9,74.2])
	y_p2 = np.array([0.0,36.3,11.5])
	y_p3 = np.array([0.0,4.2,82.3])
	y_p4 = np.array([0.0,2.4,7.6])
	y_p5 = np.array([0.0,2.5,6.3])
	y_p6 = np.array([0.0,3.5,10.6])
	y_p7 = np.array([0.0,0.6,1.5])

	xnew = np.linspace(x.min(),x.max(),20)
	f_o1 = PchipInterpolator(x,y_o1)
	f_o2 = PchipInterpolator(x,y_o2)
	f_o3 = PchipInterpolator(x,y_o3)
	f_p1 = PchipInterpolator(x,y_p1)
	f_p2 = PchipInterpolator(x,y_p2)
	f_p3 = PchipInterpolator(x,y_p3)
	f_p4 = PchipInterpolator(x,y_p4)
	f_p5 = PchipInterpolator(x,y_p5)
	f_p6 = PchipInterpolator(x,y_p6)
	f_p7 = PchipInterpolator(x,y_p7)

	y_o1_new = f_o1(xnew)
	y_o2_new = f_o2(xnew)
	y_o3_new = f_o3(xnew)
	y_p1_new = f_p1(xnew)
	y_p2_new = f_p2(xnew)
	y_p3_new = f_p3(xnew)
	y_p4_new = f_p4(xnew)
	y_p5_new = f_p5(xnew)
	y_p6_new = f_p6(xnew)
	y_p7_new = f_p7(xnew)

	plt.plot(xnew,y_o1_new,c='black',linestyle='dotted')
	plt.plot(xnew,y_o2_new,c='black',linestyle='dashed')
	plt.plot(xnew,y_o3_new,c='black',linestyle='solid',linewidth='2',label='Original method')

	plt.plot(xnew,y_p1_new,c='blue',linestyle='dotted')
	plt.plot(xnew,y_p2_new,c='blue',linestyle='dashed')
	plt.plot(xnew,y_p3_new,c='blue',linestyle='solid',label='Proposed method:one pair')
	plt.plot(xnew,y_p4_new,c='green',linestyle='dotted')
	plt.plot(xnew,y_p5_new,c='green',linestyle='dashed')
	plt.plot(xnew,y_p6_new,c='green',linestyle='solid',label='Proposed method:two pairs')
	plt.plot(xnew,y_p7_new,c='red',linestyle='solid',linewidth='2',label='Proposed method:three pairs')
	"""
	x_p = np.array([5.8,5.8,5.8])
	y_p = np.array([0.0,0.6,1.5])
	x_o = np.array([9.3,9.3,9.3,11.8,11.8,11.8,18.4,18.4,18.4])
	y_o = np.array([45,43.9,24.5,22.9,22.8,13,9.2,9.3,6.4])
	xnew = np.linspace(x_o.min(),x_o.max(),20)
	#print("well")
	#f_o = interp1d(x_o,y_o,kind="cubic")
	popt,pcov = curve_fit(func2,x_o,y_o)
	ynew=[]
	print(popt,pcov)
	for i in xnew:
		ynew.append(func2(i,popt[0],popt[1],popt[2]))
	print(ynew)

	plt.plot([0,xnew.max()],[0,0],c='blue',linestyle='solid',label='Baseline')
	plt.plot(xnew,ynew,c='black',linestyle='solid',label='Original method')
	plt.scatter(x_o,y_o,c='black')
	plt.scatter(x_p,y_p,c='red',label='Proposed method')

	#plt.scatter(0.005795*1000,0.00149*1000,c='red',label='Proposed method')
	#plt.plot(x,y2,c='red',label='Proposed method')
	#plt.plot(x,y3,c='blue',label='Baseline')
	#plt.semilogy(x,y1,c='blue',label='Original method')
	#plt.semilogy(x,y2,c='red',label='Proposed method')
	#plt.semilogy(x,y3,c='black',label='Baseline')
	#plt.yscale('log')
	#plt.xlabel('Time inverse [ms]')
	plt.xlabel('DWA time per frame [ms]')
	#plt.xlabel('Acceleration ['+'m/$ \mathregular{s^2}$'+']')
	plt.ylabel('RMSE [mm]')
	#plt.xlim(0,None)
	#plt.ylim(0,None)


	ax=plt.gca()
	plt.legend()
	plt.tight_layout()
	plt.show()


def linegraph2():
	plt.rcParams["axes.labelsize"]=14
	plt.rcParams["xtick.labelsize"]=14
	plt.rcParams["ytick.labelsize"]=14
	fig,ax = plt.subplots(figsize=(7,3))
	x = np.array([0.1,0.05,0.02,0.01])*1000
	y0 = np.array([0.045,0.02287,0.00923,0.00463])*1000
	y5 = np.array([0.04387,0.02278,0.0093,0.00468])*1000
	y10 = np.array([0.02448,0.01364,0.00636,0.00336])*1000
	yy0 = np.array([0,0,0,0])*1000
	yy5 = np.array([0.0006,0.0006,0.0006,0.0006])*1000
	yy10 = np.array([0.00149,0.00149,0.00149,0.00149])*1000

	#plt.scatter(x,y1,c='blue',label='Original method')
	#plt.scatter(x,y2,c='red',label='Proposed method')
	#plt.scatter(x,y3,c='black',label='Baseline')

	#xnew = np.linspace(x.min(),x.max(),10)
	#f = interp1d(x,y1,kind='linear')
	#ynew = f(xnew)
	plt.plot(x[0:3],y0[0:3],c='black',linestyle='dotted')
	plt.plot(x[0:3],y5[0:3],c='black',linestyle='dashed')
	plt.plot(x[0:3],y10[0:3],c='black',linestyle='solid',label='Original method')
	plt.plot(x[0:3],yy0[0:3],c='red',linestyle='dotted')
	plt.plot(x[0:3],yy5[0:3],c='red',linestyle='dashed')
	plt.plot(x[0:3],yy10[0:3],c='red',linestyle='solid',label='Proposed method')
	#axins = inset_axes(ax,width="20%",height="30%",loc="lower left",bbox_to_anchor=(0.7,0.1,1,1),bbox_transform=ax.transAxes)
	#axins.plot(x[0:3],yy0[0:3],c='red',linestyle='dotted')
	#axins.plot(x[0:3],yy5[0:3],c='red',linestyle='dashed')
	#axins.plot(x[0:3],yy10[0:3],c='red',linestyle='solid',label='Proposed method')
	#axins.set_xlim(90,100)
	#axins.set_ylim(90,100)
		#plt.semilogy(x,y1,c='blue',label='Original method')
	#plt.semilogy(x,y2,c='red',label='Proposed method')
	#plt.semilogy(x,y3,c='black',label='Baseline')
	#plt.yscale('log')
	plt.xlabel('Time inverse [ms]')
	plt.ylabel('RMSE [ms]')
	ax=plt.gca()
	plt.legend()
	plt.tight_layout()
	plt.show()
if __name__ == '__main__':
	#profiler = Profiler()
	#profiler.start()
	rospy.init_node('dwa')

	#main()
	config= Config()
	draw_traj(config)



