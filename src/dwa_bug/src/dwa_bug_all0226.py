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
import datetime
import time
import matplotlib as mpl
import sympy
import mpmath as mp
import random

from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
from scipy.interpolate import interp1d,PchipInterpolator

from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
#from sympy import *
from pathsolver import solver
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mpl_toolkits.axes_grid1 import make_axes_locatable
from mpl_toolkits.axes_grid1.inset_locator import mark_inset,inset_axes
from pyinstrument import Profiler
import PIL.Image as Image

class Config():
	# simulation parameters
	def __init__(self):
		self.dt = 0.1 # [s]
		self.perdict_time = 2.0

		# robot parameter
		self.max_linear_velocity = 2.0# [m/s]
		self.min_linear_velocity = 0.0

		self.max_angular_velocity = 6.28  # [rad/s]360 degree
		self.min_angular_velocity = -6.28

		self.max_linear_acceleration = 1.0#2.0  # [m/ss]
		self.max_angular_acceleration = 5.24  # [rad/ss]300 degree

		self.max_linear_jerk = 0.5  # [m/sss]
		self.max_angular_jerk = 1.31  # [m/sss]

		self.linear_velocity_resolution = self.max_linear_acceleration*self.dt*0.5
		self.angular_velocity_resolution = self.max_angular_acceleration*self.dt*0.5

		self.linear_acceleration_resolution = self.max_linear_acceleration*0.5
		self.angular_acceleration_resolution = self.max_angular_acceleration*0.5

		self.linear_jerk_resolution = self.max_linear_jerk*0.5
		self.angular_jerk_resolution = self.max_angular_jerk*0.5

		self.robot_radius = 0.2 # [m]
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

		self.current_v = 0.0
		self.current_w = 0.0
		self.r = rospy.Rate(10)

		self.error = []

		self.linear_velocity_list = [0.0]
		self.angular_velocity_list = [0.0]
		self.linear_acceleration_list = [0.0]
		self.angular_acceleration_list = [0.0]
		self.linear_jerk_list = [0.0]
		self.angular_jerk_list = [0.0]

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
		#self.laser_reso = []
		self.map = '../world/1181.png'

	def assignOdomCoords(self, msg):
		# X- and Y- coords and pose of robot fed back into the robot config
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		r = R.from_quat([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
		(roll,pitch,theta) = r.as_euler('xyz',degrees=False)
		self.th = theta

	# Callback for cmd_vel
	def callcmd(self, msg):
		self.current_v = msg.twist.twist.linear.x
		self.current_w = msg.twist.twist.angular.z

class Obstacles():
	def __init__(self):
		# Set of coordinates of obstacles in view
		self.obst = set()
		self.obst_rc = set()
		self.laser_nums = 0
	# Custom range implementation to loop over LaserScan degrees with
	# a step and include the final degree
	def myRange(self,start,end,step):
		i = start
		while i < end:
			yield i
			i += step
		yield end

	def assignObs(self, msg, config):
		deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
		self.laser_nums = len(msg.ranges)
		self.obst = set()   # reset the obstacle set to only keep visible objects
		self.obst_rc = set()
		ABT = np.array([[math.cos(config.th),-math.sin(config.th),0,config.x],
						[math.sin(config.th),math.cos(config.th),0,config.y],
						[0,0,1,0],
						[0,0,0,1]])
		CP = np.array([config.goalX,config.goalY,0,1])
		ANTI_ABT = np.linalg.inv(ABT)
		DP = np.dot(ANTI_ABT,CP)
		for angle in self.myRange(0,deg-1,1):
		#for angle in self.myRange(int(deg*0.25),int(deg*0.75),1):
			distance = msg.ranges[angle]
			if (distance < config.sensor_range):
				scanTheta = -math.pi + angle * msg.angle_increment
				xb = distance * math.cos(scanTheta)#obstacle in robot coordinate
				yb = distance * math.sin(scanTheta)#obstacle in robot coordinate
				BP = np.array([xb,yb,0,1])
				AP = np.dot(ABT, BP)#obstacle in world coordinate
				self.obst.add((AP[0],AP[1]))
				self.obst_rc.add((xb,yb))
				if len(list(self.obst))!=len(list(self.obst_rc)):
					print("not equal")
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

def accelerationToVelocity(config,a,mode):
	time = 0
	predict_list = []
	if mode =='av':
		predict_list.append(config.linear_velocity_list[-1])
		while time <= config.predict_time:
			if predict_list[-1]+a*config.dt >= 0:
				predict_list.append(min(config.max_linear_velocity,predict_list[-1]+a*config.dt))
			else:
				predict_list.append(max(0,predict_list[-1]+a*config.dt))
			time = time + config.dt
	elif mode =='aw':
		predict_list.append(config.angular_velocity_list[-1])
		while time <= config.predict_time:
			if predict_list[-1]+a*config.dt >= 0:
				predict_list.append(min(config.max_angular_velocity, w_predict_list[-1] + aw * config.dt))
			else:
				predict_list.append(max(-config.max_angular_velocity, w_predict_list[-1] + aw * config.dt))
			time = time + config.dt
	del(predict_list[0])
	return predict_list

# Determine the dynamic window from robot configurations
def calcDynamicWindow(x, config):
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
	dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
	return dw

# Calculate a trajectory sampled across a prediction time
def calcTrajectory(config, xinit, v_predict_list, w_predict_list,):
	x = np.array(xinit)
	traj = np.array(xinit)  # many motion models stored per trajectory

	for i in range(len(v_predict_list)):
		x = motion(x, [v_predict_list[i], w_predict_list[i]], config.dt)
		traj = np.vstack((traj, x))
	traj = np.delete(traj,0,0)
	return traj

# Calculate trajectory, costings, and return velocities to apply to robot
def calcFinalInput(x, u, dw, config, obst,obst_rc):

	xinit = x[:]
	max_cost = -100000000
	state = 0
	max_u = u

	# evaluate all trajectory with sampled input in dynamic window
	if config.type == "dwa":
		deaccel_dist = 0.7
		line = 0
		for v in np.arange(dw[0], dw[1]+0.5*config.linear_velocity_resolution,config.linear_velocity_resolution):
			for w in np.arange(dw[2], dw[3]+0.5*config.angular_velocity_resolution, config.angular_velocity_resolution):
				v_predict_list = [v]*int(config.predict_time / config.dt)
				w_predict_list = [w]*int(config.predict_time / config.dt)
				traj = calc_trajectory(xinit, v_predict_list, w_predict_list, config)
				#to_goal_cost,stop_mark,indexremain,remain_end = calc_to_goal_cost(traj, config)
				to_goal_cost = calc_to_goal_cost(traj, config)
				ob_cost,_ = calc_obstacle_cost(traj, obst, config)
				odom_angle_cost = calc_odom_angle_cost(traj,config)
				add_cost = 0
				speed_cost = abs(v/config.max_linear_velocity)

				final_cost = (1.0*add_cost+0.0*odom_angle_cost+1.0*to_goal_cost+2.0*ob_cost +config.v_weight*speed_cost)
				if final_cost > max_cost:
					max_cost = final_cost
					max_u = [v, w]
		if max_cost <= 0:
			state = -1
		return max_u,state,line

	elif config.type == "a":
		per_pair1 = []
		per_pair2 = []
		per_pair3 = []

		for av in np.arange(-config.max_linear_acceleration, config.max_linear_acceleration+0.5*config.linear_acceleration_resolution,config.linear_acceleration_resolution):
			for w in np.arange(dw[2], dw[3]+0.5*config.angular_velocity_resolution, config.angular_velocity_resolution):
				v_predict_list = accelerationToVelocity(config, av, 'av')
				w_predict_list = [w]*int(config.predict_time / config.dt)


				time_s = time.perf_counter()
				traj = calcTrajectory(config, xinit, v_predict_list, w_predict_list)
				ob_cost1, _ = calcObstacleCost(config, traj, obst)
				to_goal_cost1 = calcToGoalCost(config, traj)
				time_e = time.perf_counter()

				odom_angle_cost = 0#calcOdomAngleCost(config, traj)
				speed_cost = np.mean(v_predict_list)/config.max_linear_velocity
				final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost1+2.0*ob_cost1 +config.v_weight*speed_cost)

				per_pair1.append(time_e - time_s)

				if final_cost > max_cost:
					max_cost = final_cost
					max_u = [v_predict_list[int(0.1/config.dt-1)], w, av]
		config.cal_time1.append(sum(per_pair1))
		if max_cost <= 0:
			state = -1
		return max_u,state

	elif config.type == "a2":
		per_pair1=[]
		per_pair2=[]
		per_pair3=[]
		for av in np.arange(-config.max_linear_acceleration, config.max_linear_acceleration+0.5*config.linear_acceleration_resolution,config.linear_acceleration_resolution):
			for w in np.arange(dw[2], dw[3]+0.5*config.angular_velocity_resolution, config.angular_velocity_resolution):
				v_predict_list = accelerationToVelocity(config,av,'av')
				w_predict_list = [w]*int(config.predict_time / config.dt)

				time_s = time.perf_counter()
				ob_cost2,xl,yl,_,_= calcObstacleCost2(config, obst_rc,av,v_predict_list,w)
				to_goal_cost2 = calcToGoalCost2(config,xl,yl)
				time_e = time.perf_counter()

				odom_angle_cost = 0#calcOdomAngleCost(config, traj)
				speed_cost = np.mean(v_predict_list)/config.max_linear_velocity
				final_cost = (0.0*odom_angle_cost+1.0*to_goal_cost2+2.0*ob_cost2 +config.v_weight*speed_cost)

				per_pair1.append(time_e-time_s)

				if final_cost > max_cost:
					max_cost = final_cost
					max_u = [v_predict_list[int(0.1/config.dt-1)], w,av]

		config.cal_time1.append(sum(per_pair1))
		if max_cost <= 0:
			state = -1
		return max_u,state

# Calculate obstacle cost inf: collision, 0:free

def calcObstacleCost(config, traj, obst):
	minr = 1000
	# Loop through every obstacle in set and calc Pythagorean distance
	# Use robot radius to determine if collision
	path = traj[:,0:2]
	obst = list(obst)
	if len(obst) == 0:
		minr = 1000
	else:
		obst = np.array(obst)
		config.ob_nums.append(obst.shape[0])
		dist_matrix = distance.cdist(obst,path,metric="euclidean")
		#dist_matrix = np.sqrt(np.sum((obst[:,np.newaxis]-path[np.newaxis,:])**2,axis=2))
		minr = np.min(dist_matrix)
	if minr <= config.robot_radius:
		cost = -10000  # collision
	elif config.robot_radius< minr <= 1.2:
		cost = minr - 0.2
	else:
		cost = 1
	return cost,minr

def calcObstacleCost2(config,obst_rc,av,v_predict_list,w):
	minr = 1000
	vn = v_predict_list[-1]
	v0 = config.linear_velocity_list[-1]
	w0 = w
	def ff_(t):
		if av > 0:
			acct = (config.max_linear_velocity-v0)/av
			if 0 <=t<=acct:
				ff_ = (v0+av*t)*math.cos(w0*t)
			elif acct<t<=config.predict_time:
				ff_ = config.max_linear_velocity*math.cos(w0*t)
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
			acct = (config.max_linear_velocity-v0)/av
			if 0 <=t<=acct:
				gg_ = (v0+av*t)*math.sin(w0*t)
			elif acct<t<=config.predict_time:
				gg_ = config.max_linear_velocity*math.sin(w0*t)
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
			acct = (config.max_linear_velocity-v0)/av
			x0 = (av*math.cos(w0*0)+w0*(v0+av*0)*math.sin(w0*0))/(w0**2)
			dx1 = x0 -0
			if 0 <=t<=acct:
				f = (av*math.cos(w0*t)+w0*(v0+av*t)*math.sin(w0*t))/(w0**2)-dx1
			elif acct <t<=config.predict_time:
				x1 = (av*math.cos(w0*acct)+w0*(v0+av*acct)*math.sin(w0*acct))/(w0**2)-dx1
				x2 = config.max_linear_velocity*math.sin(w0*acct)/w0
				dx2 = x2-x1
				f = config.max_linear_velocity*math.sin(w0*t)/w0-dx2
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
			acct = (config.max_linear_velocity-v0)/av
			y0 = (av*math.sin(w0*0)-w0*(v0+av*0)*math.cos(w0*0))/(w0**2)
			dy1 = y0 -0
			if 0 <=t<=acct:
				g = (av*math.sin(w0*t)-w0*(v0+av*t)*math.cos(w0*t))/(w0**2)-dy1
			elif acct <t<=config.predict_time:
				y1 = (av*math.sin(w0*acct)-w0*(v0+av*acct)*math.cos(w0*acct))/(w0**2)-dy1
				y2 = -config.max_linear_velocity*math.cos(w0*acct)/w0
				dy2 = y2-y1
				g = -config.max_linear_velocity*math.cos(w0*t)/w0-dy2
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
			midtime = config.predict_time * 0.5

		xmid = ff(midtime)
		ymid = gg(midtime)
		dx21 = (v0/w0) * math.sin(w0 * config.predict_time) - xend
		dy21 = (v0/w0) * (1 - math.cos(w0 * config.predict_time)) - yend
		dx31 = (vn/w0) * math.sin(w0 * config.predict_time) - xend
		dy31 = (vn/w0) * (1 - math.cos(w0 * config.predict_time)) - yend

		dx41 = (v0/w0)*math.sin(w0*midtime)-xmid
		dy41 = (v0/w0)*(1-math.cos(w0*midtime))-ymid
		dx51 = (vn/w0)*math.sin(w0*midtime)-xmid
		dy51 = (vn/w0)*(1-math.cos(w0*midtime))-ymid

		inside_circle_center = np.array([[0,v0/w0],#2,4,6
										[0-dx21,v0/w0-dy21],
										[0-dx41,v0/w0-dy41]])
		outside_circle_center=np.array([[0,vn/w0],#3,5,7
										[0-dx31,vn/w0-dy31],
										[0-dx51,vn/w0-dy51]])

		if ff_(config.predict_time) != 0:
			k = gg_(config.predict_time) / ff_(config.predict_time)
			k = -1/k
		else:
			k = 1
		b = yend-k*xend
		anglemax = math.atan2(yend-b,xend)

		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
			obst_inrange=np.array([0])
			obst_outrange=np.array([0])
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
			#per_pair1.append(obst_inrange.shape[0])
			#per_pair2.append(obst_outrange.shape[0])

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

	else:
		if av > 0:
			acct = (config.max_linear_velocity-v0)/av
			xend = (v0*acct+0.5*av*acct**2)+config.max_linear_velocity*(config.predict_time - acct)
		elif av < 0:
			acct = abs(v0/av)
			if acct > config.predict_time:
				xend = (v0 * config.predict_time + 0.5 * av * config.predict_time ** 2)
			else:
				xend = (v0*acct+0.5*av*acct**2)
		else:
			xend = v0*config.predict_time
		yend = 0
		obst_rc_list = list(obst_rc)
		if len(obst_rc_list) == 0:
			dist = 1000
			obst_inrange=np.array([0])
			obst_outrange=np.array([0])

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

	return cost,xend,yend,obst_inrange.shape[0],obst_outrange.shape[0]

# Calculate goal cost via Pythagorean distance to robot
def calcToGoalCost(config, traj):
	# If-Statements to determine negative vs positive goal/trajectory position
	# traj[-1,0] is the last predicted X coord position on the trajectory
	remain_end = math.sqrt((config.localX-traj[-1,0])**2 + (config.localY-traj[-1,1])**2)
	cost = 1 -(remain_end / 5)
	return cost

def calcToGoalCost2(config,xl,yl):
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

def calcOdomAngleCost(config, traj):
	#function in dwa paper
	angle_end1 = math.atan2(config.localY-traj[-1,1],config.localX-traj[-1,0])#-traj[-1,2]
	angle_end2 = traj[-1,2]
	cost = 1-abs((angle_end1-angle_end2)/math.pi)
	return cost

def calcTrajAngleCost(config, traj):
	angle_start = traj[0,2]
	angle_end = traj[-1,2]
	cost = 1-abs((angle_end-angle_start)/math.pi)
	return cost

# Begin DWA calculations
def dwaControl(x, u, config,obst,obst_rc):
	# Dynamic Window control
	dw = calcDynamicWindow(x, config)
	u,state= calcFinalInput(x, u, dw, config, obst,obst_rc)
	return u,state
# Determine whether the robot has reached its goal
def atGoal(config, x):
	# check at goal
	if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) <= config.stop_dist:
		#if x[3] <= 0.1*config.max_linear_velocity:
		if config.current_v <= 0.1*config.max_linear_velocity:
			return True
	return False

def reject_outliers(data, m = 2.):
	data = np.array(data)
	d = np.abs(data - np.median(data))
	mdev = np.median(d)
	s = d/mdev if mdev else np.zeros(len(d))
	return data[s<m]

def main():
	print(__file__ + " start!!")
	config = Config()
	config.path = ast(config)

	obs = Obstacles()
	subOdom = rospy.Subscriber("base_pose_ground_truth", Odometry, config.assignOdomCoords)
	subLaser = rospy.Subscriber("/base_scan", LaserScan, obs.assignObs, config)
	subCmd = rospy.Subscriber("odom", Odometry, config.callcmd)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	speed = Twist()
	# initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
	x = np.array([config.x, config.y, config.th, config.current_v, config.current_w])
	# initial linear and angular velocities
	u = np.array([0.0, 0.0])
	profiler = Profiler()
	profiler.start()
	t = 0
	while not rospy.is_shutdown():
		t = t + 1
		if (atGoal(config, x) == False) and t <= 500:
			u, state = dwaControl(x, u, config, obs.obst,obs.obst_rc)

			if u[0] >= config.max_linear_velocity:
				speed.linear.x = config.max_linear_velocity
			else:
				speed.linear.x = u[0]
			speed.angular.z = u[1]
		else:
			print(config.linear_velocity_list[-1])
			print(config.angular_velocity_list[-1])
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			pub.publish(speed)
			break
		pub.publish(speed)
		x = [config.x, config.y, config.th, config.current_v, config.current_w]
		config.linear_velocity_list.append(config.current_v)#velocity
		config.linear_acceleration_list.append(u[2])#acceleration
		config.r.sleep()
	#print(sum(config.cal_time))
	profiler.stop()
	print("obst_mean=", np.mean(config.ob_nums))

	time_day = time.strftime("%Y%m%d", time.localtime())
	path = '../results/{}/'.format(time_day)
	f = open(path+config.type+'*10.txt', "a")
	f.write('\n')
	f.write(str(config.dt)+','+'a3'+','+str(obs.laser_nums))
	f.write('\n')
	#config.cal_time1 = reject_outliers(config.cal_time1)
	#config.cal_time2 = reject_outliers(config.cal_time2)
	f.write(str(len(config.cal_time1))+',')
	f.write(str(np.mean(config.cal_time1))+',')
	f.write(str(np.std(config.cal_time1))+',')
	#f.write(str(np.mean(config.cal_time2[0:int(len(config.cal_time2)*0.5)]))+',')
	#f.write(str(np.std(config.cal_time2[0:int(len(config.cal_time2)*0.5)]))+',')
	f.write('\n')
	#profiler.print(file=f)
	f.close()
	profiler.print()


if __name__ == '__main__':
	rospy.init_node('dwa')
	main()
	"""
	time1 = time.perf_counter_ns()
	for i in range(100):
		arr1 = np.array(range(0,180))
		arr1 = np.reshape(arr1,[-1,2])
		arr2 = np.random.randint(200,400,100)
		arr2 = np.reshape(arr2,[-1,2])
		dist_matrix = distance.cdist(arr1,arr2,metric="euclidean")
	time2 = time.perf_counter_ns()
	print((time2-time1)/1000000000/100*25)
	"""


