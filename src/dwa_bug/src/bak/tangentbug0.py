#!/usr/bin/env python
#--coding:utf8--

import rospy
import tf
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt32

# max range is the artificial value used as the maximum range of the laser scanner
maxRange = 3.0

# determines if the robot can see the goal directly
goalVisible = 0

# threshold that determines if a point in the laser scan is a discontinuity
discThresh = 0.1

scan = LaserScan()
xList = [-20]
yList = [3]
dList = []
def scanUpdate(lScan):
	global scan
	scan = lScan

def current():
	current = rospy.wait_for_message('base_pose_ground_truth', Odometry)
	p = current.pose.pose.position
	q = (
			current.pose.pose.orientation.x,
			current.pose.pose.orientation.y,
			current.pose.pose.orientation.z,
			current.pose.pose.orientation.w)
	t = math.degrees(math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[2]**2+q[1]**2)))
	cx = current.pose.pose.position.x
	cy = current.pose.pose.position.y
	return cx,cy

def tangentbug():
	global goalVisible
	global scan

	goalfromlaser = rospy.Publisher('robot_0/robot2goal',Twist,queue_size=10)
	robotController = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	listener = tf.TransformListener()
	rate = rospy.Rate(1)
	#bestAngle = 0.0
	#besti = 0
	bestDist = 10000.0
	currentDist = []
	smoothNess = []
	state = [] #0=FR,1=FL,2=F,3=RR,4=RL,5=TR,6=TL
	while not rospy.is_shutdown():
		try:

			(trans,rot) = listener.lookupTransform('robot0/trueOdom','robot0/goal',rospy.Time(0))
			twi = Twist()#goal相对于现在位置的坐标
			twi.linear.x = trans[0]
			twi.linear.y = trans[1]
			twi.linear.z = trans[2]
			#ignore angular orientations since we only care about getting to the goal position
			twi.angular = Vector3(0.0,0.0,0.0)
			#print(rot)
			if math.sqrt((twi.linear.x) ** 2 + (twi.linear.y) ** 2) > 0.5:
				#print("1")
				cx,cy = current()
				xList.append(cx)
				yList.append(cy)
				d = math.sqrt((xList[-1]-xList[-2]) ** 2 + (yList[-1]-yList[-2]) ** 2)
				dList.append(d)
				if len(dList) > 3:
					b = math.sqrt((xList[-1]-xList[-2]) ** 2 + (yList[-1]-yList[-2]) ** 2)
					c = math.sqrt((xList[-1]-xList[-3]) ** 2 + (yList[-1]-yList[-3]) ** 2)
					a = math.sqrt((xList[-2]-xList[-3]) ** 2 + (yList[-2]-yList[-3]) ** 2)
					#print(a,b,c)
					if a != 0 and b != 0:
						acosvalue = (a ** 2 + b ** 2 - c ** 2) / (2 * a * b)
						#print(acosvalue)
						if (-1.0 <= acosvalue <= 1.0):
							angle = math.pi - math.acos(acosvalue)
							k = 2 * angle/(a+b)
							#print(angle,k)
							smoothNess.append(k)
				# trans is a list of 3 elements x,y,z of the transform. rot is a list of
				# 4 elements of the quaternion for the transform

				#print(trans)

				currentDist.append(math.sqrt(twi.linear.x ** 2 + twi.linear.y ** 2))
				control = Twist()
				control.linear = Vector3(0.0,0.0,0.0)
				control.angular = Vector3(0.0,0.0,0.0)
				angle2goal = math.atan2(trans[1],trans[0])#goal相对于现在位置的方位角

				sensorIndex = int((angle2goal-scan.angle_min)/scan.angle_increment)
				#print(angle2goal)
				# check if we can see the goal directly. Move towards it if we can
				#print(scan.ranges[sensorIndex])
				if (scan.ranges[sensorIndex] >= maxRange):
					#print(angle2goal)
					goalVisible = 1
					if (angle2goal > 0):
						control.linear.x = 0.1
						control.angular.z = 0.2
						state = 0
						#print(goalVisible,"right")
					elif (angle2goal < 0):
						control.linear.x = 0.1
						control.angular.z = -0.2
						state = 1
						#print(goalVisible,"left")
					else:
						control.linear.x = 0.3
						control.angular.z = 0.0
						state = 2
						#print(goalVisible,"front")
				else:
					goalVisible = 0
				#print(goalVisible)
				# if we can't see the goal directly, check for the best direction of travel
				if goalVisible == 0:


					for i in range(len(scan.ranges)):
						# check for discontinuties within a specified threshold
						if (i>0) and (abs(scan.ranges[i]-scan.ranges[i-1]) > discThresh):
							# output the index for the discontinuity and the angle value and the distance to that discontinuity
							discDist = scan.ranges[i]
							if discDist==float('Inf'):
								discDist = scan.range_max
							dAng = scan.angle_min + i * scan.angle_increment#突变点的弧度
							xDist = discDist * math.cos(dAng)#突变点坐标
							yDist = discDist * math.sin(dAng)#突变点坐标
							heurDist = math.sqrt((twi.linear.x-xDist) ** 2 + (twi.linear.y-yDist) ** 2)#突变点和goal的距离
							#if ((heurDist + discDist) < bestDist):
							if ((heurDist+discDist)  < bestDist):
								bestDist = heurDist + discDist
								bestAngle = dAng
								besti = i
					#print(bestDist)
					#print(bestAngle)
					#print(heurDist,discDist)
					#print(besti)
					# drive towards the best heuristic or turn towards it if we're not facing it already

					if ((bestAngle) > 0):
						control.linear.x = 0.3
						control.angular.z = 0.1
						state = 3
						#print(goalVisible,"right")
					elif ((bestAngle) < 0):
						control.linear.x = 0.3
						control.angular.z = -0.1
						state = 4
						#print(goalVisible,"left")
					else:
						control.linear.x = 0.3
						control.angular.z = 0.0
						state = 2
						#print(goalVisible,"front")
					# prioritize avoiding obstacles
					"""
					if (besti > 90) and (besti < (len(scan.ranges)-90)):
						if scan.ranges[besti+20] < 2.0:
							control.linear.x = 0.2
							control.angular.z = 0.5
						elif scan.ranges[besti-20] < 2.0:
							control.linear.x = 0.2
							control.angular.z = -0.5
						#print("???1")
					elif (besti > 90) and (besti < (len(scan.ranges)-90)):
						if scan.ranges[besti+30] < 2.0:
							control.linear.x = 0.2
							control.angular.z = 0.5
						elif scan.ranges[besti-30] < 2.0:
							control.linear.x = 0.2
							control.angular.z = -0.5
						#print("???2")
					"""
				# if obstacles are too close to the robot, prioritize avoiding them
					j = int(len(scan.ranges)/2) - 70#290
					m = int(len(scan.ranges)/2) - 10#350
					k = int(len(scan.ranges)/2) + 70#430
					n = int(len(scan.ranges)/2) + 10#370

					for i in range(j,m):
						if (scan.ranges[i] < 0.5):
							control.linear.x = 0.0
							control.angular.z = 0.5
							state = 5
							#print(goalVisible,"right avoiding")
					for i in range(n,k):
						if (scan.ranges[i] < 0.5):
							control.linear.x = 0.0
							control.angular.z = -0.5
							state = 6
				# stop moving if we're close enough to the goal
			else:
				#print("2")
				control.linear.x = 0.0
				control.angular.z = 0.0
				robotController.publish(control)
				break
			robotController.publish(control)
			goalfromlaser.publish(twi)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	rate.sleep()

if __name__ == '__main__':
    try:
    	rospy.sleep(0.5)
    	rospy.init_node('tf_robot0_tbug')
    	rospy.Subscriber("/base_scan",LaserScan,scanUpdate)
    	tangentbug()
    	rospy.spin()
    except rospy.ROSInterruptException: pass
