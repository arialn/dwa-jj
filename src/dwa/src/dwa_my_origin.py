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
import rospy
import math
import numpy as np
import datetime
import time
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import mpl_toolkits.mplot3d


class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        # NOTE good params:
        # NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 5.0  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 2.5  # [rad/s]

        self.max_accel = 1.0  # [m/ss]
        self.max_dyawrate = 3.2  # [rad/ss]

        self.v_reso = 0.001  # [m/s]
        self.yawrate_reso = 0.16  # [rad/s]

        self.dt = 0.1  # [s]
        self.predict_time = 0.2  # [s]

        self.to_goal_cost_gain = 2.4  # lower = detour
        # self.angle_gain = 1000
        self.speed_cost_gain = 0.1  # lower = faster
        self.obs_cost_gain = 3.2  # lower z= fearless

        self.robot_radius = 0.2  # [m]
        self.x = -20.0
        self.y = 3.0
        self.th = 0.0
        self.goalX = -2
        self.goalY = 3
        self.startX = -20.0
        self.startY = 3.0
        self.acc_time = self.max_speed / self.max_accel
        self.acc_distance_quad = 0.5 * self.max_accel * self.acc_time ** 2

        self.current_v = 0.0
        self.current_w = 0.0
        self.r = rospy.Rate(10)

    # Callback for Odometry
    def assignOdomCoords(self, msg):
        # X- and Y- coords and pose of robot fed back into the robot config
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = \
            euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.th = theta

    # Callback for attaining goal co-ordinates from Rviz Publish Point
    def goalCB(self, msg):
        self.goalX = 10  # msg.linear.x
        self.goalY = 3  # msg.linear.y

    # Callback for cmd_vel
    def callcmd(self, msg):
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z


class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()

    # Custom range implementation to loop over LaserScan degrees with
    # a step and include the final degree
    def myRange(self, start, end, step):
        i = start
        while i < end:
            yield i
            i += step
        yield end

    # Callback for LaserScan
    def assignObs(self, msg, config):
        deg = len(msg.ranges)  # Number of degrees - varies in Sim vs real world
        self.obst = set()  # reset the obstacle set to only keep visible objects
        for angle in self.myRange(0, deg - 1, 1):
            distance = msg.ranges[angle]
            # only record obstacles that are within 4 metres away
            if (distance < 3):
                # angle of obstacle wrt robot
                # angle/2.844 is to normalise the 512 degrees in real world
                # for simulation in Gazebo, use angle/4.0
                # laser from 0 to 180
                # scanTheta = (angle/2.844 + deg*(-180.0/deg)+90.0) *math.pi/180.0
                scanTheta = -math.pi + angle * msg.angle_increment
                xb = distance * math.cos(scanTheta)
                yb = distance * math.sin(scanTheta)
                BP = np.array([xb, yb, 0, 1])

                ABT = np.array([[math.cos(config.th), -math.sin(config.th), 0, config.x],
                                [math.sin(config.th), math.cos(config.th), 0, config.y],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                AP = np.dot(ABT, BP)
                self.obst.add((AP[0], AP[1]))


# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]
    return x


# Determine the dynamic window from robot configurations
def calc_dynamic_window(x, config, moved):
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
    # v2 = t_to_v(t2)
    # v1 = t_to_v(t1)
    if moved <= config.acc_distance_quad:
        v = x[3] + config.max_accel * config.dt
    elif math.sqrt((config.goalX - x[0]) ** 2 + (config.goalY - x[1]) ** 2) <= 3:
        v = x[3] - config.max_accel * config.dt
    else:
        v = x[3]
    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [v,
          v,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
    dw = [min(v, Vs[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    # print(math.sqrt((config.goalX-x[0])**2 + (config.goalY-x[1])**2),config.acc_distance_quad)
    print(v)
    return dw


def quadratic_window(x, config):
    def v_to_t(v):
        if v <= config.max_speed / 2:
            t = math.sqrt(2 * v * config.acc_time / config.max_accel)
        elif config.max_speed / 2 < v <= config.max_speed:
            t = 2 * config.acc_time - math.sqrt((config.max_speed - v) * (2 * config.acc_time / config.max_accel))
        else:
            t = 2 * config.acc_time
        return t

    def t_to_v(t):
        if t <= config.max_speed / config.max_accel:
            v = config.max_accel * t ** 2 / (2 * config.acc_time)
        elif config.max_speed / config.max_accel < t <= 2 * config.max_speed / config.max_accel:
            v = config.max_speed - (0.5 * config.max_accel / config.acc_time) * (t - 2 * config.acc_time) * (
                        t - 2 * config.acc_time)
        elif t <= 0:
            v = 0
        else:
            v = config.max_speed
        return v

    t = v_to_t(x[3])
    t1 = t - config.dt
    t2 = t + config.dt
    v1 = t_to_v(t1)
    v2 = t_to_v(t2)
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

    # print(dw)

    return dw


def sigmoid_window(x, config):
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
    v1 = t_to_v(t2)
    v2 = t_to_v(t2)
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

    # print(dw)

    return dw


# Calculate a trajectory sampled across a prediction time
def calc_trajectory(xinit, v, w, config):
    # if v != 0:
    #	config.predict_time = 0.5/v
    # print(config.predict_time)
    x = np.array(xinit)
    traj = np.array(x)  # many motion models stored per trajectory
    time = 0
    # print(config.predict_time)
    while time <= config.predict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, w], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt  # next sample
    # print(traj)

    return traj


# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):
    xinit = x[:]
    min_cost = 10000.0
    max_cost = 0
    min_u = u
    max_u = u
    # min_u[0] = 0.0
    # print(xinit)
    # evaluate all trajectory with sampled input in dynamic window
    # print(np.arange(dw[0], dw[1]+config.v_reso, config.v_reso).shape)

    for w in np.arange(dw[1], dw[2], config.yawrate_reso):
        v = dw[0]
        traj = calc_trajectory(xinit, v, w, config)
        # calc costs with weighted gains
        to_goal_cost = calc_to_goal_cost(traj, config)
        # speed_cost = config.speed_cost_gain * (config.max_speed - traj[-1, 3])
        speed_cost2 = traj[-1, 3] / config.max_speed
        # ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain
        ob_cost2 = calc_obstacle_cost2(traj, ob, config)
        angle_cost = calc_angle_cost(traj, config, x)
        # final_cost = to_goal_cost + ob_cost + speed_cost
        final_cost2 = 0.2 * angle_cost + 2.0 * ob_cost2 + 0.5 * speed_cost2 + (1 / to_goal_cost)
        # search minimum trajectory
        # if min_cost >= final_cost2:
        #	min_cost = final_cost
        #	min_u = [v, w]
        if final_cost2 >= max_cost:
            max_cost = final_cost2
            max_u = [v, w]

    # return min_u
    return max_u


# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 2
    minr = float("inf")

    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(0, len(traj[:, 1])):
        for i in ob.copy():
            ox = i[0]
            oy = i[1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx ** 2 + dy ** 2)

            if r <= config.robot_radius:
                return float("Inf")  # collision

            if r <= minr:
                minr = r
    # print(dx,dy)
    # print(ox,oy)
    return 1.0 / minr


def calc_obstacle_cost2(traj, ob, config):
    minr = 1000

    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(0, len(traj[:, 1])):
        for i in ob.copy():
            ox = i[0]
            oy = i[1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx ** 2 + dy ** 2)
            if r <= minr:
                minr = r

    if minr <= config.robot_radius:
        cost = -10  # collision
    elif config.robot_radius < minr <= 2.0:
        cost = 0.556 * minr - 0.111
    else:
        cost = 1
    # print(minr)
    return cost


# Calculate goal cost via Pythagorean distance to robot
def calc_to_goal_cost(traj, config):
    # If-Statements to determine negative vs positive goal/trajectory position
    # traj[-1,0] is the last predicted X coord position on the trajectory
    """
    if (config.goalX >= 0 and traj[-1,0] < 0):
        dx = config.goalX - traj[-1,0]
    elif (config.goalX < 0 and traj[-1,0] >= 0):
        dx = traj[-1,0] - config.goalX
    else:
        dx = abs(config.goalX - traj[-1,0])
    # traj[-1,1] is the last predicted Y coord position on the trajectory
    if (config.goalY >= 0 and traj[-1,1] < 0):
        dy = config.goalY - traj[-1,1]
    elif (config.goalY < 0 and traj[-1,1] >= 0):
        dy = traj[-1,1] - config.goalY
    else:
        dy = abs(config.goalY - traj[-1,1])
    """
    cost = math.sqrt((config.goalX - traj[-1, 0]) ** 2 + (config.goalY - traj[-1, 1]) ** 2)
    return cost


def calc_angle_cost(traj, config, x):
    # If-Statements to determine negative vs positive goal/trajectory position
    # traj[-1,0] is the last predicted X coord position on the trajectory
    # angle_start = math.atan2(1-x[1],-1-x[0])-x[2]
    angle_end = math.atan2(config.goalY - traj[-1, 1], config.goalX - traj[-1, 0]) - traj[-1, 2]
    # print(angle_start,angle_end)
    cost = 1 - abs(angle_end / math.pi)

    return cost


# Begin DWA calculations
def dwa_control(x, u, config, ob, moved):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config, moved)
    # dw = quadratic_window(x, config)
    # dw = sigmoid_window(x, config)
    # print(dw)
    u = calc_final_input(x, u, dw, config, ob)

    return u


# Determine whether the robot has reached its goal
def atGoal(config, x):
    # check at goal
    if math.sqrt((x[0] - config.goalX) ** 2 + (x[1] - config.goalY) ** 2) <= 0.5:
        return True
    return False


def main():
    print(__file__ + " start!!")
    # robot specification
    config = Config()
    # position of obstacles
    obs = Obstacles()
    subOdom = rospy.Subscriber("base_pose_ground_truth", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber("/base_scan", LaserScan, obs.assignObs, config)
    subGoal = rospy.Subscriber("robot0/goal", Twist, config.goalCB)
    subCmd = rospy.Subscriber("/cmd_vel", Twist, config.callcmd)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    speed = Twist()
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    # x = np.array([config.x, config.y, config.th, config.current_v, config.current_w])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])
    odomx = [-20]
    odomy = [3]
    moved = []
    timelist = []
    tl = []
    xl = []
    xl2 = []
    al = [0]
    yl = [0]
    xs = [-20]
    ys = [3]
    moved = 0
    # runs until terminated externally
    time_start = datetime.datetime.now()
    while not rospy.is_shutdown():
        x = np.array([config.x, config.y, config.th, config.current_v, config.current_w])
        # t = datetime.datetime.now()
        # tl.append(t)
        # if len(tl) >=2:
        #	moved.append((tl[-1]-tl[-2]).total_seconds()*x[3])
        xs.append(config.x)
        ys.append(config.y)
        moved = moved + math.sqrt((xs[-1] - xs[-2]) ** 2 + (ys[-1] - ys[-2]) ** 2)
        if (atGoal(config, x) == False):
            u = dwa_control(x, u, config, obs.obst, moved)
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
        else:
            # if at goal then stay there until new goal published
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            break
        # print(yl)
        yl.append(x[3])
        al.append((yl[-1] - yl[-2]) / 0.1)
        pub.publish(speed)
        config.r.sleep()
    # odomx.append(config.x)
    # odomy.append(config.y)
    # moved.append(math.sqrt((odomx[-1]-odomx[-2])**2 + (odomy[-1]-odomy[-2])**2))
    # print(config.x)
    # print(odomx)
    # print(sum(moved))
    # time_end = datetime.datetime.now()
    # time_space = time_end-time_start
    # timelist.append(time_space.total_seconds())

    xl = range(len(yl))
    xl2 = range(len(al))

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(xl, yl, "black", linewidth=2, label='Velocity')
    ax2 = ax.twinx()
    ax2.plot(xl2, al, "red", linewidth=1, label='Acceleration')
    ax.set_xlabel('Frame')
    ax.set_ylabel('Velocity[m/s]')
    ax2.set_ylabel('Acceleration[m/s^2]')
    ax.set_ylim(0, 6.0)
    ax2.set_ylim(-2, 4)
    plt.grid(axis='y', linestyle='--')
    handles1, labels1 = ax.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    plt.legend(handles1 + handles2, labels1 + labels2, loc='upper right')
    plt.show()

    """
    yl.append(0)	
    ax = plt.subplot(projection = "3d")
    ax.scatter(xs,ys,yl,"red,",linestyle="--")
    ax.scatter(xs,ys,0)
    #xs = np.asarray(xs,dtype='float64')
    #ys = np.asarray(ys,dtype='float64')
    #ax.plot(xs,ys,"blue","-")
    plt.show()
    """


# print(sum(time)/len(time))
# with open("/home/z_lin/Desktop/ROS-Dynamic-Window-Approach-master/dwa_time.txt","a+")as f:
# with open("/home/z_lin/Desktop/ROS-Dynamic-Window-Approach-master/quad_moved_v_w.txt","a+")as f:
#   f.write(str(sum(moved))+","+str(speed.linear.x)+","+str(speed.angular.z)+"\n")
#   f.close()
# with open("/home/z_lin/Desktop/ROS-Dynamic-Window-Approach-master/quad_s_v_w_time_2.5ms.csv","a+")as f:
#    f.write(str(sum(moved))+","+str(speed.linear.x)+","+str(speed.angular.z)+","+str(timelist[-1])+"\n")
#    f.close()

if __name__ == '__main__':
    rospy.init_node('dwa')
    main()
