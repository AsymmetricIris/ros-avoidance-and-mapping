#! /usr/bin/env python
from math import atan2
from collections import deque
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
#import matplotlib.pyplot as plt
import numpy as np

#from srv_examples.srv import SetBool, SetBoolResponse
from srv_examples.srv import Approach, ApproachResponse


class robot:
    
    def __init__(self):
        self.angle_to_goal = 0
        self.theta = 0
        self.Distance = 0
        self.x = 0
        self.y = 0
        self.Approach_Velocity = 0.0
        self.X_pos = 0
        self.Y_pos = 0
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometer)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.my_service = rospy.Service('/move_service', Approach, self.callback_function)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel = Twist()
        self.loc = False
        self.path_loop = 0
        self.world_robot_map_shift = 15
        self.Map = np.stack(([], []))
        self.pathfinder = True
        self.pixel = [[1, 0], [0, 1], [-1, 0], [0, -1],[-1, -1], [-1, 1], [1, -1], [1, 1]]
        self.att_gain = 5.0
        self.repel_gain = 100.0
        self.Area_w = 10.0
        self.pot_X = np.array([])
        self.pot_Y = np.array([])


    def callback(self, msg):
        # print(msg.ranges[0])
        #global Distance
        self.Distance = np.asarray(msg.ranges)

        self.proximity()

        # self.mapping()

    def callback_function(self, service_request):
        #global lidar_angle

        self.service_response = ApproachResponse()
        self.Approach_Distance = service_request.approach_distance
        self.Approach_Velocity = service_request.approach_velocity
        self.X_pos = service_request.X
        self.Y_pos = service_request.Y
        print("Start")

        return self.service_response

    def odometer(self, msg):
        # print("odometer")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #print("odometer", self.x, self.y)
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def Total_potential_field(self, gx, gy, ox, oy, reso, rr, sx, sy):          # finds U(q)t  = U(q)att + U(q)rep   
        minx = min(min(ox), sx, gx) - self.Area_w / 2.0                        
        miny = min(min(oy), sy, gy) - self.Area_w / 2.0
        maxx = max(max(ox), sx, gx) + self.Area_w / 2.0
        maxy = max(max(oy), sy, gy) + self.Area_w / 2.0
        xw = int(round((maxx - minx) / reso))
        yw = int(round((maxy - miny) / reso))
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]
        for ix in range(xw):
            print("Calculating Total Potential field")
            x = ix * reso + minx
            for iy in range(yw):
                y = iy * reso + miny
                ug = self.attractive_potential(x, y, gx, gy)
                uo = self.repulsive_potential(x, y, ox, oy, rr)
                uf = ug + uo
                pmap[ix][iy] = uf

        return pmap, minx, miny

    def attractive_potential(self, x, y, gx, gy):                   # finds U(q)att 
        return 0.5 * self.att_gain * np.hypot(x - gx, y - gy)      

    def repulsive_potential(self, x, y, ox, oy, rr):                # finds U(q)rep
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calcating repulsive potential
        dq = np.hypot(x - ox[minid], y - oy[minid])

        if dq <= rr:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * self.repel_gain * (1.0 / dq - 1.0 / rr) ** 2
        else:
            return 0.0

    def potential_field(self, sx, sy, gx, gy, ox, oy, reso, rr):

        # calc potential field
        pmap, minx, miny = self.Total_potential_field(
            gx, gy, ox, oy, reso, rr, sx, sy)

        # search path
        d = np.hypot(sx - gx, sy - gy)
        ix = round((sx - minx) / reso)
        iy = round((sy - miny) / reso)
        gix = round((gx - minx) / reso)
        giy = round((gy - miny) / reso)

        rx, ry = [sx], [sy]
        motion = self.pixel
        previous_ids = deque()

        while d >= reso:
            print(d, reso)
            print("preparing...")
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    print("outside potential!")
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * reso + minx
            yp = iy * reso + miny
            d = np.hypot(gx - xp, gy - yp)
            rx.append(xp)
            ry.append(yp)

        print(rx, ry)
        print("Goal!!")

        return rx, ry

    def path_planing(self):                # plans the points which the robot has to move using potetial field    

        if (self.pathfinder == True) and (self.Approach_Velocity > 0.0):
            RX, RY, MX, MY = self.Lidar_Sub()
            self.pot_X, self.pot_Y = self.potential_field(self.x, self.y, self.X_pos, self.Y_pos, MX, MY, 1.0, 1.0)                  # reference : https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/PotentialFieldPlanning/potential_field_planning.py
            #self.mapping(self.pot_X, self.pot_Y, MX, MY, 'potential.png')
            self.pathfinder = False
        #path = np.asarray(path)
       
        print(self.pot_X ,self.pot_Y)
        print("length" , len(self.pot_Y))
        print("Loop ", self.path_loop)
        self.motion_planning(self.pot_X[self.path_loop],self.pot_Y[self.path_loop], len(self.pot_Y))

    def proximity(self):    # stops when unavoidable obstacle appear
        # self.Lidar_Sub()
        # for a in range(0,89):
        """if(abs(self.Distance[0] < 1.0) or (abs(self.Distance[90] < 1.0)) or (abs(self.Distance[630] < 1.0)) or (abs(self.Distance[45] < 1.0)) or (abs(self.Distance[675] < 1.0))):
            print("Proximity....")
            self.Stop_func()
        else:"""
        self.path_planing()

    def Stop_func(self):    # stops the robot
        print("stopping")
        print(" ")
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)

    def motion_planning(self, X_goal, Y_goal, path_distance):   # set the robot to move to the said position(x,y)
        location = Point()
        location.x = X_goal
        location.y = Y_goal

        Val_X = location.x - self.x
        Val_Y = location.y - self.y
        self.angle_to_goal = atan2(Val_Y, Val_X)
        print("2tan :", self.angle_to_goal)
        print("theta :", self.theta)
        print("curr loc : ", self.x, " : ", self.y)
        print("loc :", X_goal, " :  ", Y_goal)
        # self.proximity()
        if(round(self.x, ndigits=1) == round(location.x, ndigits=1)) and (round(self.y, ndigits=1) == round(location.y, ndigits=1)):

            print("position reached....")
            if (self.path_loop < path_distance):
                self.path_loop += 1
                if(self.path_loop >= path_distance):
                    self.Stop_func()
            elif(self.path_loop >= path_distance):
                self.path_loop = 0
                self.Stop_func()

        else:

            if (round(self.angle_to_goal, ndigits=1) != round(self.theta, ndigits=1)):
                print("turning")
                print(" ")
                #T = round(self.angle_to_goal,ndigits=1)*pi/180
                ang_vel = (self.Approach_Velocity*2) * \
                    (self.angle_to_goal-(self.theta))
                self.vel.linear.x = 0.0
                self.vel.angular.z = ang_vel
                print("angular velocity", ang_vel)
                self.pub.publish(self.vel)
            else:
                print("straight")
                print(" ")
                self.vel.linear.x = self.Approach_Velocity
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)

    def Lidar_Sub(self):
        ranges = self.Distance
        ranges[np.isinf(ranges)] = 12.0
        theta = (np.arange(720)*np.pi)/720
        RX, RY, MX, MY = self.obstacle_position(theta, ranges)
        return(RX, RY, MX, MY)

    def obstacle_position(self, angle, distance):
        A = distance * np.cos(angle)
        B = distance * np.sin(angle)
        X = (self.x + (A*np.cos(self.theta+angle)) - (B*np.sin(self.theta+angle)))
        Y = (self.y + (A*np.sin(self.theta+angle)) + (B*np.cos(self.theta+angle)))
        self.Map = np.stack((X, Y))
        A, B = self.robot_to_World(self.x, self.y)
        MX, MY = self.robot_to_World(self.Map[0], self.Map[1])
        self.mapping(A, B, MX, MY, 'map.png')
        return A, B, MX, MY

    def robot_to_World(self, X, Y):
        return X+self.world_robot_map_shift, Y+self.world_robot_map_shift

    def world_to_robot(self, X, Y):
        return self.world_robot_map_shift-X, self.world_robot_map_shift-Y

    def mapping(self, A, B, X, Y, name):
        plt.figure(figsize=(10, 10))
        plt.plot(A, B, marker="o", markersize=5, markeredgecolor="red",
                 markerfacecolor="green")  # Location of Robot
        plt.scatter(X, Y, s=1)
        # plt.show()
        plt.savefig(name)


if __name__ == '__main__':
    rospy.init_node('move_service_server')
    a = robot()

    rospy.loginfo('the move_service server is ready to be called...')

    rospy.spin()
