#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
# from cw1.srv import MyMap
# import typing

# world map stores a numpy array and handles x, y addressing within it to make it intuitive to me


class Map:
    def __init__(self, size_x, size_y):
        # a numpy array to store the world in
        self.occupancy_grid = np.ndarray((size_x, size_y), dtype=int)
        if (size_x % 2) == 0:
            size_x += 1
        if (size_y % 2) == 0:
            size_y += 1
        grid_sqr_len = 1.0  # grid squares have length 1x1 m
        idx_per_sqr = 5  # occupancy grid indexes per square

        self.occupancy_grid.fill(0)
        self.size_x = size_x
        self.max_x = int(size_x/2)
        self.min_x = -int(size_x/2)
        self.size_y = size_y
        self.max_y = int(size_y/2)
        self.min_y = -int(size_y/2)
        # distance in m represented by each grid index
        self.idx_dist = grid_sqr_len/idx_per_sqr

        # need a ROS OccupancyGrid nav msg to publish
        self.msg = OccupancyGrid()

        self.pub = rospy.Publisher("map", OccupancyGrid, queue_size=1)
    # end

    def getCoord(self, x_coord, y_coord):
        grid_x = int(self.size_x/2 + x_coord)
        grid_y = int(self.size_y/2 + y_coord)

        return self.occupancy_grid[grid_y][grid_x]
    # end

    def setCoord(self, input, x_coord, y_coord):
        grid_x = int(self.size_x/2 + x_coord)
        grid_y = int(self.size_y/2 + y_coord)

        self.occupancy_grid[grid_x, grid_y] = input
    # end

    def update(self, _):  # blank argument is to avoid error sent when ros tries to give an extra param
        range_max = 2.0     # LiDAR's max accurate range is 2 metres... TODO pretty sure the scan data can tell me max range

        # assign robot and object coords in robot & world reference frames
        obj_from_robo = np.zeros((2, 1))    # TODO get from scan data
        robo_x = odometry.pose.pose.position.x
        robo_y = odometry.pose.pose.position.y
        robo_from_wrld = np.zeros((2, 1))   # TODO get this from odom topic
        robo_from_wrld[0, 0] = robo_x
        robo_from_wrld[1, 0] = robo_y

        # TODO implement loop
        # for every given range in scanner.ranges
        for r_idx in range(len(scanner.ranges)):
            # when a range is less than range_max...
            # record range r
            r = scanner.ranges[r_idx]

            if r <= range_max:
                # record beam angle θ
                θ = r_idx*scanner.angle_increment       # θ in rads

                # calculate object x = r*cos(θ)
                obj_from_robo[0, 0] = r*np.cos(θ)

                # calculate object y = r*sin(θ)
                obj_from_robo[1, 0] = r*np.sin(θ)

                # robot's rotation angle (in radians) in relation to the world
                φ = odometry.yaw

                rotate_from_wrld = np.array(
                    [[np.cos(φ), -np.sin(φ)],
                     [np.sin(φ), np.cos(φ)]])

                obj_from_wrld = robo_from_wrld + \
                    np.dot(rotate_from_wrld, obj_from_robo)

                obj_x = obj_from_wrld[0]
                obj_y = obj_from_wrld[1]

                # calculate map index in order to mark occupancy
                x_idx = int(obj_x/self.idx_dist)
                y_idx = int(obj_y/self.idx_dist)

                if (self.min_x < x_idx < self.max_x) and (self.min_y < y_idx < self.max_y):
                    self.setCoord(1, x_idx, y_idx)
                # end
            # end
        # end

        rospy.loginfo('publishing map...')
        self.pub.publish(self.msg)
    # end

    def display(self, _):
        robo_x = odometry.pose.pose.position.x
        robo_y = odometry.pose.pose.position.y
        self.setCoord(2, robo_x, robo_y)
        for row in range(0, self.occupancy_grid.shape[0]):
            map_row_str = ""
            for col in range(0, self.occupancy_grid.shape[1]):
                switch = {
                    0: "░░",
                    1: "▓▓",
                    2: "👾 "}
                map_row_str += switch.get(
                    self.occupancy_grid[row, col], "invalid value found")
            # end

            print(map_row_str)
        # end
        self.setCoord(0, robo_x, robo_y)
    # end

    def getCentreCoords(self):
        cntr_x = int(self.size_x/2)
        cntr_y = int(self.size_y/2)

        return cntr_y, cntr_x
    # end

# end


class OdomListener:
    def __init__(self):
        self.pose = Pose()
        self.twist = Twist()
        self.yaw = 0.0
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
    # end

    def odom_cb(self, msg):
        self.pose = msg.pose
        self.twist = msg.twist
        x_deg = msg.pose.pose.orientation.x
        y_deg = msg.pose.pose.orientation.y
        z_deg = msg.pose.pose.orientation.z
        w_deg = msg.pose.pose.orientation.w
        _, _, self.yaw = euler_from_quaternion([x_deg, y_deg, z_deg, w_deg])
    # end
# end


class ScanListener:
    def __init__(self):
        self.ranges = np.empty(720, dtype=float)
        self.angle_min = -np.inf
        self.angle_max = np.inf
        self.range_min = -np.inf
        self.range_max = np.inf
        self.angle_increment = 0.0
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)

    def scan_cb(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
    # end
# end


scanner = ScanListener()
odometry = OdomListener()
rospy.init_node("map_publisher")
rate = rospy.Rate(10)       # to publish at 10Hz
wrld_map = Map(31, 31)
rospy.loginfo('mapping node started...')

# TODO clear terminal
# rospy.Timer(rospy.Duration(1), wrld_map.update)
# rospy.Timer(rospy.Duration(1), wrld_map.display)
# TODO publish wrld_map.msg

# rospy.spin()
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    rate.sleep()
    wrld_map.update(None)
    wrld_map.display(None)
