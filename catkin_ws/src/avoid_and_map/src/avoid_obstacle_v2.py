#! /usr/bin/env python

import rospy
import os
import time
#import typing
import numpy as np
from geometry_msgs.msg import Twist, Pose
from cw1.srv import Avoid, AvoidResponse
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


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


class OdomListener:
    def __init__(self):
        self.pose = Pose()
        self.position = self.pose.position
        self.twist = Twist()
        self.yaw = 0.0
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
    # end

    def odom_cb(self, msg):
        self.pose = msg.pose
        self.position = msg.pose.pose.position
        self.twist = msg.twist
        x_deg = msg.pose.pose.orientation.x
        y_deg = msg.pose.pose.orientation.y
        z_deg = msg.pose.pose.orientation.z
        w_deg = msg.pose.pose.orientation.w
        _, _, self.yaw = euler_from_quaternion([x_deg, y_deg, z_deg, w_deg])
    # end
# end


def getTrgtBearing(trgt_x: float, trgt_y: float):
    trgt_bearing = {"dx": 0.0, "dy": 0.0, "distance": 0.0, "radians": 0.0}

    robo_x = odometry.position.x
    robo_y = odometry.position.y

    dx = trgt_x - robo_x
    trgt_bearing["dx"] = dx
    dy = trgt_y - robo_y
    trgt_bearing["dy"] = dy

    dist = np.sqrt(dx**2 + dy**2)
    trgt_bearing["distance"] = dist
    # get angle between current robo and target positions usig inverse cosinne rule
    # cosA = (b**2 + c**2 - a**2) / 2bc
    # a = dy
    # A = target yaw in radians
    trgt_bearing["radians"] = np.arccos((dx**2 + dist**2 - dy**2)/(2*dx*dist))

    if (dy < 0 < dx) or (dx < 0 < dy):
        trgt_bearing["radians"] *= -1
    # end

    return trgt_bearing
# end


def minRngInArc(arc_start_rads, arc_end_rads):

    trgt_angls = np.arange(arc_start_rads, arc_end_rads,
                           scanner.angle_increment)
    trgt_rng_indexes = [int(angl/scanner.angle_increment)
                        for angl in trgt_angls]
    path_dists = [scanner.ranges[index] for index in trgt_rng_indexes]
    path_dists = np.array(path_dists)

    min_rng = np.min(path_dists)
    return min_rng
# end


def avoid(service_request):
    max_linear_msec = 1
    max_angular_msec = 1
    req_vel = service_request.approach_velocity
    avoid_dist = service_request.approach_distance
    detect_fov = 90
    narrow_fov = 50

    rospy.loginfo("Robot will now move at {} m/s until obstructed within {} metres...".format(
        req_vel, avoid_dist))

    # for some reason, the service request defined in "srv/Avoid2.srv" isn't recognised by ros
    # when I run this program and try to send a request message with target_x/_y params
    # bearing = getTrgtBearing(service_request.target_x,
    #                          service_request.target_y)
    target_x = 7.5
    target_y = 0.5

    mode = 0  # set mode 0 : point at the target location and go forward
    # mode 1 : rotate orthogonal to an obstacle so that "lazy bug" algo can run
    # mode 2 : run "lazy bug" algo until there is space to return to mode 0

    # debug
    avoid_angl = 0

    # TODO - squish this logic into support functions with readable names
    # long as bot outside of target's square
    while not ((target_x - 0.5 < odometry.position.x < target_x + 0.5) and (target_y - 0.5 < odometry.position.y < target_y + 0.5)):
        display_readout()
        vel = Twist()
        vel.linear.x = req_vel
        vel.linear.z = 0

        # calculate target angle/distance from robo pos
        bearing = getTrgtBearing(target_x, target_y)
        trgt_angl = bearing["radians"]
        rltv_angl = trgt_angl - np.deg2rad(odometry.yaw)
        trgt_rng_idx = int(rltv_angl/scanner.angle_increment)
        path_dist = scanner.ranges[trgt_rng_idx]

        # calculate distances in arcs around the robot
        front_left_rng = minRngInArc(0, np.deg2rad(narrow_fov/2))
        front_right_rng = minRngInArc(np.deg2rad(
            360) - np.deg2rad(narrow_fov/2), np.deg2rad(360))
        left_rng = minRngInArc(np.deg2rad(
            90) - np.deg2rad(detect_fov/2), np.deg2rad(90) + np.deg2rad(detect_fov/2))
        right_rng = minRngInArc(np.deg2rad(
            270) - np.deg2rad(detect_fov/2), np.deg2rad(270) + np.deg2rad(detect_fov/2))

        # calculate available space to pass objects
        clearance_angl = trgt_angl
        clearance_rng_idx = int(clearance_angl/scanner.angle_increment)
        clearance_dist = minRngInArc(np.deg2rad(
            trgt_angl) - np.deg2rad(detect_fov/2), np.deg2rad(trgt_angl) + np.deg2rad(detect_fov/2))

        # calculate available space to robo's side
        side_rng_idx = int(np.deg2rad(270)/scanner.angle_increment)
        side_dist = scanner.ranges[side_rng_idx]

        fwd_correction = 0.2
        yaw_correction = 0.3
        # if left distance is less than 2m, add rotational rightward velocity (scaled by a correction factor)
        if (left_rng < avoid_dist):
            # print("block left")
            vel.angular.z += ((avoid_dist - left_rng) /
                              avoid_dist)*yaw_correction
        # end

        # if right distance is less than 2m, add rotational leftward velocity (scaled by a correction factor)
        if (right_rng < avoid_dist):
            # print("block right")
            vel.angular.z += ((avoid_dist - right_rng) /
                              avoid_dist)*yaw_correction
        # end

        # if front-left distance is less than 2m, + right vel, - fwd vel (both scaled by a correction factor)
        if (front_left_rng < 2):
            vel.linear.x -= (req_vel*(2 - front_left_rng)/2)*fwd_correction
            vel.angular.z -= ((2 - front_left_rng)/2)*yaw_correction
        # end

        # if front-right distance is less than 2m, + left vel, - fwd vel (both scaled by a correction factor)
        if (front_right_rng < 2):
            # print("obstacle fwd-right")
            vel.linear.x -= (req_vel*(2 - front_right_rng)/2)*fwd_correction
            vel.angular.z += ((2 - front_right_rng)/2)*yaw_correction
        # end

        if mode == 0:  # line follow towards destination
            magic_num = 0.5  # sorry not sorry!

            # calc error between robo yaw and target yaw
            vel.angular.z += (trgt_angl - odometry.yaw)*magic_num

            # when distance at target angle is less than avoid distance -> set mode = 1
            if path_dist <= avoid_dist:
                mode = 1
                approach_angl = trgt_angl
                avoid_angl = approach_angl + np.deg2rad(90)
            # end
        # end

        if mode == 1:  # prepare to run lazy bug
            magic_num = 1  # sorry not sorry!

            # calc rotational vel from error
            vel.linear.x = 0
            vel.angular.z += (avoid_angl - odometry.yaw)*magic_num

            if abs(avoid_angl - odometry.yaw) < 0.05:  # when robot faces orthogonal to obstacle
                mode = 2
            # end
        # end

        if mode == 2:  # run lazy bug algo
            magic_num = 1

            # calc rotational vel from error
            if side_dist > avoid_dist:
                vel.angular.z += (avoid_dist - side_dist)*magic_num
            # end

            if clearance_dist > avoid_dist*2:
                mode = 0
            # end
        # end

        print("f_l dist : {}".format(front_left_rng))
        print("f_r dist : {}".format(front_right_rng))
        print("l dist : {}".format(left_rng))
        print("r dist : {}".format(right_rng))
        print("trgt angle : {}".format(trgt_angl))
        print("approach angl idx : {}".format(clearance_rng_idx))
        print("approach dist : {}".format(clearance_dist))
        print("x vel : {}".format(vel.linear.x))
        print("z vel : {}".format(vel.angular.z))
        print("Avoid angl : {}".format(avoid_angl))
        print("Mode : {}".format(mode))

        # when forward velocity is less than 0, set forward velocity <- 0
        if vel.linear.x > max_linear_msec:
            vel.linear.x = max_linear_msec
        elif vel.linear.x < 0:
            vel.linear.x = 0
        # end

        # when angular velocity is outside min/max, set angular velocity <- min or max
        if vel.angular.z > max_angular_msec:
            vel.angular.z = max_angular_msec
        elif vel.angular.z < -max_angular_msec:
            vel.angular.z = -max_angular_msec
        # end

        pub.publish(vel)
    # end

    vel.linear.x = 0.0
    vel.angular.z = 0.0
    pub.publish(vel)

    rospy.loginfo("Robot stopped at: \n Position: x={} y={} \n Yaw: {} pi radians".format(
        odometry.position.x, odometry.position.y, (odometry.yaw/3.14)))
# end


def callback_function(service_request):
    service_response = AvoidResponse()

    req_vel = service_request.approach_velocity
    req_dist = service_request.approach_distance

    if req_vel > 0:
        avoid(service_request)

        service_response.response_message = 'Request complete.'
    else:
        service_response.response_message = "Improper request parameters should be 4 (properly formatted) floats"
    return service_response
# end


def display_readout():
    os.system('clear')
    print("X: {0:1.2f}, Y: {1:2.2f}, Yaw: {2:3.2f}".format(
        odometry.position.x, odometry.position.y, odometry.yaw))
# end


rospy.init_node('avoid_service_server')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
scanner = ScanListener()
odometry = OdomListener()
my_service = rospy.Service(
    '/cw1/Avoid', Avoid, callback_function)
rospy.loginfo('the avoid_service server is ready to be called...')
time.sleep(1)

rospy.spin()
