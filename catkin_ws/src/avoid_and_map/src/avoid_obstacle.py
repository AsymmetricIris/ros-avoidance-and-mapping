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


def avoid(service_request):
    req_vel = service_request.approach_velocity
    avoid_dist = service_request.approach_distance

    rospy.loginfo("Robot will now move at {} m/s until obstructed within {} metres...".format(
        req_vel, avoid_dist))
    vel = Twist()
    vel.linear.x = req_vel
    pub.publish(vel)
    # rospy.loginfo("Published the velocity command to /cmd_vel")

    # for some reason, the service request defined in "srv/Avoid2.srv" isn't recognised by ros
    # when I run this program and try to send a request message with target_x/_y params
    # bearing = getTrgtBearing(service_request.target_x,
    #                          service_request.target_y)
    target_x = 7.5
    target_y = 0.5

    # long as bot outside of target's square
    mode = 0
    while not ((target_x - 0.5 < odometry.position.x < target_x + 0.5) and (target_y - 0.5 < odometry.position.y < target_y + 0.5)):
        display_readout()

        # calculate target angle/distance from robo pos
        bearing = getTrgtBearing(target_x, target_y)
        trgt_angl = bearing["radians"]
        trgt_rng_idx = int(trgt_angl/scanner.angle_increment)
        path_dist = scanner.ranges[trgt_rng_idx]

        # calculate available space to robo's side
        side_angl = np.deg2rad(270)
        side_rng_idx = int(side_angl/scanner.angle_increment)
        side_dist = scanner.ranges[side_rng_idx]

        # calculate available space to pass objects
        detect_fov = 90
        clearance_angl = trgt_angl - np.deg2rad((detect_fov/2))
        clearance_rng_idx = int(clearance_angl/scanner.angle_increment)
        clearance_dist = scanner.ranges[clearance_rng_idx]

        # if mode == 0: line follow at the target angle
        if mode == 0:
            vel.linear.x = req_vel*0.9

            # when distance at target angle is less than avoid distance -> set mode = 1
            if path_dist <= avoid_dist:
                mode = 1
                approach_angl = trgt_angl
            # end

            # calc error between robo yaw and target yaw
            dif_yaw = trgt_angl - odometry.yaw
            magic_num = 1  # sorry not sorry!
        # end

        # when mode == 1: (scan distance at target angle is less than avoid distance) -> avoid
        if mode == 1:
            vel.linear.x = req_vel*0.5
            # calc rotational vel from error
            avoid_angl = approach_angl + np.deg2rad(90)
            dif_yaw = avoid_angl - odometry.yaw
            magic_num = 1  # sorry not sorry!

            if abs(dif_yaw) < 0.1:
                mode = 2
            # end
        # end

        # when mode 2 -> run impatient bug algo
        if mode == 2:
            vel.linear.x = req_vel
            if clearance_dist > avoid_dist*2:
                mode = 0
            # end

            # calc rotational vel from error
            dif_yaw = avoid_dist - side_dist
            magic_num = 0.05  # sorry not sorry!
        # end

        vel.angular.z = magic_num*dif_yaw
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
