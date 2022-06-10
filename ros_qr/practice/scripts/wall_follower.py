#!/usr/bin/env python

import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


'''
1. Wall Follower
2. On "ctrl+c" -> show pose
'''
 
np.set_printoptions(precision=2)

odom_x = odom_y = odom_z = amcl_x = amcl_y = amcl_z = 0

orbit = 0
laser_sensors = {'w': 0, 'nw': 0, 'n': 0, 'ne': 0, 'e': 0}
linear_vel = 0.3
angular_vel = 0.7

wall_distance_diagonal = 0.8
wall_distance_forward = 0.7
wall_distance_side = 0.7


inf = float('inf')


left = -1
going_left = -2
right = 1
going_right = 2


def calculate_lasers_range(data):
    '''Dynamic range intervals'''
    global laser_sensors
    laser_sensors['e'] = np.mean(np.setdiff1d(data.ranges[525:535], [0]))
    laser_sensors['ne'] = np.mean(np.setdiff1d(data.ranges[615:645], [0]))
    laser_sensors['n'] = np.mean(np.setdiff1d(
        data.ranges[:35] + data.ranges[-35:], [0]))
    laser_sensors['nw'] = np.mean(np.setdiff1d(data.ranges[75:105], [0]))
    laser_sensors['w'] = np.mean(np.setdiff1d(data.ranges[165:195], [0]))


def log_info():
    '''Initial orbit state'''

    global orbit, laser_sensors

    orbit_values = {-2: 'Going Left', -1: 'Left',
                    0: 'Undefined', 1: 'Right', 2: 'Going Right'}

    # rospy.loginfo("Orbit: %s, W : %s, NW: %s, N : %s, NE: %s, E : %s",
    #               orbit_values[orbit], laser_sensors['w'], laser_sensors['nw'], laser_sensors['n'], laser_sensors['ne'], laser_sensors['e'])


def create_velocity_message(turn_left, turn_right, forward):

    angular = 0
    linear = 0
    if (turn_left):
        angular += angular_vel
    if (turn_right):
        angular -= angular_vel
    if (forward):
        linear = linear_vel
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular

    return vel_msg


def publish_velocity_message(vel_msg):
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_pub.publish(vel_msg)


def laser_callback(data):

    global orbit, laser_sensors
    calculate_lasers_range(data)
    log_info()
    linear = 0
    angular = 0
    forward = False
    turn_left = False
    turn_right = False
    if (orbit == 0):
        if (laser_sensors['w'] < wall_distance_side):
            orbit = left
        elif (laser_sensors['e'] < wall_distance_side):
            orbit = right
        elif (laser_sensors['nw'] < wall_distance_diagonal):
            orbit = going_left
            turn_right = True
        elif (laser_sensors['ne'] < wall_distance_diagonal):
            orbit = going_right
            turn_left = True
        elif (laser_sensors['n'] < wall_distance_forward):
            orbit = going_left
            turn_right = True
        else:
            forward = True

    elif (orbit == going_left or orbit == going_right):
        if (laser_sensors['w'] < wall_distance_side):
            orbit = left
        elif (laser_sensors['e'] < wall_distance_side):
            orbit = right
        elif (orbit == going_left):
            turn_right = True
        elif (orbit == going_right):
            turn_left = True

    elif (orbit == left):
        if (laser_sensors['n'] > wall_distance_forward
                and (laser_sensors['w'] > wall_distance_side
                     or laser_sensors['e'] > wall_distance_side)):
            forward = True

        if (laser_sensors['w'] <= wall_distance_side
                and laser_sensors['e'] <= wall_distance_side):
            turn_right = True
        elif (laser_sensors['nw'] <= wall_distance_diagonal
              or laser_sensors['ne'] <= wall_distance_diagonal):
            turn_right = True
        else:
            if (laser_sensors['ne'] < wall_distance_diagonal
                    or laser_sensors['nw'] < wall_distance_diagonal
                    or laser_sensors['n'] < wall_distance_forward):
                turn_right = True
            else:
                turn_left = True
    elif (orbit == right):
        if (laser_sensors['n'] > wall_distance_forward
                and (laser_sensors['w'] > wall_distance_side
                     or laser_sensors['e'] > wall_distance_side)):
            forward = True
        if (laser_sensors['w'] <= wall_distance_side
                and laser_sensors['e'] <= wall_distance_side):
            turn_left = True
        elif (laser_sensors['nw'] <= wall_distance_diagonal
              or laser_sensors['ne'] <= wall_distance_diagonal):
            turn_left = True
        else:

            if (laser_sensors['ne'] < wall_distance_diagonal
                    or laser_sensors['nw'] < wall_distance_diagonal
                    or laser_sensors['n'] < wall_distance_forward):
                turn_left = True
            else:
                turn_right = True

    vel_msg = create_velocity_message(turn_left, turn_right, forward)
    publish_velocity_message(vel_msg)


def odom_callback(odom_pose):
    global odom_x, odom_y, odom_z
    odom_x = odom_pose.pose.pose.position.x
    odom_y = odom_pose.pose.pose.position.y
    odom_z = odom_pose.pose.pose.position.z

    #print("Odom: x = {}  y = {}  z = {}".format(odom_x, odom_y, odom_z))


def amcl_callback(amcl_pose):
    global amcl_x, amcl_y, amcl_z
    amcl_x = round(amcl_pose.pose.pose.position.x, 3)
    amcl_y = round(amcl_pose.pose.pose.position.y, 3)
    amcl_z = round(amcl_pose.pose.pose.position.z, 3)

    #print("Amcl: x = {}  y = {}  z = {}".format(amcl_x, amcl_y, amcl_z))


def clean_shutdown():

    print("\nSystem shutting down.")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    stop_vel = Twist()
    stop_vel.linear.x = 0
    stop_vel.linear.y = 0
    stop_vel.angular.z = 0

    while pub.get_num_connections() < 1:
        print("Waiting for connection to publisher...")
        time.sleep(1)

    pub.publish(stop_vel)
    print("Robot stopped.")

    show_odom_pose()
    show_amcl_pose()


def show_odom_pose():
    print("Odom: x = {}  y = {}".format(odom_x, odom_y))


def show_amcl_pose():
    print("Amcl: x = {}  y = {}".format(amcl_x, amcl_y))


def listener():
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    rospy.on_shutdown(clean_shutdown)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('wall_follower', anonymous=True)
    listener()
