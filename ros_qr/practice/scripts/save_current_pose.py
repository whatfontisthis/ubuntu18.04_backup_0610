#!/usr/bin/python

import rospy
import sys
from nav_msgs.msg import Odometry  # odom pose

# file is saved at directory you execute rosrun


def save_pose(pose):
    with open('pose.txt', 'a') as file:
        file.write(pose)


def callback_odom(odom):
    pos_x = round(odom.pose.pose.position.x, 3)
    pos_y = round(odom.pose.pose.position.y, 3)
    pos_z = round(odom.pose.pose.position.z, 3)

    ori_x = round(odom.pose.pose.orientation.x, 3)
    ori_y = round(odom.pose.pose.orientation.y, 3)
    ori_z = round(odom.pose.pose.orientation.z, 3)
    ori_w = round(odom.pose.pose.orientation.w, 3)

    # pose = "Position: {}, {}, {}\tOrientation: {}, {}, {}".format(
    #     pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w)
    pose = "[{},{},{},{}],".format(pos_x, pos_y, ori_z, ori_w)

    save_pose(pose)
    print("{} was added to pose.txt".format(pose))
    rospy.wait_for_message('/odom', Odometry)
    # rospy.signal_shutdown()


def listener():
    rospy.init_node('get_pose', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.spin()


if __name__ == "__main__":
    listener()
