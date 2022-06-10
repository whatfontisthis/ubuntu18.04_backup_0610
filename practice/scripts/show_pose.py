#! /usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult

arrived = False


def callback_odom(odom_pose):
    odom_pose_x = round(odom_pose.pose.pose.position.x, 3)
    odom_pose_y = round(odom_pose.pose.pose.position.y, 3)
    odom_pose_z = round(odom_pose.pose.pose.position.z, 3)

    odom_ori_x = round(odom_pose.pose.pose.orientation.x, 3)
    odom_ori_y = round(odom_pose.pose.pose.orientation.y, 3)
    odom_ori_z = round(odom_pose.pose.pose.orientation.z, 3)
    odom_ori_w = round(odom_pose.pose.pose.orientation.w, 3)

    global arrived
    if arrived:
        print("Odom Pos: x={} y={} z={}".format(
            odom_pose_x, odom_pose_y, odom_pose_z))
        print("Odom Ori: x={} y={} z={} w={}".format(
            odom_ori_x, odom_ori_y, odom_ori_z, odom_ori_w))


def callback_amcl(amcl_pose):
    amcl_pose_x = round(amcl_pose.pose.pose.position.x, 3)
    amcl_pose_y = round(amcl_pose.pose.pose.position.y, 3)
    amcl_pose_z = round(amcl_pose.pose.pose.position.z, 3)

    amcl_ori_x = round(amcl_pose.pose.pose.orientation.x, 3)
    amcl_ori_y = round(amcl_pose.pose.pose.orientation.y, 3)
    amcl_ori_z = round(amcl_pose.pose.pose.orientation.z, 3)
    amcl_ori_w = round(amcl_pose.pose.pose.orientation.w, 3)

    global arrived
    if arrived:
        print("Odom Pos: x={} y={} z={}".format(
            amcl_pose_x, amcl_pose_y, amcl_pose_z))
        print("Odom Ori: x={} y={} z={} w={}".format(
            amcl_ori_x, amcl_ori_y, amcl_ori_z, amcl_ori_w))


def callback_result(result):
    # will return result data when action goal is achieved
    # use as tuple or use result.status.status==3 as conditional trigger
    global arrived
    arrived = result
    print("Status: {} Text: {}".format(
        result.status.status, result.status.text))


def listener():
    # initiate node
    rospy.init_node('show_pose', anonymous=True)

    # subscribe to odom, amcl, result
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback_amcl)
    rospy.Subscriber('move_base/result', MoveBaseActionResult, callback_result)

    rospy.spin()


if __name__ == "__main__":
    listener()
