#!/usr/bin/python

import rospy
from move_base_msgs.msg import  MoveBaseGoal

# file is saved at directory you execute rosrun


def save_pose(goal):
    with open('goals.txt', 'a') as file:
        file.write(goal)


def callback_goal(data):
    pose_x = round(data.target_pose.pose.position.x, 3)
    pose_y = round(data.target_pose.pose.position.y, 3)
    pose_z = round(data.target_pose.pose.position.z, 3)
    #ori_z = round(data.target_pose.orientation.position.z, 3)
    #ori_w = round(data.target_pose.orientation.position.w, 3)

    goal = "[{},{},{}],".format(pose_x, pose_y, pose_z)

    save_pose(goal)
    print("{} was added to goals.txt".format(goal))

    # uncomment if you want to receive only one message and shutdown
    # rospy.wait_for_message('/move_base/goal', MoveBaseGoal)
    # rospy.signal_shutdown()


def listener():
    rospy.init_node('get_goals', anonymous=True)
    rospy.Subscriber('/move_base/goal', MoveBaseGoal, callback_goal)
    rospy.spin()


if __name__ == "__main__":
    listener()
