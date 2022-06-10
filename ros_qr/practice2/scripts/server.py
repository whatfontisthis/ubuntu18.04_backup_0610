#!/usr/bin/python

import rospy
from practice2.srv import add_two_int, add_two_intResponse


def callback(req):
    return add_two_intResponse(req.a + req.b)


def server1():
    rospy.init_node("server1")
    srv_server = rospy.Service("service1", add_two_int, callback)
    print("Server Online. Waiting for request.")
    rospy.spin()


if __name__ == "__main__":
    server1()
