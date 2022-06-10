#!/usr/bin/python

import rospy
import sys
from practice2.srv import add_two_int


def client1(a, b):
    rospy.wait_for_service("service1")
    srv_client = rospy.ServiceProxy("service1", add_two_int)
    res = srv_client(a, b)
    return res.c


if __name__ == "__main__":
    rospy.init_node('client', anonymous=True)

    if len(sys.argv) != 3:
        print("Wrong number of arguments.")
        sys.exit(1)

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    res = client1(a, b)
    print("{} + {} = {}".format(a, b, res))
