#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scale_driver.scale_tcp_ip import scaleDriver


def main():

    rospy.init_node("scale_driver_node")

    rc_node = scaleDriver()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
