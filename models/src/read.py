#!/usr/bin/env python

import rospy

def read():
    rospy.init_node('read')
    rospy.loginfo("Read node started")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        while not rospy.has_param('xmin') and not rospy.is_shutdown():
            rospy.loginfo("Read node waiting for parameter")
            rate.sleep()
        
        rospy.get_param('xmin', 0)
        rospy.loginfo("Parameter xmin: %s", rospy.get_param('xmin'))
        rate.sleep()

if __name__ == '__main__':
    try:
        read()
    except rospy.ROSInterruptException:
        pass