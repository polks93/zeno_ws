#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import yaml 
def start_callback(msg):
    global msg_recived
    rospy.loginfo("Start message received")
    msg_recived = True

def load():
    global msg_recived
    msg_recived = False 
    rospy.init_node('load')
    rospy.loginfo("Load node started")
    rate = rospy.Rate(1)
    rospy.Subscriber('/start', String, start_callback)
    while not rospy.is_shutdown():
        if msg_recived:
            msg_recived = False
            with open("/home/paolo/catkin_ws/src/models/params/Pmin.yaml", 'r') as file:
                Pmin = yaml.safe_load(file)
            rospy.loginfo("Pmin: %s", Pmin)
        rospy.loginfo("Load node running")
        rate.sleep()




if __name__ == '__main__':
    try:
        load()
    except rospy.ROSInterruptException:
        pass