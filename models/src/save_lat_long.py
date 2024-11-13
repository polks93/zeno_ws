#!/usr/bin/env python

import rospy
from std_msgs.msg           import String
from marta_msgs.msg         import NavStatus
import yaml

def save_callback(msg):
    global save_coordinates
    save_coordinates = True

def nav_status_callback(msg):
    global save_coordinates
    global done
    if save_coordinates:
        lat = msg.position.latitude
        lon = msg.position.longitude
        coordinates = {'latitude': lat, 'longitude': lon}
        with open('coordinates.yaml', 'w') as file:
            yaml.dump(coordinates, file)
        save_coordinates = False
        rospy.loginfo("Coordinates saved")
        done = True

def save_coordinates():
    global done
    global save_coordinates
    save_coordinates = False
    done = False
    rospy.init_node('save_coordinates')
    rospy.loginfo("Save coordinates node started")
    rate = rospy.Rate(1)
    rospy.Subscriber('/save', String, save_callback)
    rospy.Subscriber('/nav_status', NavStatus, nav_status_callback)

    while not rospy.is_shutdown() and not done:
        rospy.loginfo("Waiting for save command")
        rate.sleep()


if __name__ == '__main__':
    try:
        save_coordinates()
    except rospy.ROSInterruptException:
        pass