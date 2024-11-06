#!/usr/bin/env python

import rospy
import numpy as np

from joystick_command.msg import Rel_error_joystick
from marta_msgs.msg import NavStatus
def wrapToPi(angle):
    # type: (float) -> float
    """
    Converte l'angolo di input nell'intervallo [-pi, pi].
    Parametri:
        angle (float): L'angolo in radianti da convertire.
    Ritorna:
        float: L'angolo convertito nell'intervallo [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def nav_status_callback(msg):
    global msg_recived
    global yaw

    msg_recived = True
    yaw = wrapToPi(msg.orientation.yaw)

def test_node():
    global msg_recived
    global yaw

    msg_recived = False
    rospy.init_node('test_node')
    rate = rospy.Rate(30)
    pub = rospy.Publisher("/relative_error", Rel_error_joystick, queue_size=1)
    rospy.Subscriber("/nav_status", NavStatus, nav_status_callback, queue_size=1)
    # yaw_des = np.deg2rad(20)

    msg = Rel_error_joystick()
    msg.error_yaw = 20
    # msg.error_surge_speed = 0.1
    # for i in range(5):
    #     pub.publish(msg)
    #     rate.sleep()
    rospy.sleep(1)
    done = False
    
    while not rospy.is_shutdown():
        if not done: 
            msg.error_yaw = 20
            pub.publish(msg)
            rate.sleep()
            done = True
        # if msg_recived:
        #     yaw_error = wrapToPi(yaw_des - yaw)
        #     msg_recived = False
        #     msg = Rel_error_joystick()
        #     msg.error_yaw = yaw_error
        #     pub.publish(msg)
        #     rate.sleep()
            

if __name__ == '__main__':
    try:
        test_node()
    except rospy.ROSInterruptException:
        pass
