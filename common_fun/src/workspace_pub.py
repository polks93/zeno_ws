#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def rectangle_marker():
    rospy.init_node('rectangle_marker_node')
    marker_pub = rospy.Publisher('rectangle_marker', Marker, queue_size=10)

    # Import parametri globali
    xmin = rospy.get_param("/workspace/x_min", 0.0)
    ymin = rospy.get_param("/workspace/y_min", 0.0)
    xmax = rospy.get_param("/workspace/x_max", 10.0)
    ymax = rospy.get_param("/workspace/y_max", 10.0)

    # Crea un Marker
    marker = Marker()
    marker.header.frame_id = "map"  # Posso usare anche un altro frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "rectangle"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    # Definisco i vertici del rettangolo basati sui parametri xmin, ymin, xmax, ymax
    points = [
        Point(xmin, ymin, 0),
        Point(xmax, ymin, 0),
        Point(xmax, ymax, 0),
        Point(xmin, ymax, 0),
        Point(xmin, ymin, 0)] 
    marker.points = points

    # Colore e dimensioni del marker
    marker.scale.x = 0.05  # spessore della linea
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Loop per pubblicare il marker
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        rectangle_marker()
    except rospy.ROSInterruptException:
        pass