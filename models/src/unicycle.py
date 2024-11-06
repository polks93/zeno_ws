#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

class UnicycleNode():
    def __init__(self):
        rospy.init_node('unicycle_node')
        self.rate = rospy.Rate(30)

        x0 = rospy.get_param('~x', 0)
        y0= rospy.get_param('~y', 0)
        theta0 = np.deg2rad(rospy.get_param('~theta', 0))

        self.x = x0
        self.y = y0
        self.theta = theta0
        self.omega = 0.0
        self.v = 0.0

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.odom_broadcaster = tf.TransformBroadcaster()

    def cmd_vel_callback(self, msg):
        # type : (Twist) -> None
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update_pose(self, dt):
        # type: (float) -> None
        """ Aggiorna la posizione del robot in base ai comandi ricevuti """
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta = self.wrapToPi(self.theta + self.omega * dt)

    def wrapToPi(self, angle):
        # type: (float) -> float
        """
        Converte l'angolo di input nell'intervallo [-pi, pi].
        Parametri:
            angle (float): L'angolo in radianti da convertire.
        Ritorna:
            float: L'angolo convertito nell'intervallo [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def publish_odometry(self):
        """ Pubblica l'odometria del robot """
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.z = self.omega

        self.pub_odom.publish(odom)

    def publish_tf(self):
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

    def run(self):
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            curr_time = rospy.Time.now()
            dt = (curr_time - last_time).to_sec()

            self.update_pose(dt)

            self.publish_odometry()
            self.publish_tf()

            last_time = curr_time
            self.rate.sleep()

if __name__ == '__main__':
    try:
        unicycle_node = UnicycleNode()
        unicycle_node.run()
    except rospy.ROSInterruptException:
        pass