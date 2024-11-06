#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from joystick_command.msg import Rel_error_joystick


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

class ZenoNode():

    def __init__(self):
        rospy.init_node('zeno')
        self.rate = rospy.Rate(30)

        # Import parametri
        x0      = rospy.get_param('/zeno/start_pose/x', 0.0)
        y0      = rospy.get_param('/zeno/start_pose/y', 0.0)
        yaw0    = np.deg2rad(rospy.get_param('/zeno/start_pose/yaw', 0.0))

        # Limiti di velocita``
        self.v_surge_max    = rospy.get_param('/zeno/vel_max/surge', 0.2)               # m/s
        self.v_sway_max     = rospy.get_param('/zeno/vel_max/sway', 0.2)                # m/s
        self.omega_max      = np.deg2rad(rospy.get_param('/zeno/vel_max/omega', 10.0))    # rad/s   

        # Inizializzazione delle variabili di stato
        self.x          = x0
        self.y          = y0
        self.yaw        = yaw0
        self.v_surge    = 0.0
        self.v_sway     = 0.0
        self.omega      = 0.0

        # Accelerazioni iniziali
        self.a_surge    = 0.0
        self.a_sway     = 0.0
        self.alpha      = 0.0

        # Init comandi
        self.yaw_des        = self.yaw
        self.v_surge_des    = 0.0
        self.v_sway_des     = 0.0

        # Import parametri del controllore
        K_yaw   = rospy.get_param('/zeno/P_controller/yaw', 20.0)
        K_omega = rospy.get_param('/zeno/P_controller/omega', 50.0)
        K_surge = rospy.get_param('/zeno/P_controller/surge', 0.25)
        K_sway  = rospy.get_param('/zeno/P_controller/sway', 0.25)
        self.K  = {'yaw': K_yaw, 'omega': K_omega, 'v_surge': K_surge, 'v_sway': K_sway}	
        
        # Sottoscrizione ai comandi di velocita`
        rospy.Subscriber("/relative_error", Rel_error_joystick, self.cmd_callback)

        # Pubblicazione dell'odometria e tf
        self.pub_odom           = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.odom_broadcaster   = tf.TransformBroadcaster()

    def cmd_callback(self, msg):
        # type : (Rel_error_joystick) -> None
        
        self.yaw_des        = np.deg2rad(msg.error_yaw) + self.yaw
        self.v_surge_des    = np.clip(msg.error_surge_speed, -self.v_surge_max, self.v_surge_max)
        self.v_sway_des     = np.clip(msg.error_sway_speed, -self.v_sway_max, self.v_sway_max)
    

    def update_pose(self, dt):
        # type: (float) -> None

        yaw_error      = wrapToPi(self.yaw_des - self.yaw)
        v_surge_error  = self.v_surge_des - self.v_surge
        v_sway_error   = self.v_sway_des - self.v_sway

        # Calcolo accelerazioni
        alpha       = self.K['yaw'] * yaw_error - self.K['omega'] * self.omega
        a_surge     = self.K['v_surge'] * v_surge_error
        a_sway      = self.K['v_sway'] * v_sway_error

        self.omega      = np.clip(self.omega + alpha / 100 * dt, -self.omega_max, self.omega_max)
        self.v_surge    = np.clip(self.v_surge + a_surge * dt, -self.v_surge_max, self.v_surge_max)
        self.v_sway     = np.clip(self.v_sway + a_sway * dt, -self.v_sway_max, self.v_sway_max)

        self.yaw = wrapToPi(self.yaw + self.omega * dt)
        self.x  = self.x + (self.v_surge * np.cos(self.yaw) - self.v_sway * np.sin(self.yaw)) * dt
        self.y  = self.y + (self.v_surge * np.sin(self.yaw) + self.v_sway * np.cos(self.yaw)) * dt

    
    def publish_odometry(self):
        """ Pubblica l'odometria del robot """
        odom = Odometry()
        odom.header.stamp       = rospy.Time.now()
        odom.header.frame_id    = "odom"
        odom.child_frame_id     = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.v_surge
        odom.twist.twist.linear.y = self.v_sway
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.omega

        self.pub_odom.publish(odom)

    def publish_tf(self):
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.yaw),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

    def run(self):
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            self.update_pose(dt)
            self.publish_odometry()
            self.publish_tf()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        zeno = ZenoNode()
        zeno.run()
    except rospy.ROSInterruptException:
        pass

