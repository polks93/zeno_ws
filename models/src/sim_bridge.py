#!/usr/bin/env python


import rospy
import tf
from marta_msgs.msg         import MotionReference
from nav_msgs.msg           import Odometry
from joystick_command.msg   import Rel_error_joystick

class SimBridge():
    """ Nodo ROS che si interfaccia con il simulatore di ZENO:
        - Riceve i dati di riferimento in terna NED (North-East-Down) e pubblica l'odometria in terna ENU (East-North-Up).
        - Riceve gli errori relativi in terna ENU e li pubblica in terna NED.
        - Pubblica la trasformazione odom -> base_link.
    """
    def __init__(self):
        rospy.init_node('sim_bridge')
        self.rate = rospy.Rate(30)
        self.odom_recived = False

        # Publisher dell'odometria
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster   = tf.TransformBroadcaster()
        self.pub_rel_error = rospy.Publisher('/relative_error', Rel_error_joystick, queue_size=10)

        # Subscriber dei dati di riferimento
        rospy.Subscriber('/model/vehicle/Reference', MotionReference, self.ref_callback)
        rospy.Subscriber('/nbv/relative_error', Rel_error_joystick, self.rel_error_callback)

    def ref_callback(self, msg):
        """
        Callback per la ricezione dei messaggi di posa e twist in terna NED.
        Aggiorna lo stato interno dell'oggetto con i dati ricevuti e pubblica un messaggio
        di odometria in terna ENU.
        Args:
            msg: Il messaggio di riferimento ricevuto, contenente le informazioni di posizione
                 e velocita` rispetto alla terna NED.
        Attributi aggiornati:
            self.odom_recived (bool): Flag che indica se e` stato ricevuto un messaggio di odometria.
            self.x (float): Coordinata nord.
            self.y (float): Coordinata est (negativa).
            self.yaw (float): Angolo di imbardata (negativo).
            self.v_surge (float): Velocita` di avanzamento lungo l'asse nord.
            self.v_sway (float): Velocita` di avanzamento lungo l'asse est (negativa).
            self.omega (float): Velocita` angolare lungo l'asse verticale (negativa).
        Pubblica:
            Odometry: Messaggio di odometria con le informazioni aggiornate di posizione e velocita` in terna ENU.
        """

        
        self.odom_recived = True
        north       = msg.pn_wrt_home.north
        east        = msg.pn_wrt_home.east
        yaw_down    = msg.rpy.yaw

        v_surge_north   = msg.vb.surge
        v_sway_east     = msg.vb.sway
        omega_down      = msg.wb.heave

        self.x           = north
        self.y           = - east
        self.yaw         = - yaw_down
        self.v_surge     = v_surge_north
        self.v_sway      = - v_sway_east
        self.omega       = - omega_down

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
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
        odom.twist.twist.angular.z = self.omega

        self.pub_odom.publish(odom)

    def rel_error_callback(self, msg):
        """
        Callback per la gestione degli errori relativi.
        Questa funzione prende gli errori in terna ENU (East-North-Up) dal messaggio ricevuto
        e li pubblica in terna NED (North-East-Down) su /relative_error.
        Args:
            msg: Messaggio contenente gli errori in terna ENU.
        """

        ned_error = Rel_error_joystick()
    
        ned_error.error_yaw             = - msg.error_yaw
        ned_error.error_surge_speed     = msg.error_surge_speed
        ned_error.error_sway_speed      = - msg.error_sway_speed

        self.pub_rel_error.publish(ned_error)

    def publish_tf(self):
        """
        Pubblica la trasformazione odom -> base_link.
        """

        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0), 
            tf.transformations.quaternion_from_euler(0, 0, self.yaw), 
            rospy.Time.now(), 
            'base_link',
            'odom'
        )

    def run(self):
        while not rospy.is_shutdown():
            if self.odom_recived:
                self.odom_recived = False
                self.publish_tf()
            self.rate.sleep()



if __name__ == '__main__':
    try:
        sim_bridge = SimBridge()
        sim_bridge.run()
    except rospy.ROSInterruptException:
        pass
