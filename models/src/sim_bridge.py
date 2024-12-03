#!/usr/bin/env python

import rospy
import tf
import numpy as np
import yaml
from marta_msgs.msg         import NavStatus
from nav_msgs.msg           import Odometry
from std_msgs.msg           import String
from joystick_command.msg   import Rel_error_joystick
from geodesy.utm            import fromLatLong
 
class SimBridge():
    """ Nodo ROS che si interfaccia con il simulatore di ZENO:
        - Riceve i dati di riferimento in latitudine e longitudine e pubblica l'odometria in terna ENU (East-North-Up).
        - Riceve gli errori relativi in terna ENU e li pubblica in terna NED.
        - Pubblica la trasformazione odom -> base_link.
    """
    def __init__(self):
        rospy.init_node('sim_bridge')
        self.rate = rospy.Rate(30)
        self.start              = False
        self.odom_recived       = False
        self.odom_gps_recived   = False
        self.navstatus_origin   = None
        # Publisher dell'odometria
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster  = tf.TransformBroadcaster()
        self.pub_rel_error = rospy.Publisher('/relative_error', Rel_error_joystick, queue_size=10)

        # Subscriber dei dati di riferimento
        rospy.Subscriber('/nbv/relative_error', Rel_error_joystick, self.rel_error_callback)
        rospy.Subscriber('/nav_status', NavStatus, self.nav_status_callback)
        rospy.Subscriber('/start_bridge', String, self.start_callback)

        with open("/home/paolo/catkin_ws/src/models/params/laghetti3/P1.yaml", 'r') as file:
            Pmin = yaml.safe_load(file)
            lat_min = Pmin['latitude']
            lon_min = Pmin['longitude']
            self.Pmin = fromLatLong(lat_min, lon_min)

        with open("/home/paolo/catkin_ws/src/models/params/laghetti3/P2.yaml", 'r') as file:
            Pmax = yaml.safe_load(file)
            lat_max = Pmax['latitude']
            lon_max = Pmax['longitude']
            self.Pmax = fromLatLong(lat_max, lon_max)

    def start_callback(self, msg):
        self.start = True

    def rel_error_callback(self, msg):
        """
        Callback per la gestione degli errori relativi.
        Questa funzione prende gli errori in terna ENU (East-North-Up) dal messaggio ricevuto
        e li pubblica in terna NED (North-East-Down) su /relative_error.
        Args:
            msg: Messaggio contenente gli errori in terna ENU.
        """
        if not self.start:
            rospy.logwarn("SimBridge: received relative error before start")
            return
        
        ned_error = Rel_error_joystick()
    
        ned_error.error_yaw             = - msg.error_yaw
        ned_error.error_surge_speed     = msg.error_surge_speed
        ned_error.error_sway_speed      = - msg.error_sway_speed

        self.pub_rel_error.publish(ned_error)

    def nav_status_callback(self, msg):
        
        """
        Callback associata al topic /nav_status.
        - Quando viene ricevuto il primo messaggio, salva la posizione iniziale che verra` utilizzata come origine delle terne ENU e NED.
        - Converte latitudine e longitudine in coordinate NED.
        - Converte le velocita` in terna NED (body)
        - Infine converte posa in terna ENU e twist in terna ENU (body) e pubblica l'odometria.
        """
        if not self.start:
            return
        
        lat = msg.position.latitude
        lon = msg.position.longitude

        # Salva la posizione iniziale da usare come origine e carica i confini del workspace
        if self.navstatus_origin is None:
            self.navstatus_origin = fromLatLong(lat, lon)

            # Calcola i confini del workspace
            ned_x1 = self.Pmin.northing - self.navstatus_origin.northing
            ned_y1 = self.Pmin.easting - self.navstatus_origin.easting
            enu_x1 = ned_x1
            enu_y1 = - ned_y1
            ned_x2 = self.Pmax.northing - self.navstatus_origin.northing
            ned_y2 = self.Pmax.easting - self.navstatus_origin.easting
            enu_x2 = ned_x2
            enu_y2 = - ned_y2

            xmin = min(enu_x1, enu_x2)
            ymin = min(enu_y1, enu_y2)
            xmax = max(enu_x1, enu_x2)
            ymax = max(enu_y1, enu_y2)
            rospy.loginfo("xmin: %s, ymin: %s, xmax: %s, ymax: %s", xmin, ymin, xmax, ymax)

            rospy.set_param('/workspace/x_min', xmin)
            rospy.set_param('/workspace/y_min', ymin)
            rospy.set_param('/workspace/x_max', xmax)
            rospy.set_param('/workspace/y_max', ymax)
            return
        
        self.odom_recived = True 

        # Converte latitudine e longitudine in coordinate NED
        curr_position   = fromLatLong(lat, lon)
        ned_x           = curr_position.northing - self.navstatus_origin.northing
        ned_y           = curr_position.easting - self.navstatus_origin.easting
        ned_yaw         = msg.orientation.yaw

        # Converto le velocita` in terna NED (body)
        ned_v_surge     = msg.ned_speed.x * np.cos(ned_yaw) + msg.ned_speed.y * np.sin(ned_yaw)
        ned_v_sway      = - msg.ned_speed.x * np.sin(ned_yaw) + msg.ned_speed.y * np.cos(ned_yaw)
        ned_omega       = msg.omega_body.z
        
        # Converto la posa in terna ENU
        self.x          = ned_x
        self.y          = - ned_y
        self.yaw        = - ned_yaw

        # Converto il twist in termina ENU (body)
        self.v_surge    = ned_v_surge
        self.v_sway     = - ned_v_sway
        self.omega      = - ned_omega

        # Pubblica l'odometria
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
            if self.start:
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
