#!/usr/bin/env python

import rospy
import numpy as np
import tf

from obstacle_simulation import ShipObstacle, lidar
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

class Lidar_sim():
    """
    Classe per simulare un sensore LiDAR in un ambiente ROS.
    Metodi:
    --------
    __init__():
        Inizializza il nodo ROS, i parametri del LiDAR, e i publisher/subscriber necessari.
    odom_callback(msg):
        Callback per il subscriber dell'odometria. Aggiorna la posa del robot.
    get_ranges():
        Calcola le distanze rilevate dal LiDAR rispetto agli ostacolo ShipObstacle.
    publish_lidar_data():
        Pubblica i dati del LiDAR come messaggio LaserScan.
    run():
        Esegue il ciclo principale del nodo ROS, pubblicando i dati del LiDAR a una frequenza specificata.
    """

    def __init__(self):
        rospy.init_node('lidar_sim')

        # Import parametri globali
        self.Hz             = rospy.get_param('/lidar/frequency', 30.0)
        n_beams             = rospy.get_param('/lidar/n_beams', 90)
        max_range           = rospy.get_param('/lidar/max_range', 5.0)
        self.min_range      = rospy.get_param('/lidar/min_range', 0.1)
        FoV_deg             = rospy.get_param('/lidar/FoV', 360.0)

        # ship_center         = rospy.get_param('/ship/center', (0,0))
        ship_scale_factor   = rospy.get_param('/ship/scale', 0.9)

        self.scan_time      = 1.0 / self.Hz
        self.rate           = rospy.Rate(self.Hz)

        self.lidar_params   = {'n_beams': n_beams, 'max_range': max_range, 'FoV': np.deg2rad(FoV_deg)}

        workspace_params = ["/workspace/x_min", "/workspace/y_min", "/workspace/x_max", "/workspace/y_max"]
        while not all(rospy.has_param(param) for param in workspace_params) and not rospy.is_shutdown():
            rospy.loginfo("Attendo limiti workspace ... ")
            self.rate.sleep()
            
        # Confini workspace  
        self.xmin                = rospy.get_param("/workspace/x_min", 0.0)
        self.ymin                = rospy.get_param("/workspace/y_min", 0.0)
        self.xmax                = rospy.get_param("/workspace/x_max", 10.0)
        self.ymax                = rospy.get_param("/workspace/y_max", 10.0)

        ship_center         = ((self.xmin + self.xmax) / 2, (self.ymin + self.ymax) / 2)
        self.obstacle       = ShipObstacle(ship_center, scale=ship_scale_factor)
        self.segments       = self.obstacle.copy_segments()
        self.n_segments     = len(self.segments)
        self.pose           = None

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub_lidar = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.pub_coverage = rospy.Publisher("/coverage", Float64, queue_size=1)
        self.pub_abort    = rospy.Publisher("/abort", String, queue_size=1)
    def odom_callback(self, msg):
        """
        Callback function for the odometry topic.
        Questo metodo viene chiamato ogni volta che viene ricevuto un messaggio 
        dal topic dell'odometria. Estrae la posizione (x, y) e l'orientamento 
        (theta) dal messaggio e li memorizza in un array numpy in self.pose.

        Args:
            msg (nav_msgs.msg.Odometry): Il messaggio dell'odometria contenente 
            la posizione e l'orientamento del robot.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.pose = np.array([x, y, theta])

    def get_ranges(self):
        """
        Restituisce i ranges rilevati dal lidar come una lista.
        Questa funzione utilizza i parametri del lidar e la posizione attuale per calcolare i ranges
        dagli ostacoli circostanti. i ranges vengono restituiti come una lista di valori.
        
        Returns:
            list: Una lista di distanze rilevate dal lidar.
        """
        ranges, angles, seen_segment_id =  lidar(self.pose, self.obstacle, self.lidar_params)

        for id in seen_segment_id:
            if self.segments[id].seen == False:
                self.segments[id].seen = True
        total_seen_segments = sum(1 for segment in self.segments.values() if segment.seen)
        coverage = round(float(total_seen_segments) / float(self.n_segments) * 100, 2)
        
        return ranges.tolist(), coverage
    
    def publish_lidar_data(self):
        """
        Pubblica i dati delle distanze rilevate dal LiDAR come messaggio LaserScan e il coverage.
        Questo metodo raccoglie i dati delle distanze rilevate dal LiDAR, li formatta
        in un messaggio LaserScan e lo pubblica sul topic appropriato.
        Inoltre pubblica il coverage come messaggio Float64.
        """

        # Ottiene le distanze rilevate dal LiDAR
        ranges, coverage = self.get_ranges()

        # Crea un messaggio LaserScan
        scan_msg = LaserScan()
        scan_msg.header.stamp       = rospy.Time.now()
        scan_msg.header.frame_id    = "base_link"
        scan_msg.angle_min          = - self.lidar_params['FoV']/2
        scan_msg.angle_max          = self.lidar_params['FoV']/2
        scan_msg.angle_increment    = self.lidar_params['FoV']/self.lidar_params['n_beams']
        scan_msg.time_increment     = self.scan_time / self.lidar_params['n_beams']
        scan_msg.scan_time          = self.scan_time
        scan_msg.range_min          = self.min_range
        scan_msg.range_max          = self.lidar_params['max_range']
        scan_msg.ranges             = ranges
        scan_msg.intensities        = [0.0]*self.lidar_params['n_beams']

        coverage_msg = Float64()
        coverage_msg.data = coverage

        # Pubblica il messaggio LaserScan e il messaggio di coverage
        self.pub_lidar.publish(scan_msg)
        self.pub_coverage.publish(coverage)


    def out_of_bounds_check(self):
        x = self.pose[0]
        y = self.pose[1]

        if x < self.xmin or x > self.xmax or y < self.ymin or y > self.ymax:
            msg = String()
            msg.data = "Out of bounds"
            self.pub_abort.publish(msg)
            
    def collision_check(self):
        x = self.pose[0]
        y = self.pose[1]
        
    def run(self):
        """
        Esegue il ciclo principale del nodo ROS.
        Questo metodo continua a eseguire un ciclo finche` il nodo ROS non viene arrestato.
        Durante ogni iterazione del ciclo, se l'attributo 'pose' non e' None, chiama il metodo 'publish_lidar_data'.
        Alla fine di ogni iterazione, il ciclo attende per un intervallo di tempo specificato dall'attributo `rate`.
        """
        while not rospy.is_shutdown():
            if self.pose is not None:
                self.publish_lidar_data()
                self.out_of_bounds_check()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        lidar_node = Lidar_sim()
        lidar_node.run()
    except rospy.ROSInterruptException:
        pass
