#!/usr/bin/env python

import rospy
import numpy as np
import tf

from obstacle_simulation import ShipObstacle, lidar
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
from models.msg import Coverage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# def create_zeno_marker(x, y, radius, marker_id=0, lifetime=0):
#     """
#     Crea un marker circolare per Rviz.

#     Args:
#         x (float): Coordinata x del centro del cerchio.
#         y (float): Coordinata y del centro del cerchio.
#         radius (float): Raggio del cerchio.
#         frame_id (str): Nome del frame di riferimento.
#         marker_id (int): ID univoco del marker.
#         lifetime (float): Durata del marker in secondi (0 per infinito).

#     Returns:
#         Marker: Oggetto Marker configurato.
#     """
#     marker = Marker()
#     marker.header.frame_id = 'map'
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "zeno_marker"
#     marker.id = marker_id
#     marker.type = Marker.LINE_STRIP
#     marker.action = Marker.ADD
#     marker.scale.x = 0.2  # Spessore della linea

#     # Colore del marker (RGBA)
#     marker.color.r = 1.0
#     marker.color.g = 0.0
#     marker.color.b = 0.0
#     marker.color.a = 1.0

#     # Durata del marker (0 per infinito)
#     marker.lifetime = rospy.Duration(lifetime)

#     # Calcolo dei punti del cerchio
#     points = []
#     num_points = 50  # Maggiore il il numero, piu "liscio" sara il cerchio
#     for i in range(num_points + 1):
#         angle = 2 * 3.14159 * i / num_points
#         point = Point()
#         point.x = x + radius * np.cos(angle)
#         point.y = y + radius * np.sin(angle)
#         point.z = 0  # Z fisso per il 2D
#         points.append(point)

#     marker.points = points

#     return marker

def create_zeno_marker(x, y, radius, marker_id=0, lifetime=0):
    """
    Crea un marker circolare (cilindrico) per Rviz.

    Args:
        x (float): Coordinata x del centro del cerchio.
        y (float): Coordinata y del centro del cerchio.
        radius (float): Raggio del cerchio.
        marker_id (int): ID univoco del marker.
        lifetime (float): Durata del marker in secondi (0 per infinito).

    Returns:
        Marker: Oggetto Marker configurato.
    """
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = "zeno_marker"
    marker.id = marker_id
    marker.type = Marker.CYLINDER  # Tipo cilindro per un marker circolare
    marker.action = Marker.ADD

    # Posizione del centro del marker
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0  # Z fisso per il piano 2D

    # Orientamento del marker (default: nessuna rotazione)
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Dimensioni del cilindro
    marker.scale.x = 2 * radius  # Diametro sul piano x
    marker.scale.y = 2 * radius  # Diametro sul piano y
    marker.scale.z = 0.01  # Altezza minima per simulare un cerchio sul piano

    # Colore del marker (RGBA)
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Durata del marker (0 per infinito)
    marker.lifetime = rospy.Duration(lifetime)

    return marker

def create_markers_msg(points, color=[1.0, 0.0, 0.0]):
    """Crea un messaggio MarkerArray a partire dalle celle visibili."""
    marker_array = MarkerArray()

    
    x, y = zip(*points)
    z = 0.0

    for i in range(len(x)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ship_points"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x[i]
        marker.pose.position.y = y[i]
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1  # Opacita
        marker.color.r = color[0]  
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker_array.markers.append(marker)
    return marker_array



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

        self.footprint      = rospy.get_param('/zeno/footprint', 0.25)
        rospy.loginfo("Footprint: {}".format(self.footprint))

        # ship_center         = rospy.get_param('/ship/center', (0,0))
        ship_scale_factor   = rospy.get_param('/ship/scale', 0.9)
        self.custom_ship         = rospy.get_param('/ship/custom_ship', False)

        self.scan_time      = 1.0 / self.Hz
        self.rate           = rospy.Rate(self.Hz)
        self.counter        = 0

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

        self.obstacle       = ShipObstacle(ship_center, scale=ship_scale_factor, inflation_radius=self.footprint, use_custom_ship=self.custom_ship)
        # if self.custom_ship:
        #     self.obstacle.rototranslate_ship(-np.pi/4, (0.5,-0.5))

        self.segments       = self.obstacle.copy_segments()
        self.n_segments     = len(self.segments)
        self.pose           = None
        self.start          = False
        self.enable_pub_marker = True

        rospy.Subscriber("/start_scan", String, self.start_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.pub_lidar      = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.pub_coverage   = rospy.Publisher("/coverage", Coverage, queue_size=1)
        self.pub_abort      = rospy.Publisher("/abort", String, queue_size=1)
        self.pub_markers   = rospy.Publisher("/ship_markers", MarkerArray, queue_size=1)
        self.pub_zeno_marker = rospy.Publisher("/zeno_marker", Marker, queue_size=1)
    def start_callback(self, msg):
        self.start = True

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



        # Pubblica il messaggio LaserScan e il messaggio di coverage
        self.pub_lidar.publish(scan_msg)

        coverage_msg = Coverage()
        coverage_msg.header.stamp = rospy.Time.now()
        coverage_msg.data = coverage

        self.pub_coverage.publish(coverage_msg)


    def publish_ship_marker(self):
        """
        Pubblica un marker RViz per visualizzare la forma dell'ostacolo ShipObstacle.
        """
        points = self.obstacle.points
        markers = create_markers_msg(points)
        self.pub_markers.publish(markers)

    def publish_zeno_marker(self):
        if self.pose is not None:
            x = self.pose[0]
            y = self.pose[1]
            zeno_marker = create_zeno_marker(x, y, self.footprint)
            self.pub_zeno_marker.publish(zeno_marker)

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
        Cx_ship, Cy_ship = self.obstacle.center

        if np.linalg.norm([x - Cx_ship, y - Cy_ship]) > self.obstacle.radius + self.footprint:
            return 
        else:

            if self.custom_ship:
                collision = self.obstacle.point_in_custom_ship(point=[x,y])
            else:
                collision = self.obstacle.point_in_ship(point=[x,y])

            if collision:
                msg = String()
                msg.data = "Collision"
                self.pub_abort.publish(msg)
                rospy.logwarn("Collision detected")
                return
            
    def run(self):
        """
        Esegue il ciclo principale del nodo ROS.
        Questo metodo continua a eseguire un ciclo finche` il nodo ROS non viene arrestato.
        Durante ogni iterazione del ciclo, se l'attributo 'pose' non e' None, chiama il metodo 'publish_lidar_data'.
        Alla fine di ogni iterazione, il ciclo attende per un intervallo di tempo specificato dall'attributo `rate`.
        """
        
        while not rospy.is_shutdown():
            self.counter += 1
            if self.enable_pub_marker:
                self.publish_ship_marker()
                self.publish_zeno_marker()
                
            if self.pose is not None and self.start:
                self.publish_lidar_data()
                self.out_of_bounds_check()
                if self.counter % 10 == 0:
                    self.collision_check()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        lidar_node = Lidar_sim()
        lidar_node.run()
    except rospy.ROSInterruptException:
        pass
