#!/usr/bin/env python3

import rospy
import torch
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from my_package.core.AC_v4 import AC_agent
from joystick_command.msg   import Rel_error_joystick
from models.msg import Coverage

def wrapTo180(angle):
    # type: (float) -> float
    """
    Converte l'angolo di input nell'intervallo [-180, 180].
    Parametri:
        angle (float): L'angolo in gradi da convertire.
    Ritorna:
        float: L'angolo convertito nell'intervallo [-180, 180].
    """
    return (angle + 180) % 360 - 180


class Agent:
    def __init__(self):
        rospy.init_node('rl_agent')
        self.rate = rospy.Rate(1)       # 1 Hz

        # Import parametri del lidar
        self.max_range = rospy.get_param('/lidar/max_range', 1.0)

        # Import parametri cinematici di Zeno
        v_surge_max    = rospy.get_param('/zeno/vel_max/surge', 0.2)                    # m/s
        v_sway_max     = rospy.get_param('/zeno/vel_max/sway', 0.2)                     # m/s
        omega_max      = np.deg2rad(rospy.get_param('/zeno/vel_max/omega', 10.0))       # rad/s   
        # self.zeno_limits = {'v_surge_max': v_surge_max, 'v_sway_max': v_sway_max, 'omega_max': omega_max, 'yaw_rel_max': np.pi/4}
        self.zeno_limits = {'v_surge_max': 0.1, 'v_sway_max': 0.05, 'omega_max': omega_max, 'yaw_rel_max': np.pi/4}

        self.K_distance = rospy.get_param('~my_controller/distance', 0.25)
        self.K_yaw      = rospy.get_param('~my_controller/yaw', 1.0)

        # Import limiti workspace
        workspace_params = ["/workspace/x_min", "/workspace/y_min", "/workspace/x_max", "/workspace/y_max"]
        while not all(rospy.has_param(param) for param in workspace_params) and not rospy.is_shutdown():
            rospy.loginfo("Attendo limiti workspace ... ")
            self.rate.sleep()
            
        xmin = rospy.get_param("/workspace/x_min", 0.0)
        ymin = rospy.get_param("/workspace/y_min", 0.0)
        xmax = rospy.get_param("/workspace/x_max", 10.0)
        ymax = rospy.get_param("/workspace/y_max", 10.0)
        self.workspace = (xmin, ymin, xmax, ymax)


        self.start          = False
        self.odom_recived   = False
        self.scan_received  = False
        self.curr_state     = None
        self.next_state     = None
        self.prev_state     = None
        self.data_saved     = False

        self.pub                = rospy.Publisher("/debug", String, queue_size=1)
        self.pub_rel_error      = rospy.Publisher('/relative_error', Rel_error_joystick, queue_size=10)
        self.pub_start_scan     = rospy.Publisher('/start_scan', String, queue_size=1)
        self.pub_save_data      = rospy.Publisher('/save_data', String, queue_size=1)

        rospy.Subscriber('/start', String, self.start_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/coverage', Coverage, self.coverage_callback)
        self.init_AC_agent()

    def init_AC_agent(self):
        """
        Inizializzio l'agente AC con i parametri salvati nel file .pth
        """
        # Import parametri agente
        self.data       = torch.load('/home/paolo/catkin_ws/src/rl_agent/weights/ShipRovCont-v0_750_ep_final.pth')
        self.device     = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.state_dim  = self.data['state_dim']
        action_dim      = self.data['action_dim']
        max_episodes    = self.data['max_episodes']
        buffer_params   = self.data['buffer_params']
        noise_type      = self.data['noise_type']
        noise_params    = self.data['noise_params']
        AC_params       = self.data['AC_params']

        # Inizializzo l'agente
        self.agent = AC_agent(    
            state_size=self.state_dim,
            action_size=action_dim,
            max_episodes=max_episodes,
            device=self.device,
            noise_params=noise_params,
            noise_type=noise_type,
            AC_params=AC_params,
            buffer_params=buffer_params
        )

        # Carico i pesi della rete neurale dell'attore
        self.agent.actor.load_state_dict(self.data['actor_state_dict'])

    def start_callback(self, msg):
        self.start = True
        self.next_state = "init"

    def coverage_callback(self, msg):
        if msg.data == 100.0 and self.data_saved == False:
            self.pub_save_data.publish("save data")
            self.data_saved = True
    def odom_callback(self, msg):
        """
        Callback function for the odometry topic.
        Estrae la posa (x, y, yaw) la memorizza in un array numpy in self.pose.
        Estra il twist (v_surge, v_sway, omega) e lo memorizza il self.twist

        Args:
            msg (nav_msgs.msg.Odometry): Il messaggio dell'odometria contenente 
            la posizione e l'orientamento del robot.
        """

        self.odom_recived = True
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        rotation = R.from_quat([q.x, q.y, q.z, q.w])
        _, _, yaw = rotation.as_euler('xyz', degrees=False)
        
        v_surge     = msg.twist.twist.linear.x
        v_sway      = msg.twist.twist.linear.y
        omega       = msg.twist.twist.angular.z

        # Normalizzo tutti i valori della posa e del twist tra 0 e 1
        xmin, ymin, xmax, ymax = self.workspace

        x_norm          = (x - xmin) / (xmax - xmin)
        y_norm          = (y - ymin) / (ymax - ymin)
        yaw_norm        = (yaw + np.pi) / (2 * np.pi)
        v_surge_norm    = (1 +  v_surge / self.zeno_limits['v_surge_max']) / 2
        v_sway_norm     = (1 +  v_sway  / self.zeno_limits['v_sway_max']) / 2
        omega_norm      = (1 +  omega   / self.zeno_limits['omega_max']) / 2

        # Salvo i valori normalizzati 
        self.norm_pose  = np.array([x_norm, y_norm, yaw_norm])
        self.norm_twist = np.array([v_surge_norm, v_sway_norm, omega_norm])

        # Salvo i valori reali
        self.pose = np.array([x, y, yaw])
        self.twist = np.array([v_surge, v_sway, omega])

    def scan_callback(self, msg):
        """
        Funzione di callback per il topic del Lidar.
        Salva i valori dei range solo per 10 raggi equispaziati e li normalizza tra 0 e 1.
        """

        self.scan_received = True
        # Id dei raggi selezionati
        indices = [0, 9, 19, 29, 39, 49, 59, 69, 79, 89]
        # Normalizzo i valori dei range tra 0 e 1 solo per i raggi selezionati
        self.selected_ranges = [msg.ranges[i]/self.max_range for i in indices]

    def send_commands(self):
        """
        Funzione che invia i comandi al topic /relative_error.
        Prende come input lo stato attuale del robot normalizzato tra 0 e 1
        e usa la rete neurale per calcolare l'azione da inviare.
        """

        # Creo lo stato normalizzato per la rete neurale
        state       = np.zeros(self.state_dim)
        state[:3]   = self.norm_pose
        state[3:6]  = self.norm_twist
        state[6:]   = self.selected_ranges

        # Selezione dell'azione che sarà compresa tra -1 e 1
        action = self.agent.act(state, add_noise=False)

        # Riporto i valori delle azioni ai limiti fisici di Zeno
        v_surge = action[0] * self.zeno_limits['v_surge_max']
        v_sway  = action[1] * self.zeno_limits['v_sway_max']
        yaw_rel = np.rad2deg(action[2] * self.zeno_limits['yaw_rel_max'])

        # Invio i comandi al topic /relative_error
        rel_error = Rel_error_joystick()
        rel_error.error_surge_speed = v_surge
        rel_error.error_sway_speed  = v_sway
        rel_error.error_yaw         = yaw_rel
        
        self.pub_rel_error.publish(rel_error)

    def move_to_starting_pose(self):

        curr_pose = self.pose
        xmin, ymin, xmax, ymax = self.workspace
        Cx, Cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        points = [  [xmin + 1, ymin + 1],
                    [xmin + 1, ymax - 1],
                    [xmax - 1, ymax - 1],
                    [xmax - 1, ymin + 1]]

        distances = np.array([np.linalg.norm(np.array(point) - curr_pose[:2]) for point in points])
        id_min    = np.argmin(distances)
        
        des_angle = np.arctan2(Cy - points[id_min][1], Cx - points[id_min][0])

        self.starting_pose  = np.array([points[id_min][0], points[id_min][1], des_angle])

        delta_pos = self.starting_pose[:2] - curr_pose[:2]
        yaw_des   = np.arctan2(delta_pos[1], delta_pos[0])
        yaw_rel   = np.rad2deg(yaw_des - curr_pose[2])   

        self.rotate_to_angle(yaw_rel)
        self.moving_to_pose(self.starting_pose)

        self.pub_start_scan.publish("start scan")

        rospy.loginfo("-------------------- %s --------------------", "perform full rotation")

        self.next_state = 'rl_agent'

    def rotate_to_angle(self, yaw_rel, angular_tolerance=5.0, omega_tolerance=0.015):
        """
        Funzione che fa compiere al robot una rotazione fino a raggiungere l'angolo desiderato (deg)
        """
        angular_delay = 12.0
        curr_yaw = np.rad2deg(self.pose[2])
        yaw_abs = curr_yaw + yaw_rel

        if abs(yaw_rel) < angular_delay:
            rel_error = Rel_error_joystick()
            rel_error.error_yaw = yaw_rel
            self.pub_rel_error.publish(rel_error)
            rospy.sleep(1)
        
        else:
            while abs(yaw_rel) > angular_delay and not rospy.is_shutdown():
                curr_yaw    = np.rad2deg(self.pose[2])
                yaw_rel     = wrapTo180(yaw_abs - curr_yaw)

                rel_error = Rel_error_joystick()
                rel_error.error_yaw = yaw_rel
                self.pub_rel_error.publish(rel_error)
                self.rate.sleep()
        
        rel_error = Rel_error_joystick()
        rel_error.error_yaw = 0.0
        self.pub_rel_error.publish(rel_error)
        self.rate.sleep()

        while not rospy.is_shutdown():
            rel_error = Rel_error_joystick()
            rel_error.error_yaw = 0.0
            self.pub_rel_error.publish(rel_error)
            yaw_rel = wrapTo180(yaw_abs - curr_yaw)
            if abs(self.twist[2]) < omega_tolerance:
                return
            self.rate.sleep()

    def moving_to_pose(self, pose):

        sample_reached = False
        curr_phase = 'heading'
        next_phase = 'heading'
        position_tolerance  = 0.25
        vel_tolerance       = 0.02

        # pose_marker = generate_pose_marker(pose, color=(1, 0, 0))
        # self.pub_samples.publish(pose_marker)

        while not sample_reached and not rospy.is_shutdown(): 
            if curr_phase == 'heading':  
                r_abs           = np.linalg.norm(pose[:2] - self.pose[:2])
                delta_pos       = pose[:2] - self.pose[:2]
                robot_direction = np.array([np.cos(self.pose[2]), np.sin(self.pose[2])])

                # Movimento in avanti   se l'angolo tra la posizione attuale e quella desiderata e` compreso tra -90 e 90
                if np.dot(delta_pos, robot_direction) >= 0:
                    r       = r_abs
                    yaw_des = np.arctan2(delta_pos[1], delta_pos[0])

                # Se l'angolo tra la posizione attuale e quella desiderata NON e` compreso tra -90 e 90 e 
                # la distanza e` maggiore di 2 * position_tolerance, mi riallineo
                elif np.linalg.norm(pose[:2] - self.pose[:2]) > 2 * position_tolerance:
                    r = 0.0
                    yaw_des = np.arctan2(delta_pos[1], delta_pos[0])

                # Movimento all'indietro se l'angolo tra la posizione attuale e quella desiderata NON e` compreso tra -90 e 90
                # e la distanza e` minore di 2 * position_tolerance (parking)
                else:
                    r       = - r_abs
                    yaw_des = np.arctan2(-delta_pos[1], -delta_pos[0])

                yaw_rel = np.clip(self.K_yaw * np.rad2deg(yaw_des - self.pose[2]), -90, 90)
                v_des   = np.clip(self.K_distance * r , -self.zeno_limits['v_surge_max'], self.zeno_limits['v_surge_max'])

                if np.linalg.norm(pose[:2] - self.pose[:2]) < position_tolerance and abs(self.twist[0]) < vel_tolerance:
                    next_phase = 'aligning'
                    rospy.loginfo("Aligning")

            elif curr_phase == 'aligning':
                yaw_des     = pose[2]
                curr_yaw    = self.pose[2]

                yaw_rel = wrapTo180(np.rad2deg(yaw_des - curr_yaw))
                self.rotate_to_angle(yaw_rel)
                return
                   
            # Mando i comandi al robot
            rel_error = Rel_error_joystick()
            rel_error.error_yaw         = yaw_rel
            rel_error.error_surge_speed = v_des
            self.pub_rel_error.publish(rel_error)

            # Aggiorno la fase corrente
            curr_phase = next_phase

            # Aspetto il rate
            self.rate.sleep()


    def run(self):
        while not rospy.is_shutdown():
            


            # if self.start:

            #     if self.odom_recived and self.scan_received:
            #         self.send_commands()
            if self.curr_state == "init":
                if self.odom_recived:
                    self.move_to_starting_pose()

            elif self.curr_state == "rl_agent":
                if self.odom_recived and self.scan_received:
                    self.send_commands()

            # Update macchina a stati
            self.prev_state = self.curr_state
            self.curr_state = self.next_state
            self.rate.sleep()


if __name__ == '__main__':
    try:
        agent = Agent()
        agent.run()
    except rospy.ROSInterruptException:
        pass
