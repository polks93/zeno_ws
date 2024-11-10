#!/home/paolo/miniconda3/envs/zenoenv/bin/python

import rospy
import torch
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from my_package.core.AC_v4 import AC_agent
from joystick_command.msg   import Rel_error_joystick


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

        # Import limiti workspace
        xmin = rospy.get_param("/workspace/x_min", 0.0)
        ymin = rospy.get_param("/workspace/y_min", 0.0)
        xmax = rospy.get_param("/workspace/x_max", 10.0)
        ymax = rospy.get_param("/workspace/y_max", 10.0)
        self.workspace = (xmin, ymin, xmax, ymax)


        self.start          = False
        self.odom_recived   = False
        self.scan_received  = False

        self.pub            = rospy.Publisher("/debug", String, queue_size=1)
        self.pub_rel_error  = rospy.Publisher('/relative_error', Rel_error_joystick, queue_size=10)
        
        rospy.Subscriber('/start', String, self.start_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

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
        self.norm_pose = np.array([x_norm, y_norm, yaw_norm])
        self.norm_twist = np.array([v_surge_norm, v_sway_norm, omega_norm])

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

        state = np.zeros(self.state_dim)
        state[:3]   = self.norm_pose
        state[3:6]  = self.norm_twist
        state[6:]   = self.selected_ranges

        rospy.loginfo(state)

        # state = np.random.random(self.state_dim)

        # Selezione dell'azione che sarà compresa tra -1 e 1
        action = self.agent.act(state, add_noise=False)
        # action[0] = 0.0
        # action[1] = 0.0
        # action[2] = np.pi/4

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

    def run(self):
        while not rospy.is_shutdown():
            if self.start:
                if self.odom_recived and self.scan_received:
                    self.send_commands()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        agent = Agent()
        agent.run()
    except rospy.ROSInterruptException:
        pass
