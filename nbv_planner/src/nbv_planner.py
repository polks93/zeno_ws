#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg           import Odometry
from std_msgs.msg           import String, Float64
from joystick_command.msg   import Rel_error_joystick
from nav_msgs.msg           import OccupancyGrid
from gridmap_functions      import OccupancyGridWrapper, find_visible_cells, find_frontier_cells, find_contour_cells, find_occ_cells, create_markers_msg
from RRT_functions          import generate_RRT_samples, generate_pose_marker, generate_sequence
from visualization_msgs.msg import MarkerArray, Marker


def deWrapAngle(angle):
    # type: (float) -> float
    """
    Converte un angolo da radianti a gradi e lo normalizza nell'intervallo [0, 360).

    Parameters:
    angle (float): L'angolo in radianti da convertire e normalizzare.

    Returns:
    float: L'angolo convertito in gradi e normalizzato nell'intervallo [0, 360).
    """

    angle = np.rad2deg(angle)
    if angle < 0:
        angle += 360
    return angle

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

class NbvPlanner():
    def __init__(self):
        rospy.init_node('nbv_planner')
        self.rate = rospy.Rate(30)

        # Import parametri lidar
        n_beams             = rospy.get_param('/lidar/n_beams', 90)
        max_range           = rospy.get_param('/lidar/max_range', 5.0)
        FoV_deg             = rospy.get_param('/lidar/FoV', 360.0)
        self.lidar_params   = {'n_beams': n_beams, 'max_range': max_range, 'FoV': np.deg2rad(FoV_deg)}

        # Import parametri di campionamento
        R                       = rospy.get_param('/sampling/R', 3)   
        N_samples               = rospy.get_param('/sampling/N_samples', 50)
        T_max                   = rospy.get_param('/sampling/T_max', 0.5)
        max_counter             = rospy.get_param('/sampling/max_counter', 100)
        self.sampling_params    = {'R': R, 'N_samples': N_samples, 'T_max': T_max, 'max_counter': max_counter}

        # Import pesi dei campioni
        w_frontier              = rospy.get_param('/weights/frontier', 0.1)
        w_contour               = rospy.get_param('/weights/contour', 0.9)
        w_occupied              = rospy.get_param('/weights/occupied', 0.0)
        self.sample_weights     = {'frontier': w_frontier, 'contour': w_contour, 'occupied': w_occupied}
        
        # Import parametri del controllore simulato per RRT
        K_yaw   = rospy.get_param('/zeno/sim_controller/yaw', 20.0)
        K_omega = rospy.get_param('/zeno/sim_controller/omega', 50.0)
        K_surge = rospy.get_param('/zeno/sim_controller/surge', 0.25)
        K_sway  = rospy.get_param('/zeno/sim_controller/sway', 0.25)
        self.controller_params  = {'yaw': K_yaw, 'omega': K_omega, 'v_surge': K_surge, 'v_sway': K_sway}	

        # Import parametri cinematici di Zeno
        v_surge_max    = rospy.get_param('/zeno/vel_max/surge', 0.2)                    # m/s
        v_sway_max     = rospy.get_param('/zeno/vel_max/sway', 0.2)                     # m/s
        omega_max      = np.deg2rad(rospy.get_param('/zeno/vel_max/omega', 10.0))       # rad/s   
        self.zeno_limits = {'v_surge_max': v_surge_max, 'v_sway_max': v_sway_max, 'omega_max': omega_max}

        # Parametri di controllo attivo 
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

        # Import inflation radius
        self.inflation_radius = rospy.get_param("/map/inflation_radius", 0.5)
        self.inflation_radius_samples = rospy.get_param("/map/inflation_radius_samples", 0.75)

        # Init variabili di classe 
        self.pose               = None
        self.twist              = None 
        self.my_map             = None
        self.coverage           = 0.0
        self.curr_state         = None
        self.prev_state         = None
        self.next_state         = None

        self.publish_cell_on    = True
        self.publish_map_on     = True
        self.publish_samples_on = True
        self.publish_path_on    = True

        # Dizionario relativo alle informazioni di recovery
        self.recovery_info = {
            'counter':      0,
            'max_counter':  8,
            'yaw_step':     45,
            'cause':   None,
        }

        # Genero tutti i publisher
        self.pub_rel_error       = rospy.Publisher('/relative_error', Rel_error_joystick, queue_size=10)
        self.pub_my_map          = rospy.Publisher('/my_map', OccupancyGrid, queue_size=10)
        self.pub_visible_cells   = rospy.Publisher('/visible_cells', MarkerArray, queue_size=10)
        self.pub_frontier_cells  = rospy.Publisher('/frontier_cells', MarkerArray, queue_size=10)
        self.pub_contour_cells   = rospy.Publisher('/contour_cells', MarkerArray, queue_size=10)
        self.pub_samples         = rospy.Publisher('/samples', MarkerArray, queue_size=10)
        self.pub_best_samples    = rospy.Publisher('/best_samples', MarkerArray, queue_size=10)
        self.pub_pose_path       = rospy.Publisher('/pose_path', MarkerArray, queue_size=10)

        # Genero tutti i subscriber
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/start', String, self.start_callback)   
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/coverage', Float64, self.coverage_callback)

    def coverage_callback(self, msg): 
        """
        Callback per il subscriber del coverage, memorizza il valore del coverage.
        Chiude il nodo se il coverage e` 100%.
        """
        self.coverage = msg.data
        # if self.coverage == 100.0:
        #     rospy.loginfo("Coverage 100%% reached, mission completed ....")
        #     rospy.signal_shutdown("Coverage 100%% reached")

    def map_callback(self, msg):
        """
        Dopo aver ricevuto il messaggio contenente la mappa, genero un oggetto OccupancyGridWrapper
        che mi permette di accedere facilmente ai dati della mappa.
        """
        self.my_map = OccupancyGridWrapper(msg, self.workspace)

    def odom_callback(self, msg):
        """
        Callback function for the odometry topic.
        Estrae la posa (x, y, yaw) la memorizza in un array numpy in self.pose.
        Estra il twist (v_surge, v_sway, omega) e lo memorizza il self.twist

        Args:
            msg (nav_msgs.msg.Odometry): Il messaggio dell'odometria contenente 
            la posizione e l'orientamento del robot.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        v_surge     = msg.twist.twist.linear.x
        v_sway      = msg.twist.twist.linear.y
        omega       = msg.twist.twist.angular.z

        self.pose = np.array([x, y, yaw])
        self.twist = np.array([v_surge, v_sway, omega])

    def start_callback(self, msg):
        """
        Callback associata alla ricezione del comando di start.
        Inizializza la variabile di stato e calcola l'angolo che si trova a 180 gradi
        rispetto alla posizione iniziale del robot.
        Inoltre effettua il passaggio allo stato 'init'.
        """
        if msg.data == 'move':
            self.move = True
            return

        if msg.data == 'test':
            self.next_state = 'test'
            self.test_angle = 10
            return
        
        if msg.data == 'save_coordinates':
            self.next_state = 'save_coordinates'
            return
        
        if self.pose is not None:

            # Bool per il mezzo giro completato
            self.half_turn      = False
            # Salvo l'orientamento iniziale del robot
            self.starting_yaw   = np.rad2deg(self.pose[2])
            # Calcolo l'angolo che si trova a 180 gradi rispetto alla posizione iniziale
            self.half_yaw       = np.rad2deg(wrapToPi(self.pose[2] + np.pi))
            # Passaggio allo stato 'init'
            self.next_state = 'init'

    def perform_full_rotation(self):
        """
        Funzione che fa compiere al robot un giro completo su se stesso.
        Serve a geneare una zona libera intorno al robot.
        """
        # Se il robot ha compiuto mezzo giro, calcolo yaw_realtivo
        if self.half_turn:
            curr_yaw      = np.rad2deg(self.pose[2])
            yaw_rel       = wrapTo180(self.starting_yaw - curr_yaw)
            self.rotate_to_angle(yaw_rel)
            self.next_state = 'sampling'
            return

        # Se il robot non ha compiuto mezzo giro, il valore di yaw desiderato e` + 90
        else:
            curr_yaw        = np.rad2deg(self.pose[2])
            yaw_rel         = 90.0

            # Check per il mezzo giro
            if self.starting_yaw > 0 and abs(curr_yaw) < abs(self.half_yaw) and np.sign(curr_yaw) == np.sign(self.half_yaw):
                self.half_turn = True     
            elif self.starting_yaw < 0 and curr_yaw > self.half_yaw and np.sign(curr_yaw) == np.sign(self.half_yaw):
                self.half_turn = True
            elif self.starting_yaw == 0 and curr_yaw < 0:
                self.half_turn = True
            elif self.starting_yaw == np.pi and curr_yaw > 0:
                self.half_turn = True

        yaw_rel = np.clip(yaw_rel, -90, 90)
        rel_error = Rel_error_joystick()
        rel_error.error_yaw = yaw_rel
        self.pub_rel_error.publish(rel_error)

    def publish_cell_markers(self, pose=None):
        """
        Funzione che pubblica i marker delle celle visibili, delle celle di frontiera e delle celle di contorno
        del campione passato come argomento.
        Se il campione non e` passato, usa la posizione corrente del robot.
        """
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker)
        self.pub_contour_cells.publish(marker_array)
        self.pub_frontier_cells.publish(marker_array)
        self.pub_visible_cells.publish(marker_array)

        if pose is None:
            pose = self.pose
        
        if self.my_map is not None:
            visible_cells   = find_visible_cells(pose, self.lidar_params, self.my_map)
            frontier_cells  = find_frontier_cells(self.my_map, visible_cells)
            contour_cells   = find_contour_cells(self.my_map, visible_cells)

            visible_cells_marker    = create_markers_msg(self.my_map, visible_cells, color=(0, 1, 0))
            frontier_cells_marker   = create_markers_msg(self.my_map, frontier_cells, color=(1, 0, 0))
            contour_cells_marker    = create_markers_msg(self.my_map, contour_cells, color=(0, 0, 1))

            self.pub_visible_cells.publish(visible_cells_marker)
            self.pub_frontier_cells.publish(frontier_cells_marker)
            self.pub_contour_cells.publish(contour_cells_marker)

    def publish_samples_markers(self, samples, value_function):
        """
        Genero i marker dei 3 campioni migliori e li pubblico in rosso.
        Pubblica anche i marker di tutti gli altri campioni con un colore diverso.
        Inoltre pubblica i marker delle celle visibili, delle celle di frontiera e delle celle di contorno
        del campione migliore.
        """

        # Estraggo i 3 campioni migliori
        top_3_id        = np.argsort(value_function)[-3:][::-1]
        top_3_samples   = np.array([samples[i] for i in top_3_id])
        samples         = np.delete(samples, top_3_id, axis=0)

        # Pubblico i marker delle celle del campione migliore
        if self.publish_cell_on:
            self.publish_cell_markers(top_3_samples[0])
        
        # Pubblico i marker dei 3 migliori campioni
        bests_samples_marker = generate_pose_marker(top_3_samples[0], color=(1, 0, 0))
        self.pub_best_samples.publish(bests_samples_marker)

        # Pubblico i marker di tutti gli altri campioni
        all_poses_marker = generate_pose_marker(samples, color=(0, 0, 1), scale=0.5)
        self.pub_samples.publish(all_poses_marker)

    def publish_pose_path(self, pose_sequence):
        """
        Funzione che pubblica i marker relativi alla sequenza di pose passata come argomento.
        """
        pose_path_marker = generate_pose_marker(pose_sequence, color=(0, 1, 0), scale=0.55)
        self.pub_pose_path.publish(pose_path_marker)

    def sampling(self):
        """
        Genero un albero RRT a partire dalla posizione corrente del robot contenente N nodi.
        Se riesco a generare l'albero, calcolo il valore della funzione di costo per ogni nodo.
            Scelgo il nodo con il valore piu` alto.
            Mi muovo verso il nodo scelto.
            
        Altrimenti rimango nello stato di sampling e riprovo a generare l'albero.
        """

        # Svuoto i marker relativi alle pose
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker)
 
        self.pub_best_samples.publish(marker_array)
        self.pub_samples.publish(marker_array)
        self.pub_pose_path.publish(marker_array)

        # Genero due mappe inflated
        x, y                    = self.pose[0], self.pose[1]
        my_inflated_map         = self.my_map.mini_inflated_gridmap_generation((x, y), self.inflation_radius, self.sampling_params['R'])
        samples_inflated_map    = self.my_map.mini_inflated_gridmap_generation((x, y), self.inflation_radius_samples, self.sampling_params['R'])

        # Genero un albero RRT sulla base della mappa inflated
        tree, samples = generate_RRT_samples(
            workspace=self.workspace,
            map=my_inflated_map,
            samples_map=samples_inflated_map,
            curr_pose=self.pose,
            sampling_params=self.sampling_params,
            kinematics_params=self.zeno_limits,
            control_params=self.controller_params
        )

        # Se l'albero e` troppo piccolo, vado in recovering
        if len(tree) < 20:
            rospy.loginfo("Tree too small, retrying")
            self.next_state = 'recovering'
            self.recovery_info['cause'] = 'small_tree'
            return None, None
        
        # Calcolo il valore della funzione obiettivo per ogni campione
        value_function = self.evaluate_samples(samples)

        # Se non ci sono campioni validi, vado in recovering
        if np.max(value_function) == 0:
            rospy.loginfo("No valid samples, retrying")
            self.next_state = 'recovering'
            self.recovery_info['cause'] = 'no_samples'
            return None, None
        
        # Estraggo il campione migliore e calcolo la sequenza di 
        # nodi per raggiungerlo
        id_best_sample      = np.argmax(value_function)
        # best_sample         = samples[id_best_sample]

        # Pubblico i marker relativi ai campioni
        if self.publish_samples_on:
            self.publish_samples_markers(samples, value_function)

        # Pubblico la mappa inflated
        if self.publish_map_on:
            self.pub_my_map.publish(samples_inflated_map.grid)

        # Passo al prossimo stato
        self.next_state = 'moving_to_nbv'
        
        # self.move = False
        # while not self.move and not rospy.is_shutdown():
        #     self.publish_samples_markers(samples, value_function)
        #     rospy.sleep(10)
        # Come output ho il campione migliore
        return id_best_sample, tree
    
    def evaluate_samples(self, samples):
        """
        Funzione che calcola il valore della funzione obiettivo per ogni campione.
        La funzione obiettivo e` definita come:
            f(x) = w_f * N_frontier/N_max_frontier + w_c * N_contour/N_max_contour
        Args:
            samples (np.array): Un array numpy contenente i campioni da valutare.
        Returns:
            np.array: Un array numpy contenente il valore della funzione obiettivo per ogni campione.
        """
        # Per prima cosa calcolo l'area massima visibile
        x, y, _     = self.pose
        curr_cell   = self.my_map.world_to_grid((x, y))
        R_cell      = int(np.ceil(self.sampling_params['R'] / self.my_map.resolution))
        range_cell  = int(np.ceil(self.lidar_params['max_range'] / self.my_map.resolution))

        # Indici dell'area massima visibile
        min_row = max(curr_cell[0] - R_cell - range_cell, self.my_map.min_row)
        max_row = min(curr_cell[0] + R_cell + range_cell, self.my_map.max_row)
        min_col = max(curr_cell[1] - R_cell - range_cell, self.my_map.min_col)
        max_col = min(curr_cell[1] + R_cell + range_cell, self.my_map.max_col)

        # Cacolo le celle all'interno dell'area massima visibile
        max_visible_cells = set()
        for i in range(min_row + 1, max_row + 1):
            for j in range(min_col + 1, max_col + 1):
                max_visible_cells.add((i, j))

        # Calcolo le celle di frontiera e di contorno massime 
        max_visible_cells   = np.array(list(max_visible_cells))
        max_frontier_cells  = find_frontier_cells(self.my_map, max_visible_cells)
        max_contour_cells   = find_contour_cells(self.my_map, max_visible_cells)
        max_occ_cells       = find_occ_cells(self.my_map, max_visible_cells)  


        # Calcolo il numero massimo di celle di frontiera e di contorno per normalizzare
        N_max_frontier  = max(len(max_frontier_cells), 1)
        N_max_contour   = max(len(max_contour_cells), 1)
        N_max_occ       = max(len(max_occ_cells), 1)

        # Inizializzo il valore della funzione obiettivo per ogni campione
        value_function = np.zeros(len(samples))
        w_f         = self.sample_weights['frontier']
        w_c         = self.sample_weights['contour']
        w_occ       = self.sample_weights['occupied']
        w_distance  = 0.0

        if len(max_occ_cells) == 0 and len(max_contour_cells) == 0:
            w_distance = 0.2

        # Calcolo il valore della funzione obiettivo per ogni campione, escludendo la sorgente
        for i in range(1, len(samples)):
            visible_cells   = find_visible_cells(samples[i], self.lidar_params, self.my_map)
            frontier_cells  = find_frontier_cells(self.my_map, visible_cells)
            contour_cells   = find_contour_cells(self.my_map, visible_cells)
            occ_cells       = find_occ_cells(self.my_map, visible_cells)

            N_frontier  = len(frontier_cells)
            N_contour   = len(contour_cells)
            N_occ       = len(occ_cells)

            distance    = np.linalg.norm(samples[i][:2] - self.pose[:2])

            value_function[i] = w_f * N_frontier/N_max_frontier + w_c * N_contour/N_max_contour + w_occ * N_occ/N_max_occ + w_distance * distance/self.sampling_params['R']

        return value_function

    def moving_to_nbv(self, pose_sequence):

        """ 
        Questa funzione fa muovere l'agente attraverso tutte le pose che lo portano alla Next Best View.
        - I campioni intermedi vengono raggiungi con velocita` costante, dopo aver passato un campione, il robot si muove verso il successivo.
        - L'ultimo campione deve essere raggiunto con velocita` che tende a zero.
        Args:
            pose_sequence (list): Una lista di array numpy contenenti le pose che portano alla Next Best View.
        """
        if self.publish_path_on:
            self.publish_pose_path(pose_sequence)

        # Se ho solo una posa da raggiungere, mi muovo direttamente verso di essa
        if np.shape(pose_sequence)[0] == 1:
            self.moving_to_pose(pose_sequence[0])
            return
        
        
        # Se ho piu` di una posa da raggiungere, mi muovo verso le pose intermedie
        else:

            position_tolerance  = 0.5
            while (np.shape(pose_sequence)[0]) > 1 and not rospy.is_shutdown():

                # Ricavo la prossima posa da raggiungere e la rimuovo dalla lista
                pose            = pose_sequence.pop(0)
                curr_distance   = np.linalg.norm(pose[:2] - self.pose[:2])

                # Resto in questo ciclo finche` non raggiungo la posa intermedia o la supero (condizione sugli angoli)
                while curr_distance > position_tolerance and not rospy.is_shutdown():
                    curr_distance   = np.linalg.norm(pose[:2] - self.pose[:2])
                    delta_pos       = pose[:2] - self.pose[:2]
                    yaw_des         = np.arctan2(delta_pos[1], delta_pos[0])
                    v_des           = self.zeno_limits['v_surge_max']/2
                    yaw_rel         = np.clip(self.K_yaw * np.rad2deg(yaw_des - self.pose[2]), -90, 90)
                    robot_direction = np.array([np.cos(self.pose[2]), np.sin(self.pose[2])])

                    # Se ho superato la posa intermedia senza trovarmi entro position_tolerance, 
                    # mi fermo e punto al prossimo campione
                    if np.dot(delta_pos, robot_direction) < 0:
                        rospy.logwarn("Superato campione intermedio, mi allineo comunque come il campione")

                        yaw_des = pose[2]
                        curr_yaw    = self.pose[2]
                        yaw_rel     = wrapTo180(np.rad2deg(yaw_des - curr_yaw))
                        self.rotate_to_angle(yaw_rel)
                        rospy.logwarn("Fatto, vado al successivo")
                        break
                    

                    # Mando i comandi al robot
                    rel_error = Rel_error_joystick()
                    rel_error.error_yaw         = yaw_rel
                    rel_error.error_surge_speed = v_des
                    self.pub_rel_error.publish(rel_error)

                    self.rate.sleep()

            # Finite le pose intermedie, mi muovo verso l'ultima
            self.moving_to_pose(pose_sequence[0])       

    def moving_to_pose(self, pose):

        sample_reached = False
        curr_phase = 'heading'
        next_phase = 'heading'
        position_tolerance  = 0.25
        vel_tolerance       = 0.02

        pose_marker = generate_pose_marker(pose, color=(1, 0, 0))
        self.pub_samples.publish(pose_marker)

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
                v_des   = np.clip(self.K_distance * r , -self.zeno_limits['v_surge_max']/2, self.zeno_limits['v_surge_max']/2)

                if abs(r) < position_tolerance and abs(self.twist[0]) < vel_tolerance:
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

    def print_state(self):
        if self.curr_state != self.prev_state:
            rospy.loginfo("-------------------- %s --------------------", self.curr_state) 

    def handle_recovery(self):
        rospy.logwarn("Recovery in progress, cause: %s, prev_state: %s", self.recovery_info['cause'], self.prev_state)

        # Se ho gia` provato a recuperare per un numero massimo di volte, termino la missione
        if self.recovery_info['counter'] >= self.recovery_info['max_counter']:
            rospy.logerr("Recovery failed, mission failed ....")
            self.next_state = 'abort'
            return
        
        if self.recovery_info['cause'] == 'small_tree':
            self.rotate_to_angle(self.recovery_info['yaw_step'])
            self.next_state = 'sampling'
            return

        elif self.recovery_info['cause'] == 'no_samples':
            self.rotate_to_angle(self.recovery_info['yaw_step'])
            self.next_state = 'sampling'
            return

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
        
    def run(self):
        while not rospy.is_shutdown():
            
            # INIT: l'agente fa un giro completo su se stesso
            if self.curr_state == 'init':
                
                self.print_state()
                self.perform_full_rotation()

            # SAMPLING: l'agente genera un albero RRT, se riesce a generarlo, passa allo stato moving_to_nbv
            elif self.curr_state == 'sampling':
                self.print_state()
                id_best_sample, tree = self.sampling()

                if self.next_state == 'moving_to_nbv':
                    sequence = generate_sequence(tree, id_best_sample)
            
            # MOVING_TO_NBV: l'agente si muove verso la Next Best View passando per le pose intermedie
            elif self.curr_state == 'moving_to_nbv':   
                self.print_state()
                self.moving_to_nbv(sequence)
                self.next_state = 'sampling'
            
            # RECOVERING: l'agente effettua delle rotazioni di recovery, poi torna allo stato di sampling
            elif self.curr_state == 'recovering':
                self.print_state()
                self.handle_recovery()
            
            elif self.curr_state == 'test':
                rospy.loginfo("Rotation start")
                self.rotate_to_angle(self.test_angle)
                rospy.loginfo("Rotation done")
                self.next_state = 'idle'

            elif self.curr_state == 'save_coordinates':
                xmin, ymin, xmax, ymax = self.workspace
                if self.pose is not None:
                    # self.rotate_to_angle(180)
                    self.moving_to_pose(np.array([xmax, ymax, 0.0]))
                    self.next_state = 'idle'

            # DONE: la missione e` completata, termino il nodo
            elif self.curr_state == 'done':
                rospy.loginfo("Mission completed, shutting down ....")
                rospy.signal_shutdown("Mission completed")
            
            # ABORT: la missione e` fallita, termino il nodo
            elif self.curr_state == 'abort':
                rospy.logerr("Mission aborted, shutting down ....")
                rospy.signal_shutdown("Mission aborted")

            # Update recovery
            if self.next_state == 'recovering' and self.prev_state == 'recovering':
                self.recovery_info['counter'] += 1
                rospy.logwarn("Recovery done %d times, after: %s", self.recovery_info['counter'], self.curr_state)
            elif self.prev_state == 'recovering' and self.next_state != 'recovering' and self.next_state != 'abort':
                rospy.logwarn("Azzero il counter di recovery")
                rospy.logwarn("Curr state %s, prev state %s, next state %s", self.curr_state, self.prev_state, self.next_state)
                if self.recovery_info['counter'] > 0:
                    self.recovery_info['counter'] = 0

            if self.coverage == 100.0:
                self.next_state = 'done'

            # Update macchina a stati
            self.prev_state = self.curr_state
            self.curr_state = self.next_state


            self.rate.sleep()


if __name__ == '__main__':
    try:
        nbv_planner = NbvPlanner()
        nbv_planner.run()
    except rospy.ROSInterruptException:
        pass
