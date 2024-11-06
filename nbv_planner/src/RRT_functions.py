import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
import tf
import rospy

def generate_pose_marker(poses, color=(1.0, 0.0, 0.0), scale=1.0):
    marker_array = MarkerArray()

    if len(np.shape(poses)) == 1:
        poses = [tuple(poses)]  # Converte una singola posa in una lista di tuple
    
    for i, (x, y, theta) in enumerate(poses):
        # Crea un nuovo marker di tipo ARROW
        marker = Marker()
        marker.header.frame_id = "map"  # Puoi cambiare il frame se necessario
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = i  # Assicurati che ogni marker abbia un ID unico
        
        # Posizione
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0  # Posizione z fissata a 0 per una visualizzazione 2D

        # Orientamento (calcola da theta in quaternioni)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        marker.pose.orientation = Quaternion(*quaternion)
        
        # Configurazione dell'aspetto della freccia
        marker.scale.x = 0.5 * scale # Lunghezza della freccia
        marker.scale.y = 0.1 * scale # Spessore della freccia
        marker.scale.z = 0.1 * scale # Altezza della freccia
        
        marker.color.a = 1.0  # Opacita
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        # Aggiungi il marker al MarkerArray
        marker_array.markers.append(marker)
    
    return marker_array

def wrapToPi(angle):
    """
    Converte l'angolo di input nell'intervallo [-pi, pi].
    Parametri:
        angle (float): L'angolo di input da convertire in radianti
    Ritorna:
        float: L'angolo convertito nell'intervallo [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def generate_sequence(tree, goal_id):
    """ 
    Dato un albero RRT e l'ID di un nodo obiettivo, genera una sequenza di pose dal nodo iniziale al nodo obiettivo.
    Args:
        tree (dict): Dizionario che rappresenta l'albero RRT. Le chiavi sono indici dei nodi e i valori sono dizionari con informazioni sul nodo.
        goal_id (int): L'ID del nodo obiettivo.
    Returns:
        list: Una lista di np.array che rappresentano la sequenza di pose dal nodo iniziale (escluso) al nodo obiettivo.
    """
    sequence = []
    curr_id = goal_id
    while curr_id != 0:
        sequence.append(tree[curr_id]['pose'])
        curr_id = tree[curr_id]['parent']

    return sequence[::-1]

def update_tree(tree, pose, twist, parent, path):
    """
    Aggiorna l'albero RRT con un nuovo nodo.
    Args:
        tree (dict): Dizionario che rappresenta l'albero RRT. Le chiavi sono indici dei nodi e i valori sono dizionari con informazioni sul nodo.
        pose (tuple): Posa del nuovo nodo da aggiungere all'albero.
        twist (tuple): Velocita` del nuovo nodo da aggiungere all'albero.
        parent (int): Indice del nodo genitore nel dizionario dell'albero.
        path (list): Lista di tuple che rappresentano il percorso dal nodo genitore al nuovo nodo.
    Returns:
        None
    """
    
    N = len(tree)
    cost = 0
    if len(path) > 0:
        curr_position = path[0][:2]
        
        for i in range(1, len(path)):
            next_position = path[i][:2]
            cost += np.linalg.norm(curr_position - next_position)
            curr_position = next_position
        
        
    tree[N] = {'pose': pose, 'twist': twist, 'parent': parent, 'cost': cost, 'path': path}

def boundary_check(workspace, x, y):
    """
    Verifica se un punto (x, y) si trova all'interno dei confini specificati del workspace.
    Args:
        workspace (tuple): Una tupla di quattro valori (xmin, ymin, xmax, ymax) che definiscono i confini del workspace.
        x (float): Coordinata x del punto da verificare.
        y (float): Coordinata y del punto da verificare.
    Returns:
        bool: True se il punto (x, y) si trova all'interno dei confini del workspace, altrimenti False.
    """
    
    if workspace[0] <= x <= workspace[2] and workspace[1] <= y <= workspace[3]:
        return True
    return False

def generate_random_state(R, curr_state, workspace, map):
    """
    Genera uno stato casuale all'interno di un raggio specificato attorno allo stato corrente.
    Args:
        R (float): Il raggio entro il quale generare lo stato casuale.
        curr_state (array-like): Lo stato corrente rappresentato come una coppia di coordinate (x, y).
        workspace (object): L'oggetto che rappresenta lo spazio di lavoro, utilizzato per verificare i confini.
        map (object): L'oggetto mappa che fornisce metodi per convertire coordinate e controllare celle.
    Returns:
        numpy.ndarray: Un array contenente le coordinate (x, y) dello stato casuale generato.
    """

    Cx = curr_state[0]
    Cy = curr_state[1]
    
    sample_found = False

    while not sample_found:
        alpha = 2 * np.pi * np.random.rand()
        r = R * np.sqrt(np.random.rand())
        x = r * np.cos(alpha) + Cx
        y = r * np.sin(alpha) + Cy
        
        if boundary_check(workspace, x, y):
            cell = map.world_to_grid([x,y])
            
            if map.get_cell(cell) == 0:
                sample_found = True

    return np.array([x, y])

def generate_new_node(nearest_pose, nearest_twist, sample, T_max, unicycle_params, control_params, map, samples_map, workspace, dt=0.1):
    """
    Genera un nuovo nodo a partire dallo stato piu` vicino e un campione.
    Args:
        nearest_pose (tuple): Lo stato piu` vicino sotto forma di (x, y, yaw).
        sample (tuple): Il campione target sotto forma di (x, y).
        T_max (float): Il tempo massimo per l'integrazione.
        unicycle_params (dict): Parametri del modello unicycle, contenente:
            - 'v_max' (float): Velocita` massima.
            - 'omega_max' (float): Velocita` angolare massima.
            - 'K_omega' (float): Guadagno per il controllo della velocita` angolare.
            - 'dt' (float): Passo di tempo per l'integrazione.
        map (object): Oggetto mappa che fornisce metodi per la conversione e il controllo delle celle.
        workspace (object): Oggetto workspace che definisce i confini dell'area di lavoro.
    Returns:
        tuple: Una tupla contenente:
            - new_node (numpy.ndarray): Il nuovo nodo generato sotto forma di (x, y, yaw).
            - path_to_node (numpy.ndarray): Il percorso verso il nuovo nodo come array di stati.
            - node_found (bool): Indica se il nodo e` stato trovato correttamente.
    """
    
    # Import parametri del modello unicycle
    v_surge_max = unicycle_params['v_surge_max']
    omega_max   = unicycle_params['omega_max']

    K_yaw       = control_params['yaw']
    K_omega     = control_params['omega']
    K_surge     = control_params['v_surge']

    min_distance = v_surge_max * dt

    # Estraggo le coordinate del nodo piu` vicino
    x, y, yaw = nearest_pose
    v_surge, omega = nearest_twist
    # Inizializzo il percorso dal nodo piu` vicino
    path_to_node = np.array(nearest_pose)
    
    # Init flag node_found
    node_found = False
    new_node_pose = nearest_pose

    # Simulo la dinamica del modello di Zeno per un tempo massimo T_max
    for _ in np.arange(0, T_max, dt):
        
        # Check minima distanza
        distance_from_node = np.linalg.norm(sample[:2] - np.array([x,y]))
        if distance_from_node <= min_distance:

            # Controllo che i campioni siano in una cella libera della samples_map
            # che ha un raggio di inflazione piu` grande
            if samples_map.get_cell(samples_map.world_to_grid(sample)) == -1 or samples_map.get_cell(samples_map.world_to_grid(sample)) == 100:
                rospy.logwarn("Sample in cella occupata o sconosciuta")
                return np.array([]), np.array([]), np.array([]), False
            
            new_node_pose = np.array([x, y, yaw])
            new_node_twist = np.array([v_surge, omega])
            return new_node_pose, new_node_twist, path_to_node, node_found

        

        # Calcolo la direzione verso il campione
        yaw_des = np.arctan2(sample[1] - y, sample[0] - x)
        yaw_diff = wrapToPi(yaw_des - yaw)
        
        # Controllo proporzionale della velocita` angolare
        alpha = K_yaw * yaw_diff - K_omega * omega
        omega = np.clip(omega + alpha/100 * dt, -omega_max, omega_max)
        yaw = wrapToPi(yaw + omega*dt)

        a_surge = K_surge * (v_surge_max - v_surge)
        v_surge = np.clip(v_surge + a_surge * dt, -v_surge_max, v_surge_max)

        # Update stato
        x += v_surge_max*np.cos(yaw)*dt
        y += v_surge_max*np.sin(yaw)*dt

        # Check path fuori dai confini workspace
        if not boundary_check(workspace, x, y):
            return np.array([]), np.array([]), np.array([]), False
        
        # Check collisione con cella occupata o sconosciuta  
        cell = map.world_to_grid([x, y])
        if map.get_cell(cell) == -1 or map.get_cell(cell) == 100:
            return np.array([]), np.array([]), np.array([]), False

        # Aggiorno il percorso al nuovo stato
        path_to_node = np.vstack([path_to_node, np.array([x, y, yaw]) ])
        node_found = True


    # Se il percorso e` stato completato, restituisci il nuovo nodo
    new_node_pose = np.array([x, y, yaw])

    # Controllo che il nodo sia nella mappa con il raggio di inflazione piu` grande
    new_node_cell = samples_map.world_to_grid([x, y])
    if samples_map.get_cell(new_node_cell) == -1 or samples_map.get_cell(new_node_cell) == 100:
        return np.array([]), np.array([]), np.array([]), False
    
    new_node_twist = np.array([v_surge, omega])

    return new_node_pose, new_node_twist, path_to_node, node_found

def find_nearest_node(sample, tree):
    """
    Trova il nodo piu` vicino in un albero di ricerca RRT.
    Args:
        sample (numpy.ndarray): Un array numpy che rappresenta il campione di stato (coordinate x, y, ...).
        tree (list of dict): Una lista di dizionari che rappresenta l'albero di ricerca RRT. Ogni dizionario contiene almeno una chiave 'state' che rappresenta lo stato del nodo (coordinate x, y, ...).
    Returns:
        tuple: Una tupla contenente l'indice del nodo piu` vicino, lo stato del nodo piu` vicino e il twist del nodo piu` vicino.
    """

    N = len(tree)
    distances = np.zeros(N)
    for i in range(N):
        node_state = tree[i]['pose']
        distances[i] = np.linalg.norm(sample[:2] - node_state[:2])

    nearest_id      = int(np.argmin(distances))
    nearest_pose    = tree[nearest_id]['pose']
    nearest_twist   = tree[nearest_id]['twist']

    return nearest_id, nearest_pose, nearest_twist

def generate_RRT_samples(workspace, map, samples_map, curr_pose, sampling_params, kinematics_params, control_params):
    """
    Genera campioni per l'algoritmo RRT.
    Args:
        workspace (list): Lista di quattro valori [xmin, ymin, xmax, ymax] che definiscono i confini del workspace.
        map (object): Oggetto mappa che fornisce metodi per la conversione e il controllo delle celle.
        curr_pose (numpy.ndarray): Array numpy che rappresenta la posa corrente del robot.
        sampling_params (dict): Dizionario contenente i parametri di campionamento, tra cui:
            - 'R' (float): Raggio massimo per la generazione di campioni.
            - 'N_samples' (int): Numero di campioni da generare.
            - 'T_max' (float): Tempo massimo per l'integrazione del modello unicycle.

        kinematics_params (dict): Parametri del cinamtici modello, contenente:
            - 'v_surge_max' (float): Velocita` massima.
            - 'v_sway_max' (float): Velocita` laterale massima.
            - 'omega_max' (float): Velocita` angolare massima.
        
        control_params (dict): Parametri del controllore, contenente:
            - 'yaw' (float): Guadagno per il controllo della direzione.
            - 'omega' (float): Guadagno per il controllo della velocita` angolare.
            - 'surge' (float): Guadagno per il controllo della velocita` longitudinale.
            - 'sway' (float): Guadagno per il controllo della velocita` later

    Returns:
        - tree (dict): Dizionario che rappresenta l'albero RRT. Le chiavi sono gli ID dei nodi e i valori sono dizionari con informazioni sul nodo.
        - samples (numpy.ndarray): Un array numpy che contiene i campioni
    """
    # Parametri campionamento
    R           = sampling_params['R']
    N_samples   = sampling_params['N_samples']
    T_max       = sampling_params['T_max']
    max_counter = sampling_params['max_counter']

    # Init tree con solo il nodo iniziale
    tree = {}
    twist = np.array([0.0, 0.0])
    update_tree(tree, curr_pose, twist, -1, np.array([]))
    counter = 0
    # Generazione N_samples campioni
    while len(tree) <= N_samples and counter < max_counter:
        
        # Genero campione casuale in una cella ammissibile [libera e dentro il workspace]
        # In questa fase uso una mappa con un raggio di inflazione leggermente piu` grande
        sample = generate_random_state(R, curr_pose, workspace, samples_map)
        counter += 1

        # Cerco il nodo piu` vicino all'interno dell'albero [distanza euclidea]
        nearest_id, nearest_pose, nearest_twist = find_nearest_node(sample, tree)
        
        # Cerco di generare un nuovo nodo valido verso il campione
        # In questa fase uso la mappa con il raggio di inflazione piu` piccolo
        new_node_pose, new_node_twist, path_to_node, node_found = generate_new_node(nearest_pose, nearest_twist, sample, T_max, kinematics_params, control_params, map, samples_map, workspace)
        
        # Se il nodo e` valido, lo aggiungo all'albero, altrimenti continuo con il prossimo campione
        if node_found:
            update_tree(tree, new_node_pose, new_node_twist, nearest_id, path_to_node)
            
    
    # Array con tutti i campioni generati
    samples = np.array([tree[i]['pose'] for i in tree])

    return tree, samples