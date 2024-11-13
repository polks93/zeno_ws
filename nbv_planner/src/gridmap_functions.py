import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray


class OccupancyGridWrapper:
    def __init__(self, occupancy_grid, workspace_limits):
        """
        Inizializza il wrapper per un oggetto OccupancyGrid di ROS.
        
        Args:
        - occupancy_grid: Messaggio di tipo OccupancyGrid proveniente da ROS.
        """
        self.grid               = occupancy_grid
        self.resolution         = occupancy_grid.info.resolution
        self.origin             = [occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y]
        self.width              = occupancy_grid.info.width
        self.height             = occupancy_grid.info.height
        self.data               = np.array(occupancy_grid.data).reshape((self.height, self.width))
        self.workspace_limits   = workspace_limits
        self.generate_max_indices()

    def generate_max_indices(self):
        """ Calcola le coordinate massime della griglia """
        x_min, y_min, x_max, y_max = self.workspace_limits
        self.min_row, self.min_col = self.world_to_grid([x_min, y_min])
        self.max_row, self.max_col = self.world_to_grid([x_max, y_max])

    def world_to_grid(self, position):
        """
        Converte coordinate mondo (x, y) in coordinate griglia (i, j).
        
        Args:
        - x, y: Coordinate mondo.
        
        Returns:
        - [i, j]: Coordinate della griglia.
        """
        x, y = position
        j = int((x - self.origin[0]) / self.resolution)
        i = int((y - self.origin[1]) / self.resolution)
        return [i, j]

    def grid_to_world(self, cell):
        """
        Converte coordinate griglia (i, j) in coordinate mondo (x, y).
        
        Args:
        - i, j: Coordinate griglia.
        
        Returns:
        - [x, y]: Coordinate mondo del CENTRO della cella.
        """
        i, j = cell
        x = self.origin[0] + (j + 0.5) * self.resolution
        y = self.origin[1] + (i + 0.5) * self.resolution
        return [x, y]

    def get_cell(self, cell):
        """
        Ottiene il valore di una cella nella matrice di occupazione.
        
        Args:
        - i, j: Coordinate della cella nella griglia.
        
        Returns:
        - Valore della cella (0 = libero, 100 = occupato, -1 = sconosciuto).
        """
        i, j = cell
        if 0 <= i < self.height and 0 <= j < self.width:
            return self.data[i, j]
        else:
            return None  # Coordinate fuori dalla mappa

    def set_cell(self, cell, value):
        """
        Imposta il valore di una cella nella matrice di occupazione.
        
        Args:
        - i, j: Coordinate della cella nella griglia.
        - value: Valore da impostare (0 = libero, 100 = occupato, -1 = sconosciuto).
        """
        i, j = cell
        if 0 <= i < self.height and 0 <= j < self.width:
            self.data[i, j] = value
            # Aggiornare i dati nell'OccupancyGrid ROS
            

    def update_ros_grid(self):
        """
        Aggiorna la occupacy grid di ROS con i nuovi dati.
        """
        self.grid.data = self.data.reshape(-1)

    def mini_inflated_gridmap_generation(self, curr_position, inflation_radius, L):
        """
        Crea una mappa inflated considerando tutte le celle occupate e sconosciute
        in un area quadrata di lato L + 2*inflation_radius centrata in curr_position.
        Args:
            curr_position (tuple): La posizione corrente nel mondo (x, y).
            inflation_radius (float): Il raggio di inflazione in metri.
            L (float): La lunghezza del lato del quadrato in cui cercare celle occupate.
        Returns:
            inflated_map (OccupancyGridWrapper): La mappa inflated.
        """

        inflated_map = OccupancyGridWrapper(self.grid, self.workspace_limits)
        occ_cell = set()
        curr_cell = self.world_to_grid(curr_position)

        L_cells = int(np.ceil(L / self.resolution))
        R_cells = int(np.ceil(inflation_radius / self.resolution))
        
        row_start   = max(self.min_row, curr_cell[0] - L_cells)
        row_end     = min(self.max_row, curr_cell[0] + L_cells)
        col_start   = max(self.min_col, curr_cell[1] - L_cells)
        col_end     = min(self.max_col, curr_cell[1] + L_cells)

        # Cerco le celle occupate o sconosciute all'interno del quadrato di lato L + 2 * R
        for i in range(row_start - R_cells, row_end+ R_cells):
            for j in range(col_start - R_cells, col_end + R_cells):
                if self.get_cell([i, j]) == 100 or self.get_cell([i, j]) == -1:

                    # Scorre tutte le celle dentro un quadrato di lato 2 * R
                    for di in range(-R_cells, R_cells + 1):
                        for dj in range(-R_cells, R_cells + 1):
                            ni = i + di
                            nj = j + dj

                            if (ni, nj) in occ_cell:
                                continue

                            if ni <= row_start or ni > row_end or nj <= col_start or nj > col_end:
                                continue

                            distance = np.sqrt(di**2 + dj**2) * self.resolution
                            if distance >= inflation_radius:
                                continue
                            else:
                                occ_cell.add((ni, nj))

        for cell in occ_cell:
            inflated_map.set_cell(cell, 100)

        inflated_map.update_ros_grid()
        return inflated_map
                                    
                            
def lidar_raycast(pose, lidar_params, map):
    """
    Simula un raycast LIDAR su una mappa di occupazione.
    
    Args:
    - pose: Posizione (x, y, theta) del robot.
    - lidar_params: Parametri del LIDAR (numero di raggi, risoluzione angolare, raggio massimo).
    - map: Oggetto della classe OccupancyGridWrapper.
    
    Returns:
    - angles: Angoli dei raggi LIDAR.
    - ranges: Distanze misurate per ciascun raggio.
    - visible_cells: Celle visibili colpite dai raggi.
    """
    # OccupancyGrid contenuta nel wrapped
    data = map.data
    
    # Converto la posizione del robot da mondo a griglia
    x0, y0, theta0 = pose
    theta0 = np.rad2deg(theta0)
    cell0 = map.world_to_grid([x0, y0])
    # x0, y0 = map.grid_to_world(cell0)
    
    # Import parametri utili lidar
    ray_num = lidar_params['ray_num']
    resolution = lidar_params['resolution']
    max_range = lidar_params['max_range']
    
    # Init array per gli angoli e le distanze
    angles = np.zeros([ray_num])
    ranges = max_range * np.ones([ray_num])
    
    # Se la cella iniziale e` occupata o sconosciuta, interrompo il raycast
    if map.get_cell(cell0) == -1 or map.get_cell(cell0) == 100:
        return angles, ranges, None

    # Init set di celle contenenti valori unici delle celle visibili
    visible_cells = set()
    
    # Simulazione del raycast per ciascun raggio
    for i in range(ray_num):
        # Calcolo l'angolo assouluto del raggio i-esimo
        angle = np.deg2rad((i - ray_num/2) * resolution + theta0)
        
        # Salvo il valore dell'angolo assoluto in gradi
        angles[i] = np.rad2deg(angle)
        
        # Calcola l'avanzamento del raggio fino al raggio massimo
        for r in np.arange(0, max_range, 0.01):
            x = x0 + r * np.cos(angle)
            y = y0 + r * np.sin(angle)
            
            # Converte le coordinate mondo in coordinate griglia
            cell = map.world_to_grid([x, y])
            
            # Controlla se il raggio e` uscito dai limiti della griglia
            if cell[0] >= data.shape[0] or cell[1] >= data.shape[1] or cell[0] < 0 or cell[1] < 0:
                break
            
            # Se incontro una cella occupata o sconosciuta, interrompo il raggio
            elif map.get_cell(cell) == 100 or map.get_cell(cell) == -1:
                ranges[i] = r
                visible_cells.add(tuple(cell))
                break
            
            # Altrimenti, aggiungi la cella come visibile
            else:
                visible_cells.add(tuple(cell))

    visible_cells = np.array(list(visible_cells))
    return angles, ranges, visible_cells

def find_visible_cells(pose, lidar_parms, map):

    data = map.data
    x0, y0, theta0 = pose
    cell0 = map.world_to_grid((x0, y0))

    if map.get_cell(cell0) == -1 or map.get_cell(cell0) == 100:
        return None
    
    n_beams     = lidar_parms['n_beams']
    FoV         = lidar_parms['FoV']
    max_range   = lidar_parms['max_range']

    # Angoli assoluti dei raggi
    angles = np.linspace(- FoV / 2, FoV / 2, n_beams) + theta0
    visible_cells = set()

    for i in range(n_beams):

        angle = angles[i]

        for r in np.arange(0, max_range, 0.01):
            x = x0 + r * np.cos(angle)
            y = y0 + r * np.sin(angle)


            # Converte le coordinate mondo in coordinate griglia
            cell = map.world_to_grid([x, y])

            # Controlla se il raggio e` uscito dai limiti della griglia (sono limiti interni della mappa)
            if cell[0] >= data.shape[0] or cell[1] >= data.shape[1] or cell[0] < 0 or cell[1] < 0:
                break
            
            # Controlla se sono ancora dentro il workspace
            if cell[0] <= map.min_row or cell[0] > map.max_row or cell[1] <= map.min_col or cell[1] > map.max_col:
                break

            # Se incontro una cella occupata o sconosciuta, interrompo il raggio
            elif map.get_cell(cell) == 100 or map.get_cell(cell) == -1:
                visible_cells.add(tuple(cell))
                break

            # Altrimenti, aggiungi la cella come visibile
            else:
                visible_cells.add(tuple(cell))

    visible_cells = np.array(list(visible_cells))
    return visible_cells

def find_neighbors(cell, map):
    neighbors = []
    row, col = cell
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            # elif row + i < 0 or row + i > map.height + 1 or col + j < 0 or col + j > map.width + 1:
            #     continue
            elif row + i <= map.min_row + 1 or row + i > map.max_row + 1 or col + j <= map.min_col + 1 or col + j > map.max_col + 1:
                continue
            else:
                neighbors.append((row + i, col + j))
    return neighbors

"""Funzione che trova le celle di frontiera"""
def find_frontier_cells(map, visible_cells):
    frontier_cells = set()
    if visible_cells is not None:
        for cell in visible_cells:
            # Cerco le celle libere in visible_cells
            if map.get_cell(cell) == 0:
                neighbors = find_neighbors(cell, map)
                # Cerco i vicini delle celle libere che siano di occupazione sconosciuta
                for neighbor in neighbors:
                    if map.get_cell(neighbor) == -1:
                        frontier_cells.add(tuple(cell))
                        continue
            
    return frontier_cells

"""Funzione che trova le celle di contorno"""
def find_contour_cells(map, visible_cells):
    contour_cells = set()
    if visible_cells is not None:
        for cell in visible_cells:
            free = False
            occupied = False
            # Cerco le celle sconosciute in visible_cells
            if map.get_cell(cell) == -1:
                neighbors = find_neighbors(cell, map)
                
                # Cerco i vicini delle celle libere che siano di occupazione sconosciuta
                for neighbor in neighbors:
                    if map.get_cell(neighbor) == 0:
                        free = True
                    elif map.get_cell(neighbor) > 50:
                        occupied = True
                
                if free and occupied:
                    contour_cells.add(tuple(cell))
                
    return contour_cells

def find_occ_cells(map, cells):
    occ_cells = set()
    if cells is not None:
        for cell in cells:
            if map.get_cell(cell) > 50:
                occ_cells.add(tuple(cell))
    return occ_cells

def create_markers_msg(occupancy_grid, all_cells, color=[1.0, 0.0, 0.0]):
    """Crea un messaggio MarkerArray a partire dalle celle visibili."""
    marker_array = MarkerArray()

    if all_cells is None:
        return marker_array
    
    if color == (0,1,0):
        z = - occupancy_grid.resolution / 2
    else:
        z = 0.0
    for i, cell in enumerate(all_cells):
        cell_x, cell_y = occupancy_grid.grid_to_world(cell)  # Converti da [riga, colonna] a coordinate del mondo
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "visible_cells"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = cell_x
        marker.pose.position.y = cell_y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = occupancy_grid.resolution / 2
        marker.scale.y = occupancy_grid.resolution / 2
        marker.scale.z = occupancy_grid.resolution / 2 
        marker.color.a = 1  # Opacita
        marker.color.r = color[0]  
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker_array.markers.append(marker)
    return marker_array