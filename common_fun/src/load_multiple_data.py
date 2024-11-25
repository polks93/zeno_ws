import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


def smooth(data, window_size):
    # """Applica una media mobile a un array."""
    # window = np.ones(window_size) / window_size
    # return np.convolve(data, window, mode='valid')
    smoothed_data = np.zeros(len(data))  # Inizializza l'array per i risultati
    
    for i in range(len(data)):
        # Calcola il numero di elementi da considerare (finestra fino all'inizio)
        start_idx = max(0, i - window_size + 1)
        smoothed_data[i] = np.mean(data[start_idx:i + 1])
    
    return smoothed_data

def fill_with_last_nonzero(data):
    # """Riempi i valori nulli con l'ultimo valore non nullo."""
    last_nonzero = 0
    first_nonzero_idx = 0

    for i in range(len(data)):
        if data[i] != 0:
            first_nonzero_idx = i
            break


    for i in range(first_nonzero_idx, len(data)):
        if data[i] == 0:
            data[i] = last_nonzero
        else:
            last_nonzero = data[i]

    return data

def elaborate_coverage(coverage, coverage_timestamps, max_time=None):
    min_time = 0.0
    if max_time is None:
        max_time = max(np.max(timestamps) for timestamps in coverage_timestamps)
    common_timestamps = np.linspace(min_time, max_time, num=round(max_time))
    interpolated_coverages = []

    for i in range(len(coverage)):
        # Crea una funzione di interpolazione
        interpolation_function = interp1d(
            coverage_timestamps[i],
            coverage[i],
            bounds_error=False,  # Non dare errore fuori dai limiti
            fill_value=(0, 100)  # Permetti l'extrapolazione se necessario
        )
        
        # Interpola i dati sulla griglia temporale comune
        interpolated_coverages.append(interpolation_function(common_timestamps))

    coverage_mean = np.mean(interpolated_coverages, axis=0)
    coverage_std = np.std(interpolated_coverages, axis=0)

    coveragePlusStd = np.clip(coverage_mean + coverage_std, 0, 100)
    coverageMinusStd = np.clip(coverage_mean - coverage_std, 0, 100)

    return common_timestamps, interpolated_coverages, coverage_mean, coveragePlusStd, coverageMinusStd

def calculate_cum_distance(x,y):
    cum_distance = 0
    for i in range(1, len(x)):
        cum_distance += np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)

    return cum_distance


def find_time_wasted(coverage, coverage_timestamps, N_sim):
    time_wasted = []
    for i in range(N_sim):
        for t in range(len(coverage[i])):
            if coverage[i][t] > 0:
                time_wasted.append(coverage_timestamps[i][t])
                break

    return time_wasted

def find_failure_percentage(sim_data, N_sim):
    abort_values = []
    for i, data in enumerate(sim_data):
        if 'abort_values' in data:
            abort_values.append(data['abort_values'])
            # print(f"Simulazione {i}: {data['abort_values']}")
        else:
            abort_values.append(np.array([]))

    categories = ['Nessun errore', 'Uscita dai confini', 'Collisioni']

    empty_count = sum(len(abort) == 0 for abort in abort_values)
    if empty_count == N_sim:
        return [100, 0, 0], categories
    
    else:
        out_of_bounds_count = 0
        for abort in abort_values:
            for msg in abort:
                if msg == b'Out of bounds':
                    out_of_bounds_count += 1
                    break

        collision_count = N_sim1 - empty_count - out_of_bounds_count

        percentages = [
            (empty_count / N_sim1) * 100,
            (out_of_bounds_count / N_sim1) * 100,
            (collision_count / N_sim1) * 100,
        ]
    return percentages, categories
    
if __name__ == '__main__':
    directory1 = '/home/paolo/NBV_data'
    directory2 = '/home/paolo/RL_data'


    N_sim1 = 19
    N_sim2 = 13
    sim_data1 = []
    sim_data2 = []
    for i in range(N_sim1):
        file_name = 'sim_data' + str(i) + '.npz'
        file_path = os.path.join(directory1, file_name)
        data = np.load(file_path)
        sim_data1.append(data)

    for i in range(N_sim2):
        file_name = 'sim_data' + str(i) + '.npz'
        file_path = os.path.join(directory2, file_name)
        data = np.load(file_path)
        sim_data2.append(data)
    
    """ Dati coverage """
    coverage1 = [data['coverage'] for data in sim_data1]
    coverage_timestamps1 = [data['coverage_timestamps'] for data in sim_data1]
    coverage2 = [data['coverage'] for data in sim_data2]
    coverage_timestamps2 = [data['coverage_timestamps'] for data in sim_data2]

    max_time1 = max(np.max(timestamps) for timestamps in coverage_timestamps1)   
    max_time2 = max(np.max(timestamps) for timestamps in coverage_timestamps2)
    max_time = max(max_time1, max_time2)
    common_timestamps1, interpolated_coverages1, coverage_mean1, coveragePlusStd1, coverageMinusStd1 = elaborate_coverage(coverage1, coverage_timestamps1, max_time)
    common_timestamps2, interpolated_coverages2, coverage_mean2, coveragePlusStd2, coverageMinusStd2 = elaborate_coverage(coverage2, coverage_timestamps2, max_time)


    """ Plot copertura """
    plt.figure(figsize=(16, 9))
    plt.plot(common_timestamps1, coverage_mean1, label='NBV')
    plt.fill_between(common_timestamps1, coveragePlusStd1, coverageMinusStd1, alpha=0.5)
    plt.plot(common_timestamps2, coverage_mean2, label='DDPG')
    plt.fill_between(common_timestamps2, coveragePlusStd2, coverageMinusStd2, alpha=0.5)
    plt.legend(fontsize=16)
    plt.xlabel('Tempo [s]', fontsize=16)
    plt.ylabel('Copertura media [%]', fontsize=16)
    plt.title('Andamento medio della copertura del perimetro nel tempo', fontsize=16)
    # plt.show()
    


    """ Dati distanza percorsa """
    x1 = [data['x'] for data in sim_data1]
    y1 = [data['y'] for data in sim_data1]

    x2 = [data['x'] for data in sim_data2]
    y2 = [data['y'] for data in sim_data2]


    cum_distances1 = []
    for i in range(N_sim1):
        cum_distances1.append(calculate_cum_distance(x1[i], y1[i]))

    cum_distances2 = []
    for i in range(N_sim2):
        cum_distances2.append(calculate_cum_distance(x2[i], y2[i]))

    plt.figure(figsize=(8,8))
    plt.boxplot(
        [cum_distances1, cum_distances2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )
    plt.ylabel("Distanza Percorsa [m]", fontsize=16)
    plt.xticks([1, 2], ["NBV", "DDPG"], fontsize=16)  # Due boxplot
    plt.title("Distribuzione delle distanze percorse per completare la missione", fontsize=16) 
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    # plt.show()

    """ Dati tempo di missione """
    durations1 = []
    for i in range(N_sim1):
        durations1.append(coverage_timestamps1[i][-1] - coverage_timestamps1[i][0])
    
    durations2 = []
    for i in range(N_sim2):
        durations2.append(coverage_timestamps2[i][-1] - coverage_timestamps2[i][0])
    
    plt.figure(figsize=(8, 8))
    plt.boxplot(
        [durations1, durations2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )

    # Etichette degli assi
    plt.ylabel("Tempo [s]", fontsize=16)
    plt.xticks([1, 2], ["NBV", "DDPG"], fontsize=16)  # Due boxplot
    plt.title("Distribuzione dei tempi necessari per completare la missione", fontsize=16)
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    # plt.show()


    """ Dati tempo speso a cercare la nave """
    time_wasted1 = find_time_wasted(coverage1, coverage_timestamps1, N_sim1)
    time_wasted2 = find_time_wasted(coverage2, coverage_timestamps2, N_sim2)

    plt.figure(figsize=(8,8))
    plt.boxplot(
        [time_wasted1, time_wasted2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )

    plt.ylabel("Tempo speso a cercare la nave [s]", fontsize=16)
    plt.xticks([1, 2], ["NBV", "DDPG"], fontsize=16)  # Due boxplot
    plt.title("Distribuzione dei tempi spesi a cercare la nave", fontsize=16)
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    # plt.show()


    """ Dati abort mission """
    percentages1, categories1 = find_failure_percentage(sim_data1, N_sim1)
    percentages2, categories2 = find_failure_percentage(sim_data2, N_sim2)

    x = np.arange(len(categories1))  # Posizioni delle categorie
    bar_width = 0.35  # Larghezza delle barre

    plt.figure(figsize=(8,8))

    # Barre per il primo dataset
    plt.bar(x - bar_width / 2, percentages1, bar_width, label='NBV', color='blue')
    plt.bar(x + bar_width / 2, percentages2, bar_width, label='RL', color='orange')

    # Aggiunta di etichette e titolo
    plt.ylabel('Percentuali %', fontsize=16)
    plt.title('Percentuali di avvenimenti', fontsize=16)
    plt.xticks(x, categories1, fontsize=16)  # Imposta le categorie come etichette sull'asse x
    plt.ylim(0, 100)  # Imposta il limite massimo al 100%
    plt.legend(fontsize=16)  # Mostra la legenda
    plt.grid(axis='y', linestyle='--')  # Aggiunge la griglia sull'asse y
    plt.tight_layout()
    # plt.show()



    x_all1 = np.concatenate(x2)
    y_all1 = np.concatenate(y2)

    # Definizione della griglia per la heatmap
    x_bins = np.linspace(min(x_all1), max(x_all1), 50)  # 50 bin lungo x
    y_bins = np.linspace(min(y_all1), max(y_all1), 50)  # 50 bin lungo y

    # Calcolo della densità dei punti nella griglia
    heatmap, xedges, yedges = np.histogram2d(x_all1, y_all1, bins=[x_bins, y_bins], density=True)

    # Trasporre la heatmap per un orientamento corretto
    heatmap = heatmap.T

    # Visualizzazione della heatmap
    plt.figure(figsize=(8, 6))
    plt.imshow(heatmap, extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
    origin='lower', aspect='equal', cmap='viridis')
    plt.colorbar(label='Density')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Heatmap of X and Y Positions')
    plt.show()