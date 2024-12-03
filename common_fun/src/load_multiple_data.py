import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from obstacle_simulation import ShipObstacle

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

    # categories = ['Nessun errore', 'Uscita dai confini', 'Collisioni']
    categories = ['USCITA DAI CONFINI', 'COLLISIONI']
    reduced_sim_data = sim_data
    empty_count = sum(len(abort) == 0 for abort in abort_values)
    if empty_count == N_sim:
        # return [100, 0, 0], categories, reduced_sim_data
        return [0, 0], categories, reduced_sim_data
    

       
    # Contatori per i fallimenti
    out_of_bounds_count = 0
    collision_count = 0
    filtered_sim_data = []  # Nuovo array per simulazioni valide

    for i, (data, abort) in enumerate(zip(sim_data, abort_values)):
        if len(abort) == 0:
            filtered_sim_data.append(data)  # Simulazioni senza errori
        else:
            # Controlla i tipi di errori
            if any(msg == b'Out of bounds' for msg in abort):
                out_of_bounds_count += 1
            elif any(msg == b'Collision' for msg in abort):
                collision_count += 1
                print(f"Simulazione {i} eliminata per collisione")
            else:
                filtered_sim_data.append(data)  # Mantieni dati non problematici

    # Calcola le percentuali
    percentages = [(out_of_bounds_count / N_sim) * 100, (collision_count / N_sim) * 100]    
    # percentages = [
    #     (empty_count / N_sim) * 100,
    #     (out_of_bounds_count / N_sim) * 100,
    #     (collision_count / N_sim) * 100,
    # ]

    return percentages, categories, filtered_sim_data

def plot_ship(center, scale):
    ship = ShipObstacle(ship_center=center, scale=scale, use_custom_ship=True)
    points = ship.points
    x = [point[0] for point in points]
    y = [point[1] for point in points]
    x.append(x[0])
    y.append(y[0])
    plt.plot(x, y, 'k-')

if __name__ == '__main__':
    TEST = 3
    SHOW_PLOTS = False

    directory1 = '/home/paolo/NBV_data'
    directory2 = '/home/paolo/RL_data'
    if TEST > 1:
        directory1 = directory1 + str(TEST)
        directory2 = directory2 + str(TEST)

    print(directory1)
    print(directory2)

    # title_label = " (Test " + str(TEST) + ")"
    title_label = " T" + str(TEST)
    font = 25

    N_sim1 = 25
    N_sim2 = 25
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



    """ Dati abort mission """
    percentages1, categories1, sim_data1 = find_failure_percentage(sim_data1, N_sim1)
    percentages2, categories2, sim_data2 = find_failure_percentage(sim_data2, N_sim2)
    # prectentages1 = percentages1[1:]
    # percentages2 = percentages2[1:]
    # categories1 = categories1[1:]
    # categories2 = categories2[1:]

    N_sim1 = len(sim_data1)
    N_sim2 = len(sim_data2)
    x = np.arange(len(categories1))  # Posizioni delle categorie
    bar_width = 0.35  # Larghezza delle barre

    plt.figure(figsize=(8,8))

    # Barre per il primo dataset
    plt.bar(x - bar_width / 2, percentages1, bar_width, label='NBV', color='blue')
    plt.bar(x + bar_width / 2, percentages2, bar_width, label='DDPG', color='green')
    #  Aggiunta delle etichette percentuali sopra ogni barra
    for i, p in enumerate(percentages1):
        plt.text(x[i] - bar_width / 2, p + 1, f'{round(p,1)}%', ha='center', va='bottom', fontsize=font, color='blue')

    for i, p in enumerate(percentages2):
        plt.text(x[i] + bar_width / 2, p + 1, f'{round(p,1)}%', ha='center', va='bottom', fontsize=font, color='green')

    # Aggiunta di etichette e titolo
    # plt.ylabel('Percentuali %', fontsize=font)
    plt.title('PERCENTUALI AVVENIMENTI' + str(title_label), fontsize=font)
    plt.yticks(fontsize=font)
    plt.xticks(x, categories1, fontsize=font)  # Imposta le categorie come etichette sull'asse x
    plt.ylim(0, 120)  # Imposta il limite massimo al 100%
    plt.legend(fontsize=font)  # Mostra la legenda
    plt.grid(axis='y', linestyle='--')  # Aggiunge la griglia sull'asse y
    plt.tight_layout()
    ax = plt.gca()  # Ottieni l'oggetto Axes corrente
    for spine in ax.spines.values():
        spine.set_visible(False)
    plt.show()


    



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
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.grid(True)
    plt.title('Andamento medio della copertura del perimetro nel tempo' + str(title_label), fontsize=16)
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

    # plt.figure(figsize=(8,8))
    # plt.boxplot(
    #     [cum_distances1, cum_distances2],  # Lista di dati
    #     vert=True,
    #     patch_artist=True,
    #     boxprops=dict(facecolor="lightblue", alpha=0.7)
    # )
    # plt.ylabel("Distanza Percorsa [m]", fontsize=16)
    # plt.xticks([1, 2], ["NBV", "DDPG"], fontsize=16)  # Due boxplot
    # plt.title("Distribuzione delle distanze percorse per completare la missione", fontsize=16) 
    # plt.grid(axis='y', linestyle='--', alpha=0.5)
    # # plt.show()

    # ----------------------- DOPPIO SUBPLOT -----------------------
    fig, axes = plt.subplots(1, 2, figsize=(16, 6), gridspec_kw={'width_ratios': [2, 1]})  # A sinistra 3x, a destra 1x
    # Subplot 1: Boxplot originale con entrambe le distribuzioni
    axes[0].boxplot(
        [cum_distances1, cum_distances2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )

    # Calcola e stampa valore medio e deviazione standard
    mean1 = np.mean(cum_distances1)
    std1 = np.std(cum_distances1)
    mean2 = np.mean(cum_distances2)
    std2 = np.std(cum_distances2)
    print("Distanza percorsa:")
    print(f"NBV - Valore medio: {mean1:.2f}, Deviazione standard: {std1:.2f}")
    print(f"DDPG - Valore medio: {mean2:.2f}, Deviazione standard: {std2:.2f}")

    axes[0].set_ylabel("Distanza Percorsa [m]", fontsize=16)
    axes[0].set_xticks([1, 2])
    axes[0].set_xticklabels(["NBV", "DDPG"], fontsize=16)  # Due boxplot
    axes[0].set_title("Distribuzione delle distanze percorse per completare la missione" + str(title_label), fontsize=16)
    axes[0].grid(axis='y', linestyle='--', alpha=0.5)
    axes[0].tick_params(axis='both', which='major', labelsize=16)  # Xticks e Yticks fontsize

    # Subplot 2: Solo i dati di cum_distances2
    axes[1].boxplot(
        [cum_distances2],  # Solo cum_distances2
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightgreen", alpha=0.7)
    )
    axes[1].set_ylabel("Distanza Percorsa [m]", fontsize=16)
    axes[1].set_xticks([1])
    axes[1].set_xticklabels(["DDPG"], fontsize=16)  # Un solo boxplot
    axes[1].set_title("Zoom boxplot DDPG", fontsize=16)
    axes[1].grid(axis='y', linestyle='--', alpha=0.5)
    axes[1].tick_params(axis='both', which='major', labelsize=16)  # Xticks e Yticks fontsize
    plt.tight_layout()
    # plt.show()


    """ Dati tempo di missione """
    durations1 = []
    for i in range(N_sim1):
        durations1.append(coverage_timestamps1[i][-1] - coverage_timestamps1[i][0])
    
    durations2 = []
    for i in range(N_sim2):
        durations2.append(coverage_timestamps2[i][-1] - coverage_timestamps2[i][0])
    

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), gridspec_kw={'width_ratios': [2, 1]})  # Larghezze [2, 1]

    # ----------------------- DOPPIO SUBPLOT -----------------------
    axes[0].boxplot(
        [durations1, durations2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )
    mean1 = np.mean(durations1)
    std1 = np.std(durations1)
    mean2 = np.mean(durations2)
    std2 = np.std(durations2)
    print("Tempo di missione:")
    print(f"NBV - Valore medio: {mean1:.2f}, Deviazione standard: {std1:.2f}")
    print(f"DDPG - Valore medio: {mean2:.2f}, Deviazione standard: {std2:.2f}")

    axes[0].set_ylabel("TEMPO [s]", fontsize=font)
    axes[0].set_xticks([1, 2])
    axes[0].set_xticklabels(["NBV", "DDPG"], fontsize=font)  # Due boxplot
    axes[0].set_title("TEMPI DI MISSIONE" + str(title_label), fontsize=font)
    axes[0].grid(axis='y', linestyle='--', alpha=0.5)
    axes[0].tick_params(axis='both', which='major', labelsize=font)  # Xticks e Yticks fontsize

    # Subplot 2: Solo i dati di durations2
    axes[1].boxplot(
        [durations2],  # Solo durations2
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightgreen", alpha=0.7)
    )
    axes[1].set_ylabel("TEMPO [s]", fontsize=font)
    axes[1].set_xticks([1])
    axes[1].set_xticklabels(["DDPG"], fontsize=font)  # Un solo boxplot
    axes[1].set_title("ZOOM DDPG", fontsize=font)
    axes[1].grid(axis='y', linestyle='--', alpha=0.5)
    axes[1].tick_params(axis='both', which='major', labelsize=font)  # Xticks e Yticks fontsize
    plt.tight_layout()
    # plt.show()



    # plt.figure(figsize=(8, 8))
    # plt.boxplot(
    #     [durations1, durations2],  # Lista di dati
    #     vert=True,
    #     patch_artist=True,
    #     boxprops=dict(facecolor="lightblue", alpha=0.7)
    # )

    # # Etichette degli assi
    # plt.ylabel("Tempo [s]", fontsize=16)
    # plt.xticks([1, 2], ["NBV", "DDPG"], fontsize=16)  # Due boxplot
    # plt.title("Distribuzione dei tempi necessari per completare la missione", fontsize=16)
    # plt.grid(axis='y', linestyle='--', alpha=0.5)
    # # plt.show()


    """ Dati tempo speso a cercare la nave """
    time_wasted1 = find_time_wasted(coverage1, coverage_timestamps1, N_sim1)
    time_wasted2 = find_time_wasted(coverage2, coverage_timestamps2, N_sim2)

    # ----------------------- DOPPIO SUBPLOT -----------------------

    fig, axes = plt.subplots(1, 2, figsize=(16, 6), gridspec_kw={'width_ratios': [2, 1]})  # Larghezze [2, 1]

    # Subplot 1: Boxplot originale con entrambe le distribuzioni
    axes[0].boxplot(
        [time_wasted1, time_wasted2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )
    mean1 = np.mean(time_wasted1)
    std1 = np.std(time_wasted1)
    mean2 = np.mean(time_wasted2)
    std2 = np.std(time_wasted2)
    print("Tempo speso a cercare la nave:")
    print(f"NBV - Valore medio: {mean1:.2f}, Deviazione standard: {std1:.2f}")
    print(f"DDPG - Valore medio: {mean2:.2f}, Deviazione standard: {std2:.2f}")

    axes[0].set_ylabel("Tempo speso a cercare la nave [s]", fontsize=16)
    axes[0].set_xticks([1, 2])
    axes[0].set_xticklabels(["NBV", "DDPG"], fontsize=16)
    axes[0].set_title("Distribuzione dei tempi spesi a cercare la nave" + str(title_label), fontsize=16)
    axes[0].grid(axis='y', linestyle='--', alpha=0.5)
    axes[0].tick_params(axis='both', which='major', labelsize=16)

    # Subplot 2: Solo i dati di time_wasted2
    axes[1].boxplot(
        [time_wasted2],  # Solo time_wasted2
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightgreen", alpha=0.7)
    )
    axes[1].set_ylabel("Tempo speso a cercare la nave [s]", fontsize=16)
    axes[1].set_xticks([1])
    axes[1].set_xticklabels(["DDPG"], fontsize=16)
    axes[1].set_title("Zoom boxplot DDPG", fontsize=16)
    axes[1].grid(axis='y', linestyle='--', alpha=0.5)
    axes[1].tick_params(axis='both', which='major', labelsize=16)
    plt.tight_layout()
    # plt.show()


    # plt.figure(figsize=(8,8))
    # plt.boxplot(
    #     [time_wasted1, time_wasted2],  # Lista di dati
    #     vert=True,
    #     patch_artist=True,
    #     boxprops=dict(facecolor="lightblue", alpha=0.7)
    # )

    # plt.ylabel("Tempo speso a cercare la nave [s]", fontsize=16)
    # plt.xticks([1, 2], ["NBV", "DDPG"], fontsize=16)  # Due boxplot
    # plt.title("Distribuzione dei tempi spesi a cercare la nave", fontsize=16)
    # plt.grid(axis='y', linestyle='--', alpha=0.5)
    # plt.show()


    """ Dati heatmap """
    workspace1 = [data['workspace'] for data in sim_data1]
    workspace2 = [data['workspace'] for data in sim_data2]
    
    x1 = [data['x'] for data in sim_data1]
    y1 = [data['y'] for data in sim_data1]

    x2 = [data['x'] for data in sim_data2]
    y2 = [data['y'] for data in sim_data2]

    # Lista per contenere i risultati traslati
    x1_trans = []
    y1_trans = []
    # Itera su ciascuna simulazione
    for i in range(N_sim1):
        xmin, ymin, _, _ = workspace1[i]  # Estrai xmin e ymin dal workspace
        x_translated = x1[i] - xmin       # Trasla x rispetto a xmin
        y_translated = y1[i] - ymin       # Trasla y rispetto a ymin
        x1_trans.append(x_translated)     # Aggiungi i risultati traslati alla lista
        y1_trans.append(y_translated)     # Aggiungi i risultati traslati alla lista
        # plt.figure()
        # plt.plot(x_translated, y_translated, label='NBV')
        # plt.show()

    x2_trans = []
    y2_trans = []
    # Itera su ciascuna simulazione
    for i in range(N_sim2):
        xmin, ymin, _, _ = workspace2[i]  # Estrai xmin e ymin dal workspace
        x_translated = x2[i] - xmin       # Trasla x rispetto a xmin
        y_translated = y2[i] - ymin       # Trasla y rispetto a ymin
        x2_trans.append(x_translated)     # Aggiungi i risultati traslati alla lista
        y2_trans.append(y_translated)     # Aggiungi i risultati traslati alla lista


    for i in range(10):
        print(i)
        xnbv = x1_trans[i]
        ynbv = y1_trans[i]
        xddpg = x2_trans[i]
        yddpg = y2_trans[i]
        ship_center = sim_data1[i]['ship_center']
        ship_scale_factor = sim_data1[i]['ship_scale_factor']
        ship_center[0] -= workspace1[i][0]
        ship_center[1] -= workspace1[i][1]
        plt.figure(figsize=(8,8))
        plt.plot(xnbv, ynbv, label="NBV")
        plt.plot(xddpg, yddpg, label="DDPG")
        plot_ship(ship_center, ship_scale_factor)
        plt.legend(fontsize=font)
        plt.grid(True)
        plt.axis('equal')
        plt.title("Test 2", fontsize=20)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
    plt.show()



   # Calcolo delle lunghezze medie lungo x e y per il primo dataset
    x_lengths1 = [ws[2] - ws[0] for ws in workspace1]  # x_max - x_min
    y_lengths1 = [ws[3] - ws[1] for ws in workspace1]  # y_max - y_min

    # Calcolo delle lunghezze medie lungo x e y per il secondo dataset
    x_lengths2 = [ws[2] - ws[0] for ws in workspace2]
    y_lengths2 = [ws[3] - ws[1] for ws in workspace2]

    # Lunghezze medie complessive
    mean_x_length = np.mean(x_lengths1 + x_lengths2)
    mean_y_length = np.mean(y_lengths1 + y_lengths2)

    # Concatenazione dei dati traslati
    x_all1 = np.concatenate(x1_trans)
    y_all1 = np.concatenate(y1_trans)

    x_all2 = np.concatenate(x2_trans)
    y_all2 = np.concatenate(y2_trans)

    # Definizione della griglia per entrambe le heatmap
    x_bins = np.linspace(0, mean_x_length, 50)  # 50 bin lungo x
    y_bins = np.linspace(0, mean_y_length, 50)  # 50 bin lungo y

    # Calcolo della densità dei punti nella griglia (per dataset 1)
    heatmap1, xedges, yedges = np.histogram2d(x_all1, y_all1, bins=[x_bins, y_bins], density=True)
    heatmap1 = heatmap1.T  # Trasposizione per visualizzazione corretta

    # Calcolo della densità dei punti nella griglia (per dataset 2)
    heatmap2, _, _ = np.histogram2d(x_all2, y_all2, bins=[x_bins, y_bins], density=True)
    heatmap2 = heatmap2.T  # Trasposizione per visualizzazione corretta

    # # Visualizzazione delle heatmap
    # fig, axes = plt.subplots(1, 2, figsize=(16, 6), sharex=True, sharey=True)

    # # Heatmap per il primo dataset
    # im1 = axes[0].imshow(
    #     heatmap1,
    #     extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
    #     origin='lower',
    #     aspect='equal',
    #     cmap='plasma'
    # )
    # axes[0].set_title('Heatmap traiettorie NBV', fontsize=16)
    # axes[0].set_xlabel('X', fontsize=16)
    # axes[0].set_ylabel('Y', fontsize=16)
    # axes[0].tick_params(axis='both', which='major', labelsize=16)

    # # Heatmap per il secondo dataset
    # im2 = axes[1].imshow(
    #     heatmap2,
    #     extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
    #     origin='lower',
    #     aspect='equal',
    #     cmap='plasma'
    # )
    # axes[1].set_title('Heatmap traiettorie DDPG', fontsize=16)
    # axes[1].set_xlabel('X', fontsize=16)
    # axes[1].tick_params(axis='both', which='major', labelsize=16)
    # axes[1].yaxis.set_ticks_position('left')  # Mostra i tick sull'asse Y
    # axes[1].set_yticks(np.linspace(yedges[0], yedges[-1], 5))  # Tick manuali

    # # Aggiungi la colorbar al grafico a destra
    # fig.colorbar(im2, ax=axes[1], label='Density')

    # # Layout più pulito
    # plt.tight_layout()
    # # plt.show()

    # Ruota le heatmap di 90 gradi in senso antiorario
    heatmap1_rotated = np.rot90(heatmap1)  # Ruota la prima heatmap
    heatmap2_rotated = np.rot90(heatmap2)  # Ruota la seconda heatmap
    heatmap1_rotated = np.rot90(heatmap1_rotated)  # Ruota la prima heatmap
    heatmap2_rotated = np.rot90(heatmap2_rotated)  # Ruota la seconda heatmap
    heatmap1_rotated = np.rot90(heatmap1_rotated)  # Ruota la prima heatmap
    heatmap2_rotated = np.rot90(heatmap2_rotated)  # Ruota la seconda heatmap
    # Visualizzazione delle heatmap
    fig, axes = plt.subplots(1, 2, figsize=(16, 6), sharex=True, sharey=True)
    # Heatmap per il primo dataset (ruotata)
    im1 = axes[0].imshow(
        heatmap1_rotated,
        extent=[yedges[0], yedges[-1], xedges[0], xedges[-1]],  # Scambia x e y
        origin='lower',
        aspect='equal',
        cmap='plasma'
    )
    axes[0].set_title('Heatmap traiettorie NBV' + str(title_label), fontsize=16)
    # axes[0].set_xlabel('Y', fontsize=16)  # Cambia etichetta asse
    # axes[0].set_ylabel('X', fontsize=16)
    # axes[0].tick_params(axis='both', which='major', labelsize=16)
    axes[0].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    # Heatmap per il secondo dataset (ruotata)
    im2 = axes[1].imshow(
        heatmap2_rotated,
        extent=[yedges[0], yedges[-1], xedges[0], xedges[-1]],  # Scambia x e y
        origin='lower',
        aspect='equal',
        cmap='plasma'
    )
    axes[1].set_title('Heatmap traiettorie DDPG' + str(title_label), fontsize=16)
    # axes[1].set_xlabel('Y', fontsize=16)  # Cambia etichetta asse
    # axes[1].tick_params(axis='both', which='major', labelsize=16)
    # axes[1].yaxis.set_ticks_position('left')
    # axes[1].set_yticks(np.linspace(xedges[0], xedges[-1], 5))  # Tick manuali per asse X (ora Y ruotato)
    axes[1].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    # Aggiungi la colorbar al grafico a destra
    fig.colorbar(im2, ax=axes[1], label='Density')

    # Layout più pulito
    plt.tight_layout()
    if SHOW_PLOTS:
        plt.show()
    # plt.show()
