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


if __name__ == '__main__':
    directory2 = '/home/paolo/RL_data'

    directory1 = '/home/paolo/NBV_data'
    # directory1 = 'C:/Users/paolo/Documents/Università/JN/DDPG/multi'

    # directory2 = 'C:/Users/paolo/Documents/Università/JN/SHIP/ShipQuest-v1_Data/multi_no_prox'

    N_sim1 = 17
    N_sim2 = 7
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


    # plt.figure()
    # for i in range(N_sim):
    #     plt.plot(coverage_timestamps[i], coverage[i])

    # for i in range(N_sim):
    #     plt.plot(common_timestamps, interpolated_coverages[i])
    #     print(len(interpolated_coverages[i]))



    plt.figure(figsize=(8, 5))
    plt.plot(common_timestamps1, coverage_mean1, label='NBV')
    plt.fill_between(common_timestamps1, coveragePlusStd1, coverageMinusStd1, alpha=0.5)
    plt.plot(common_timestamps2, coverage_mean2, label='DDPG')
    plt.fill_between(common_timestamps2, coveragePlusStd2, coverageMinusStd2, alpha=0.5)
    plt.legend()
    plt.title('Andamento medio della copertura del perimetro nel tempo')
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

    # mean_cum_distance1 = np.mean(cum_distances1)
    # std_cum_distance1 = np.std(cum_distances1)

    # mean_cum_distance2 = np.mean(cum_distances2)
    # std_cum_distance2 = np.std(cum_distances2)

    

    # plt.figure(figsize=(8, 5))
    # plt.boxplot(cum_distances1, vert=True, patch_artist=True, boxprops=dict(facecolor="lightblue", alpha=0.7))
    # plt.boxplot(cum_distances2, vert=True, patch_artist=True, boxprops=dict(facecolor="lightblue", alpha=0.7))
    # plt.ylabel("Distanza Percorsa")
    # plt.xticks([1], ["Simulazioni"])  # Box unico
    # plt.title("Distribuzione delle Distanze Percorse")
    # plt.grid(axis='x', linestyle='--', alpha=0.5)
    # plt.show()
    # Configurazione dei boxplot
    plt.figure(figsize=(8, 5))
    plt.boxplot(
        [cum_distances1, cum_distances2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )

    # Etichette degli assi
    plt.ylabel("Distanza Percorsa [m]")
    plt.xticks([1, 2], ["NBV", "DDPG"])  # Due boxplot
    plt.title("Distribuzione delle distanze percorse per completare la missione") 

    # Griglia
    plt.grid(axis='y', linestyle='--', alpha=0.5)

    # Mostra il grafico
    # plt.show()

    """ Dati tempo di missione """

    durations1 = []
    for i in range(N_sim1):
        durations1.append(coverage_timestamps1[i][-1] - coverage_timestamps1[i][0])
    
    durations2 = []
    for i in range(N_sim2):
        durations2.append(coverage_timestamps2[i][-1] - coverage_timestamps2[i][0])
    

    # Configurazione dei boxplot 2 colonne
    plt.figure(figsize=(8, 5))
    plt.boxplot(
        [durations1, durations2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )

    # Etichette degli assi
    plt.ylabel("Tempo [s]")
    plt.xticks([1, 2], ["NBV", "DDPG"])  # Due boxplot
    plt.title("Distribuzione dei tempi necessari per completare la missione")

    # Griglia
    plt.grid(axis='y', linestyle='--', alpha=0.5)

    # Mostra il grafico
    # plt.show()




    # plt.figure(figsize=(8, 5))
    # plt.boxplot(durations, vert=True, patch_artist=True, boxprops=dict(facecolor="lightblue", alpha=0.7))
    # plt.ylabel("Tempo di Missione")
    # plt.xticks([1], ["Simulazioni"])  # Box unico
    # plt.title("Distribuzione delle Durate delle Missioni")
    # plt.grid(axis='x', linestyle='--', alpha=0.5)
    # plt.show()



    """ Dati tempo speso a cercare la nave """
    time_wasted1 = find_time_wasted(coverage1, coverage_timestamps1, N_sim1)
    time_wasted2 = find_time_wasted(coverage2, coverage_timestamps2, N_sim2)
    # for i in range(N_sim1):
    #     plt.figure(figsize=(8, 5))
    #     plt.plot([time_wasted1[i], time_wasted1[i]],[0, 100])
    #     plt.plot(coverage_timestamps1[i], coverage1[i])
    #     plt.show()


    # Configurazione dei boxplot
    plt.figure(figsize=(8, 5))
    plt.boxplot(
        [time_wasted1, time_wasted2],  # Lista di dati
        vert=True,
        patch_artist=True,
        boxprops=dict(facecolor="lightblue", alpha=0.7)
    )

    plt.ylabel("Tempo speso a cercare la nave [s]")
    plt.xticks([1, 2], ["NBV", "DDPG"])  # Due boxplot
    plt.title("Distribuzione dei tempi spesi a cercare la nave")
    plt.grid(axis='y', linestyle='--', alpha=0.5)

    plt.show()

    """ Plot vecchi altra sim windows """

    # trained_coverage_grouped = [data['trained_coverage'] for data in sim_data1]
    
    # len_ep2 = [data['len_ep'] for data in sim_data2]
    # reward2 = [data['reward'] for data in sim_data2]
    # coverage2 = [data['coverage'] for data in sim_data2]
    # trained_coverage_grouped2 = [data['trained_coverage'] for data in sim_data2]


    # trained_coverage_raw = np.concatenate(trained_coverage_grouped, axis=0)
    # # trained_coverage_raw2 = np.concatenate(trained_coverage_grouped2, axis=0)


    # trained_coverage = []
    # # trained_coverage2 = []

    # for i in range(len(trained_coverage_raw)):
    #     trained_coverage.append(fill_with_last_nonzero(trained_coverage_raw[i]))

    # # for i in range(len(trained_coverage_raw2)):
    # #     trained_coverage2.append(fill_with_last_nonzero(trained_coverage_raw2[i]))


    # window_size = 1
    # len_ep_mean = smooth(np.mean(len_ep, axis=0), window_size)
    # len_ep_std = smooth(np.std(len_ep, axis=0), window_size)
    # reward_mean = smooth(np.mean(reward, axis=0), window_size)
    # reward_std = smooth(np.std(reward, axis=0), window_size)
    # coverage_mean = smooth(np.mean(coverage, axis=0), window_size)
    # coverage_std = smooth(np.std(coverage, axis=0), window_size)
    # trained_coverage_mean = np.mean(trained_coverage, axis=0)
    # trained_coverage_std = np.std(trained_coverage, axis=0)

    # # len_ep_mean2 = smooth(np.mean(len_ep2, axis=0), window_size)
    # # len_ep_std2 = smooth(np.std(len_ep2, axis=0), window_size)
    # # reward_mean2 = smooth(np.mean(reward2, axis=0), window_size)
    # # reward_std2 = smooth(np.std(reward2, axis=0), window_size)
    # # coverage_mean2 = smooth(np.mean(coverage2, axis=0), window_size)
    # # coverage_std2 = smooth(np.std(coverage2, axis=0), window_size)
    # # trained_coverage_mean2 = np.mean(trained_coverage2, axis=0)
    # # trained_coverage_std2 = np.std(trained_coverage2, axis=0)


    # plt.figure(figsize=(10, 6))
    # # plt.plot(trained_coverage_mean2, label='Metodo 1 DQN')
    # # plt.fill_between(np.arange(len(trained_coverage_mean2)), trained_coverage_mean2 - trained_coverage_std2, trained_coverage_mean2 + trained_coverage_std2, alpha=0.5)
    # plt.plot(trained_coverage_mean)
    # plt.fill_between(np.arange(len(trained_coverage_mean)), trained_coverage_mean - trained_coverage_std, trained_coverage_mean + trained_coverage_std, alpha=0.5)
    # plt.title('Copertura del perimetro agente addestrato (Metodo DDPG)', fontsize=14)
    # plt.xlabel('Steps', fontsize=14)
    # plt.ylabel('Copertura media', fontsize=14)
    # plt.xticks(fontsize=14)
    # plt.yticks(fontsize=14)
    # plt.grid(True)
    # # plt.show()


    # plt.figure(figsize=(10, 6))
    # # plt.plot(len_ep_mean2, label='Metodo 1 DQN')
    # # plt.fill_between(np.arange(len(len_ep_mean2)), len_ep_mean2 - len_ep_std2, len_ep_mean2 + len_ep_std2, alpha=0.5)
    # plt.plot(len_ep_mean)
    # plt.fill_between(np.arange(len(len_ep_mean)), len_ep_mean - len_ep_std, len_ep_mean + len_ep_std, alpha=0.5)
    # plt.title('Lunghezza episodi addestramento (Metodo DDPG)', fontsize=14)
    # plt.xlabel('Episodi di addestramento', fontsize=14)
    # plt.ylabel('Lunghezza media', fontsize=14)
    # plt.xticks(fontsize=14)
    # plt.yticks(fontsize=14)
    # plt.grid(True)
    # plt.show()

    # plt.figure(figsize=(10, 6))
    # # plt.plot(reward_mean2, label='Metodo 1 DQN')
    # # plt.fill_between(np.arange(len(reward_mean2)), reward_mean2 - reward_std2, reward_mean2 + reward_std2, alpha=0.5)
    # plt.plot(reward_mean)
    # plt.fill_between(np.arange(len(reward_mean)), reward_mean - reward_std, reward_mean + reward_std, alpha=0.5)
    # plt.title("Ricompense durante l'addestramento (Metodo DDPG)", fontsize=14)
    # plt.xlabel('Episodi di addestramento', fontsize=14)
    # plt.ylabel('Ricompensa media', fontsize=14)
    # plt.xticks(fontsize=14)
    # plt.yticks(fontsize=14)
    # plt.grid(True)
    # plt.show()

    # plt.figure(figsize=(10, 6))
    # plt.title("Copertura del perimentro durante l'addestramento (Metodo DDPG)", fontsize=14)
    # # plt.plot(coverage_mean2, label='Metodo 1 DQN')
    # # plt.fill_between(np.arange(len(coverage_mean2)), coverage_mean2 - coverage_std2, coverage_mean2 + coverage_std2, alpha=0.5)
    # plt.plot(coverage_mean)
    # plt.fill_between(np.arange(len(coverage_mean)), coverage_mean - coverage_std, coverage_mean + coverage_std, alpha=0.5)
    # plt.xlabel('Episodi di addestramento', fontsize=14)
    # plt.ylabel('Copertura media', fontsize=14)
    # plt.xticks(fontsize=14)
    # plt.yticks(fontsize=14)
    # plt.grid(True)
    # plt.show()




