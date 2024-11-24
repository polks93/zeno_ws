import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    data_path = "/home/paolo/NBV_data/sim_data16.npz"
    data = np.load(data_path)

    """ Limiti del workspace """
    workspace = data['workspace']
    xmin, ymin, xmax, ymax = workspace
    ship_scale_factor = data['ship_scale_factor']
    ship_center = data['ship_center']

    """ Dati odometria """
    odom_timestamps = data['odom_timestamps']
    x = data['x']
    y = data['y']
    yaw = data['yaw']
    vx = data['vx']
    omega = data['omega']

    """ Dati coverage """
    coverage_timestamps = data['coverage_timestamps']
    coverage = data['coverage']

    """ Dati campioni """
    best_samples_timestamps = data['best_samples_timestamps']
    best_samples_x = data['best_samples_x']
    best_samples_y = data['best_samples_y']
    best_samples_theta = data['best_samples_theta']

    relative_error_timestamps = data['relative_error_timestamps']
    v_surge_des = data['v_surge_des']
    yaw_rel = data['yaw_rel']

    abort_timestamps = data['abort_timestamps']
    abort_msg = data['abort_values']
    out_of_bounds_counter = 0
    collision_counter = 0
    for msg in abort_msg:
        if msg == b'Out of bounds':
            out_of_bounds_counter += 1
            abort_msg = np.delete(abort_msg, np.where(abort_msg == msg))
        elif msg =="Collision":
            collision_counter += 1

    print(len(abort_msg))
    # print(collision_counter)
    # print(out_of_bounds_counter)
    # ship_scale_factor = np.array(0.9)
    # ship_center = np.array([xmin + xmax / 2, ymin + ymax / 2])

    # np.savez("/home/paolo/NBV_data/sim_data2", 
    #         coverage_timestamps=coverage_timestamps, 
    #         coverage=coverage,
    #         odom_timestamps=odom_timestamps, 
    #         x=x,   
    #         y=y, 
    #         yaw=yaw, 
    #         vx=vx, 
    #         omega=omega,
    #         best_samples_timestamps=best_samples_timestamps, 
    #         best_samples_x=best_samples_x,
    #         best_samples_y=best_samples_y, 
    #         best_samples_theta=best_samples_theta,
    #         relative_error_timestamps=relative_error_timestamps, 
    #         v_surge_des=v_surge_des, 
    #         yaw_rel=yaw_rel,
    #         workspace=workspace,
    #         ship_scale_factor=ship_scale_factor,
    #         ship_center=ship_center)
    
    """ Plot """
    plt.figure()
    plt.plot(x, y, color='blue', label='Trajectory')
    plt.quiver(
        best_samples_x, 
        best_samples_y, 
        np.cos(best_samples_theta), 
        np.sin(best_samples_theta), 
        color='red',  # Imposta il colore
        scale=1,       # Imposta la scala (lunghezza delle frecce)
        scale_units='xy',  # Usa unità coerenti per x e y
        angles='xy',  # Gli angoli sono riferiti al piano xy
        width=0.005,   # Imposta la larghezza delle frecce
        label='NBV'
    )
    plt.title("Trajectory")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.grid(True)
    plt.show()


    # plt.figure()
    # plt.plot(odom_timestamps, yaw, color='blue')
    # plt.plot(relative_error_timestamps, yaw_rel, color='red')
    # plt.title("Yaw")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Yaw [rad]")
    # plt.show()

    # plt.figure()
    # plt.plot(odom_timestamps, vx, color='blue')
    # plt.plot(relative_error_timestamps, v_surge_des, color='red')
    # plt.title("Surge velocity")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Surge velocity [m/s]")
    # plt.show()

    # plt.figure()
    # plt.plot(odom_timestamps, omega)
    # plt.title("Yaw velocity")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Yaw velocity [rad/s]")
    # plt.show()

    plt.figure()
    plt.plot(coverage_timestamps, coverage)
    plt.title("Coverage")
    plt.xlabel("Time [s]")
    plt.ylabel("Coverage [%]")
    plt.show()