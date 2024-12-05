import numpy as np

def get_kinematics(sim_data):
    # Let sim_data be a N x 4 array, N data points
    # Each row = [x, y, z, t]

    # Differences between data points, N-1 x 4
    sim_data_diff = np.diff(sim_data, axis=0)

    # Velocity = ds / dt
    velocities = sim_data_diff[:, :3] / sim_data_diff[:, 3]

    # Accel = dv / dt
    vel_diff = np.diff(velocities, axis=0)
    accels = vel_diff / sim_data_diff[:-1, 3]

    # Get magnitudes of s, v, a
    sx = sim_data_diff[:, 0]
    sy = sim_data_diff[:, 1]
    sz = sim_data_diff[:, 2]
    dist_mags = np.sqrt(np.power(sx,2) + np.power(sy,2) + np.power(sz,2))

    vx = velocities[:, 0]
    vy = velocities[:, 1]
    vz = velocities[:, 2]
    vel_mags = np.sqrt(np.power(vx,2) + np.power(vy,2) + np.power(vz,2))

    ax = accels[:, 0]
    ay = accels[:, 1]
    az = accels[:, 2]
    accel_mags = np.sqrt(np.power(ax,2) + np.power(ay,2) + np.power(az,2))

    # Get total time, distance, and max acceleration
    t_total = sim_data[-1, 3] - sim_data[0, 3]
    s_total = np.sum(dist_mags)
    a_max = np.max(accel_mags)

    return(s_total, t_total, a_max)

