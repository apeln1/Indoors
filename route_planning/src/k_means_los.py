import numpy as np
import copy


def xy_to_ij(x, y, x_lim, y_lim, res):
    i = int(np.floor((x - x_lim[0]) / res))
    j = int(np.floor((y - y_lim[0]) / res))
    return i, j

def k_means_los(n_of_centers, matrix, res, drone_pos, drone_goal, x_lim, y_lim, kmeans_flag):

    res_factor = 2
    if drone_goal and drone_pos:
        rng_start_to_goal = 1.5 * np.linalg.norm(np.subtract(drone_pos, drone_goal))
        x_min = np.maximum(drone_pos[0] - rng_start_to_goal, x_lim[0] + (res_factor * res))
        x_max = np.minimum(drone_pos[0] + rng_start_to_goal, x_lim[1] - (res_factor * res))
        y_min = np.maximum(drone_pos[1] - rng_start_to_goal, y_lim[0] + (res_factor * res))
        y_max = np.minimum(drone_pos[1] + rng_start_to_goal, y_lim[1] - (res_factor * res))
    else:
        x_min = x_lim[0] + (res_factor * res)
        x_max = x_lim[1] - (res_factor * res)
        y_min = y_lim[0] + (res_factor * res)
        y_max = y_lim[1] - (res_factor * res)

    i_min, j_min = xy_to_ij(x_min, y_min, x_lim, y_lim, res)
    i_max, j_max = xy_to_ij(x_max, y_max, x_lim, y_lim, res)
    allowed_samp = []

    for i in range(i_min, i_max):
        for j in range(j_min, j_max):
            if (matrix[i-1][j-1] == 0 and
                    matrix[i][j-1] == 0 and
                    matrix[i+1][j-1] == 0 and
                    matrix[i+1][j] == 0 and
                    matrix[i+1][j+1] == 0 and
                    matrix[i][j+1] == 0 and
                    matrix[i-1][j+1] == 0 and
                    matrix[i-1][j] == 0):
                allowed_samp.append([i, j])

    idxs = list(range(len(allowed_samp)))
    np.random.shuffle(idxs)
    nodes_idxs = idxs[:n_of_centers]
    nodes = [allowed_samp[n] for n in nodes_idxs]
    nodes = np.array(nodes)

    if kmeans_flag:
        data = np.array(allowed_samp)
        data = np.reshape(data, (len(data), 2))

        # Number of clusters
        k = n_of_centers
        # Number of training data
        n = data.shape[0]
        # Number of features in the data
        c = data.shape[1]

        centers_old = np.zeros(nodes.shape)  # to store old centers
        centers_new = copy.deepcopy(nodes)  # Store new centers

        clusters = np.zeros(n)
        distances = np.zeros((n, k))

        error = np.linalg.norm(np.subtract(centers_new, centers_old))

        # When, after an update, the estimate of that center stays the same, exit loop
        while error > res:
            # Measure the distance to every center
            for i in range(k):
                distances[:, i] = np.linalg.norm(data - nodes[i], axis=1)
            # Assign all training data to closest center
            clusters = np.argmin(distances, axis=1)

            centers_old = copy.deepcopy(centers_new)
            # Calculate mean for every cluster and update the center
            for i in range(k):
                centers_new[i] = np.mean(data[clusters == i], axis=0)
            error = np.linalg.norm(centers_new - centers_old)

        nodes = np.array(copy.deepcopy(centers_new))

    return nodes



