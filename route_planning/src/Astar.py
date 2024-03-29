#!/usr/bin/env python
import math
import numpy as np
from bresenham import bresenham
import matplotlib.pyplot as plt
from copy import deepcopy as dcpy
from k_means_los import k_means_los

class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind


class Astar:

    def __init__(self, x_lim, y_lim, matrix, res, tf_prefix):
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.matrix = matrix
        self.res = res
        self.tf_prefix = tf_prefix
        self.scanning_range = np.inf
        self.num_of_temp_nodes = 60
        self.use_dict_drone = False

    def PlanningAlg(self, sx, sy, gx, gy):

        # sx: start x position [m]
        # sy: start y position [m]
        # gx: goal x position [m]
        # gy: goal y position [m]

        nstart = Node(sx, sy, 0, -1)
        ngoal = Node(gx, gy, 0, -1)

        # Derive obstacle map from the Grid
        obmap, minx, miny, maxx, maxy, xw, yw = self.calc_obstacle_map()

        # If the path free exit
        g_i, g_j = self.xy_to_ij(gx, gy)
        s_i, s_j = self.xy_to_ij(sx, sy)
        if self.is_path_free(s_i, s_j, g_i, g_j, obmap):
            Astar_path = []
            return Astar_path

        # Choose motion nodes
        mx, my = self.get_motion_nodes(sx, sy, gx, gy)

        image = dcpy(obmap)
        for i in range(len(mx)):

            t = self.xy_to_ij(mx[i], my[i])
            image[t[0]][t[1]] = 50

        for i in range(-3,3):
            for j in range(-3, 3):
                image[g_i + i, g_j + j] = 50
                image[s_i + i, s_j + j] = 50

        plt.imsave('mapWithoutlocTarget.png', obmap)
        plt.imsave('map.png', image)

        openset, closedset = dict(), dict()
        openset[self.calc_index(nstart, xw, minx, miny)] = nstart

        while 1:

            try:
                c_id = min(openset, key=lambda o: openset[o].cost + self.calc_heuristic(ngoal, openset[o]))
            except:
                print('array empty')
                Astar_path = []
                return Astar_path

            current = openset[c_id]

            if abs(current.x - gx) <= self.res and abs(current.y - gy) <= self.res:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del openset[c_id]

            # Add it to the closed set
            closedset[c_id] = current

            motion = self.get_motion_model(mx, my, current, ngoal, obmap)

            # expand search grid based on motion model
            for i in range(len(motion)):
                node = Node(int(motion[i][0]),
                            int(motion[i][1]),
                            current.cost + motion[i][2], c_id)

                n_id = self.calc_index(node, xw, minx, miny)

                if n_id in closedset:
                    continue

                if not self.verify_node(node, obmap, minx, miny, maxx, maxy):
                    continue

                if n_id not in openset:
                    openset[n_id] = node  # Discover a new node
                else:
                    if openset[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        openset[n_id] = node


        Astar_path = self.calc_fianl_path(ngoal, closedset)

        return Astar_path

    def calc_heuristic(self, n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)

        return d


    def verify_node(self, node, obmap, minx, miny, maxx, maxy):

        if node.x < minx:
            return False
        elif node.y < miny:
            return False
        elif node.x >= maxx:
            return False
        elif node.y >= maxy:
            return False

        i, j = self.xy_to_ij(node.x, node.y)
        if obmap[i][j]:
            return False

        return True

    def calc_obstacle_map(self):

        minx = self.x_lim[0]
        miny = self.y_lim[0]
        maxx = self.x_lim[1]
        maxy = self.y_lim[1]

        xwidth = round(maxx - minx)
        ywidth = round(maxy - miny)

        obmap = self.matrix != 0

        return obmap, minx, miny, maxx, maxy, xwidth, ywidth

    def calc_index(self, node, xwidth, xmin, ymin):
        return (node.y - ymin) * xwidth + (node.x - xmin)

    def get_motion_nodes(self, sx, sy, gx, gy):

        mx, my = [], []

        # resfactor = 2
        # current_drone_pos = [sx, sy]
        # current_drone_goal = [gx, gy]
        # rng_start_to_goal = 2*np.linalg.norm(np.subtract(current_drone_pos, current_drone_goal))
        # x_min = np.maximum(current_drone_pos[0] - rng_start_to_goal, self.x_lim[0]+(resfactor*self.res))
        # x_max = np.minimum(current_drone_pos[0] + rng_start_to_goal, self.x_lim[1]-(resfactor*self.res))
        # y_min = np.maximum(current_drone_pos[1] - rng_start_to_goal, self.y_lim[0]+(resfactor*self.res))
        # y_max = np.minimum(current_drone_pos[1] + rng_start_to_goal, self.y_lim[1]-(resfactor*self.res))
        #
        # x_rand_vec = np.random.choice(range(int(np.round(x_min)), int(np.round(x_max))), int(self.num_of_temp_nodes))
        # y_rand_vec = np.random.choice(range(int(np.round(y_min)), int(np.round(y_max))), int(self.num_of_temp_nodes))
        #
        # for k in range(len(x_rand_vec)):
        #     temp_m_node_xy = [x_rand_vec[k], y_rand_vec[k]]
        #     temp_i, temp_j = self.xy_to_ij(x_rand_vec[k], y_rand_vec[k])
        #     # if self.matrix[temp_i][temp_j] == 0:
        #     if (self.matrix[temp_i][temp_j] == 0 and
        #             self.matrix[temp_i - 1][temp_j - 1] == 0 and
        #             self.matrix[temp_i][temp_j - 1] == 0 and
        #             self.matrix[temp_i + 1][temp_j - 1] == 0 and
        #             self.matrix[temp_i + 1][temp_j] == 0 and
        #             self.matrix[temp_i + 1][temp_j + 1] == 0 and
        #             self.matrix[temp_i][temp_j + 1] == 0 and
        #             self.matrix[temp_i - 1][temp_j + 1] == 0 and
        #             self.matrix[temp_i - 1][temp_j] == 0):
        #         mx.append(temp_m_node_xy[0])
        #         my.append(temp_m_node_xy[1])
        #     if len(mx) >= (self.num_of_temp_nodes / 2):
        #         break
        # mx.append(gx)
        # my.append(gy)

        nodes = k_means_los(self.num_of_temp_nodes, self.matrix, self.res, [sx, sy], [gx, gy],
                            self.x_lim, self.y_lim, False)
        for idx, elem in enumerate(nodes):
            tempx, tempy = self.ij_to_xy(elem[0], elem[1])
            mx.append(tempx)
            my.append(tempy)

        return mx, my

    def get_motion_model(self, mx, my, nstart, ngoal, obmap):

        okways = []
        allx, ally, allcost, allmidxs = [], [], [], []
        pradius = self.scanning_range
        # pradius = math.sqrt((nstart.x - ngoal.x) ** 2 + (nstart.y - ngoal.y) ** 2)
        for j, jend in enumerate(zip(mx, my)):
            dcost = math.sqrt((nstart.x - jend[0]) ** 2 + (nstart.y - jend[1]) ** 2)
            if dcost <= pradius:  # Not mandatory if... Only to speed up!
                allx.append(jend[0])
                ally.append(jend[1])
                allcost.append(dcost)
        allx.append(ngoal.x)
        ally.append(ngoal.y)
        allcost.append(math.sqrt((nstart.x - ngoal.x) ** 2 + (nstart.y - ngoal.y) ** 2))
        for i, iend in enumerate(zip(allx, ally, allcost)):
            s_i, s_j = self.xy_to_ij(nstart.x, nstart.y)
            g_i, g_j = self.xy_to_ij(iend[0], iend[1])
            if self.is_path_free(s_i, s_j, g_i, g_j, obmap):
                okways.append([iend[0], iend[1], iend[2]])

        return okways

    def is_path_free(self, si, sj, gi, gj, obmap):
        bpath = list(bresenham(si, sj, gi, gj))
        ok_way = True
        for ii, elem in enumerate(bpath[1:]):
            # if obmap[elem[0]][elem[1]] != 0:
            if (obmap[elem[0]][elem[1]] != 0 or
                    obmap[elem[0] - 1][elem[1] - 1] != 0 or
                    obmap[elem[0]][elem[1] - 1] != 0 or
                    obmap[elem[0] + 1][elem[1] - 1] != 0 or
                    obmap[elem[0] + 1][elem[1]] != 0 or
                    obmap[elem[0] + 1][elem[1] + 1] != 0 or
                    obmap[elem[0]][elem[1] + 1] != 0 or
                    obmap[elem[0] - 1][elem[1] + 1] != 0 or
                    obmap[elem[0] - 1][elem[1]] != 0):
                ok_way = False
                break

        return ok_way

    def calc_fianl_path(self, ngoal, closedset):
        # generate final path
        final_path = [[ngoal.x, ngoal.y]]
        # rx, ry = [ngoal.x], [ngoal.y]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            # rx.append(n.x)
            # ry.append(n.y)
            final_path.append([n.x, n.y])
            pind = n.pind

        return list(reversed(final_path))

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y


def build_trj(pos, env_limits, res, matrix, goal, tf_prefix):
    x_lim = env_limits[0:2]
    y_lim = env_limits[2:4]
    astar = Astar(x_lim, y_lim, matrix, res, tf_prefix)

    gx = goal[0][0]
    gy = goal[0][1]
    print('res')
    print(res)
    print('start')
    print([pos[0][0], pos[0][1]])
    print('goal')
    print([gx, gy])
    astar_movement = astar.PlanningAlg(pos[0][0], pos[0][1], gx, gy)
    print('astar_movement')
    print(astar_movement)
    Astar_Movement = astar_movement[1:-1]

    return Astar_Movement
