#!/usr/bin/env python
import numpy as np
import networkx as nx
from bresenham import bresenham
from MCTS import MCTS

class CentralPathPlanner:
    def __init__(self, num_of_agents, num_of_steps, x_lim, y_lim, res, Grid):
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.res = res
        self.num_of_agents = num_of_agents
        self.num_of_steps = num_of_steps
        self.x_grid = np.round((x_lim[1] - x_lim[0]) / 2)
        self.y_grid = np.round((y_lim[1] - y_lim[0]) / 2)
        self.percent_of_nodes = 0.2
        self.number_of_nodes = 20
        self.sensor_range = 200 #cm
        self.sensor_range_ij = np.round(self.sensor_range/self.res)
        self.Grid = np.array(Grid)
        self.i_min, self.j_min = self.xy_to_ij(self.x_lim[0], y_lim[0])
        self.i_max, self.j_max = self.xy_to_ij(self.x_lim[1], y_lim[1])
        self.max_tiles_explored = np.round((self.x_lim[1]*self.y_lim[1])/self.res)
        self.mcts = MCTS(30, self.num_of_agents, num_of_steps, self.max_tiles_explored, self.number_of_nodes)


    def FindGlobalTrajectories(self, dict_of_drones_pos, Grid):
        self.Grid = np.array(Grid)
        graph, nodesidxs_ij = self.MakeGraph(dict_of_drones_pos)
        nodes_matrix = self.mcts.act_mcts(graph)
        nodes_matrix = np.reshape(nodes_matrix, (self.num_of_agents, len(nodes_matrix))).T

        for a_idx in range(self.num_of_agents):
            temp_trajectory = list()
            for s_idx in range(len(nodes_matrix)):
                temp_trajectory.append(graph.node[nodes_matrix[s_idx][a_idx]]['xy_idx'])
            dict_of_drones_pos[a_idx].trajectory = temp_trajectory
        return dict_of_drones_pos, nodesidxs_ij


    def MakeGraph(self, dict_of_drones_pos):
        currdronespos = list()
        currdronespos.append([list(self.xy_to_ij(dict_of_drones_pos[x].pos[0][0], dict_of_drones_pos[x].pos[0][1])) for x in dict_of_drones_pos])
        G = nx.Graph()
        rowidx = np.where(self.Grid == 0)[0]
        colidx = np.where(self.Grid == 0)[1]
        combinedidx = zip(rowidx, colidx)
        # randidx = np.random.choice(range(len(combinedidx)), int(np.round(len(colidx)*self.percent_of_nodes)))
        randidx = np.random.choice(range(len(combinedidx)), self.number_of_nodes)
        nodesidxs_ij = [list(combinedidx[x]) for x in randidx]
        # The first n nodes will be the current positions of the drones.
        # The visibility graph will be built considering these nodes.
        nodesidxs_ij = currdronespos[0] + nodesidxs_ij

        for k in range(len(nodesidxs_ij)):
            G.add_node(k)
            x_idx, y_idx = self.ij_to_xy(nodesidxs_ij[k][0], nodesidxs_ij[k][1])
            G.node[k]['xy_idx'] = [x_idx, y_idx]
            list_exp_nodes = self.EstTOF(nodesidxs_ij[k])
            G.node[k]['estexpnodes'] = list_exp_nodes
            G.node[k]['ij_idx'] = nodesidxs_ij[k]

        for ki in range(G.number_of_nodes()):
            for kj in range(G.number_of_nodes()):
                node_i_idx = G.node[ki]['ij_idx']
                node_j_idx = G.node[kj]['ij_idx']
                if np.array_equal(node_i_idx, node_j_idx):
                    continue
                linepath = list(bresenham(node_i_idx[0], node_i_idx[1], node_j_idx[0], node_j_idx[1]))
                freepathflag = True
                for ii, elem in enumerate(linepath[1:-1]):
                    if self.Grid[elem[0]][elem[1]] != 0:
                        freepathflag =False
                        break
                if freepathflag:
                    G.add_edge(ki, kj)
        return G, nodesidxs_ij

    def EstTOF(self, pos):
        output_list = list()
        directions_vec = [0, np.pi/8, np.pi/4, 3*np.pi/8, np.pi/2, 5*np.pi/8, 3*np.pi/4, 7*np.pi/8, np.pi,
                                 9*np.pi/8, 5*np.pi/4, 11*np.pi/8, 3*np.pi/2, 13*np.pi/8, 7*np.pi/4, 15*np.pi/8]
        for phi in directions_vec:
            output_list = self.EstRevealedArea(pos, phi)
        return output_list

    def EstRevealedArea(self, node_pos, node_dir):
        tof_target = np.add(node_pos, np.round(np.multiply(self.sensor_range_ij, [np.cos(node_dir), np.sin(node_dir)])))
        tof_target[0] = np.minimum(tof_target[0], self.i_max-1)
        tof_target[0] = np.maximum(tof_target[0], self.i_min+1)
        tof_target[1] = np.minimum(tof_target[1], self.j_max-1)
        tof_target[1] = np.maximum(tof_target[1], self.j_min+1)
        est_revealed_arr = list()
        linepath = list(bresenham(node_pos[0], node_pos[1], int(tof_target[0]), int(tof_target[1])))
        for ii, elem in enumerate(linepath[1:]):
            if self.Grid[elem[0]][elem[1]] == -1:
                est_revealed_arr.append([elem[0], elem[1]])
            elif self.Grid[elem[0]][elem[1]] == 1:
                break
        return est_revealed_arr

    def GoalAssignment(self):
        pass

    def EvaluatePath(self):
        pass

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

