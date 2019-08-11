import numpy as np
import copy


class node:
    def __init__(self, state, parent, Q_func, was_expand, fc, nstate, revealed_tiles):
        # state is a matrix of nodes numbers. rows - drone index. columns - mcts step
        self.state = state
        self.parent = parent
        self.Q_func = Q_func
        self.was_expand = was_expand
        self.fc = fc
        self.nstate = nstate
        self.revealed_tiles = revealed_tiles


class MCTS:
    def __init__(self, niter, ndrones, mctssteps, max_tiles, nnodes):
        # number of allowed iterations
        self.niter = niter
        # The index of each node in the dictionary
        self.ncount = 0
        # Total number of drones
        self.ndrones = ndrones
        # The number of MCTS steps
        self.mctssteps = mctssteps
        #  The maximum number of tiles that can be exposed in the structure
        self.max_tiles = max_tiles
        # The number of nodes in graph
        self.nnodes = nnodes


    def act_mcts(self, Graph):
        self.G = Graph
        # all possible connections to each node in the graph (visibility graph)
        self.connections_dict = self.get_connection_list()
        nodes_set = dict()
        # The first ndrones nodes are the current position of the drones. These are the initial state of the MCTS.
        state = range(self.ndrones)
        p_id = -1
        nodes_set[self.ncount] = node(state=state, parent=p_id, Q_func=0, was_expand=0,
                                      fc=0, nstate=1, revealed_tiles=[])
        nodes_set[self.ncount].revealed_tiles = self.get_revealed_tiles(nodes_set, self.ncount)
        self.ncount += 1

        for iter in range(self.niter):
            current, c_id = self.choose_node(nodes_set, p_id)
            nodes_set = self.expand_node(nodes_set, c_id)
            p_id = c_id

        s_id = max(nodes_set, key=lambda o: nodes_set[o].fc)
        solution = self.get_final_solution(nodes_set, s_id)
        return solution


    def get_final_solution(self, nodes_set, s_id):
        pind = s_id
        states = list()
        # '0' will be the parent id of the first extension and will lead to the initial state which should be ignored
        while pind != -1:
            states.append(nodes_set[pind].state)
            pind = nodes_set[pind].parent
        return list(reversed(states))[1:]


    def get_revealed_tiles(self, nodes_set, c_id):
        revealed_tiles = list()
        if nodes_set[c_id].parent != -1:
            revealed_tiles = nodes_set[nodes_set[c_id].parent].revealed_tiles
        for i, elem in enumerate(nodes_set[c_id].state):
            temp_arr = [n for n in self.G.node[elem]['estexpnodes'] if n not in revealed_tiles]
            revealed_tiles = revealed_tiles + temp_arr
        return revealed_tiles


    def calc_Q_func(self, nodes_set, c_id):
        # Give more weight to the future when the agent at the beginning of path planning
        # and les when it approaches to the end
        alpha = np.multiply(np.divide(1, float(self.mctssteps)), float(nodes_set[c_id].nstate))
        gamma = 0.9
        try:
            parent_Q_func = nodes_set[nodes_set[c_id].parent].Q_func
        except:
            parent_Q_func = 0
        # Compute new Q function
        current_state = copy.deepcopy(nodes_set[c_id].state)
        current_nstate = copy.deepcopy(nodes_set[c_id].nstate)
        future_reward = self.est_future_reward(current_state, current_nstate)
        previous_state = copy.deepcopy(nodes_set[nodes_set[c_id].parent].state)
        reward_for_step = self.calc_reward_for_step(current_state, previous_state)
        Q_func = alpha*parent_Q_func + (1-alpha)*(reward_for_step + gamma*future_reward)
        return Q_func


    def est_future_reward(self, init_state, init_nstate):
        # This function is equivalent to rollout in MCTS
        future_reward = 0
        n_remaining_steps = self.mctssteps - init_nstate
        for i in range(n_remaining_steps):
            allowed_nodes = [self.connections_dict[x] for x in init_state]
            all_exp_options = [list(x) for x in np.array(np.meshgrid(*allowed_nodes)).T.reshape(-1, len(allowed_nodes))
                               if len(set(list(x))) == self.ndrones and not set(list(x)) & set(init_state)]
            pred_state_idx = np.random.choice(len(all_exp_options))
            pred_state = all_exp_options[pred_state_idx]
            future_reward += self.calc_reward_for_step(init_state, pred_state)
            init_state = copy.deepcopy(pred_state)
        return future_reward


    def calc_reward_for_step(self, current_state, previous_state):
        reward = 0
        for i, elem in enumerate(current_state):
            new_tiles = []
            exp_curr_tiles = self.G.node[elem]['estexpnodes']
            exp_prev_tiles = self.G.node[previous_state[i]]['estexpnodes']
            new_tiles = [n for n in exp_curr_tiles if n not in exp_prev_tiles]
            if new_tiles:
                reward += 1
            else:
                reward -= 1
        return np.divide(float(reward), float(len(current_state)))


    def choose_node(self, nodes_set, p_id):
        relevant_nodes = dict()
        # The priority is to choose nodes that were expended with current parent id.
        for i, elem in enumerate(nodes_set):
            if nodes_set[elem].nstate < self.mctssteps and nodes_set[elem].was_expand==0 and np.array_equal(nodes_set[elem].parent, p_id):
                relevant_nodes[elem] = nodes_set[elem]
        try:
            c_id = max(relevant_nodes, key=lambda o: relevant_nodes[o].Q_func)
        except:
            for i, elem in enumerate(nodes_set):
                if nodes_set[elem].nstate < self.mctssteps and nodes_set[elem].was_expand==0:
                    relevant_nodes[elem] = nodes_set[elem]
            c_id = max(relevant_nodes, key=lambda o: relevant_nodes[o].Q_func)
        current = relevant_nodes[c_id]
        return  current, c_id


    def expand_node(self, nodes_set, c_id):
        # Current node is expanded
        nodes_set[c_id].was_exp = 1
        num_expand_nodes = 50
        current = copy.deepcopy(nodes_set[c_id])
        # Extracting all the possible connections for each node ib the state
        allowed_nodes = [self.connections_dict[x] for x in current.state]
        for idx, elem in enumerate(allowed_nodes):
            if not elem:
                allowed_nodes[idx] = [np.random.choice(self.nnodes)]

        # (*)  Creating container types
        # The T attribute is the transpose of the array
        # -1 in reshape means that it is an unknown dimension and we want numpy to figure it out.
        # If there are two equal nodes in the expanded state than the state is invalid
        # If there are elements from the current state than the expanded state is invalid
        all_exp_options = [list(x) for x in np.array(np.meshgrid(*allowed_nodes)).T.reshape(-1, len(allowed_nodes))
                           if len(set(list(x))) == self.ndrones and not set(list(x)) & set(current.state)]
        n_chosen_nodes_idx = np.random.choice(len(all_exp_options), [1, num_expand_nodes])
        all_exp_options = [all_exp_options[x] for x in n_chosen_nodes_idx[0]]
        for i, elem in enumerate(all_exp_options):
            # Create new node
            nodes_set[self.ncount] = node(elem, c_id, 0, 0, 0, current.nstate + 1, [])
            nodes_set[self.ncount].revealed_tiles = self.get_revealed_tiles(nodes_set, self.ncount)
            nodes_set[self.ncount].fc = self.fitnes_criteria(nodes_set[self.ncount].revealed_tiles)
            nodes_set[self.ncount].Q_func = self.calc_Q_func(nodes_set, self.ncount)
            self.ncount += 1
        return nodes_set


    def get_connection_list(self):
        connections_dict = dict()
        list_of_nodes = self.G.nodes()
        for i in range(len(list_of_nodes)):
            connections_list = [n for n in self.G.neighbors(list_of_nodes.keys()[i])]
            connections_dict[list_of_nodes.keys()[i]] = connections_list
        return connections_dict


    def fitnes_criteria(self, n_exp_tiles):
        fc = np.divide(float(len(n_exp_tiles)), float(self.max_tiles)*0.8)
        return fc