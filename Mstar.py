"""
A naive implementation of M* algorithm
"""
import copy
import heapq
from collections import deque


class joint_Node:
    def __init__(self, config, parent, g):
        """
        The joint configuration
        config: a  joint tuple of current vertex for each agent
        g: node_cost
        back_set: back propagation set
        C_k: collision set
        """
        self.config = config
        self.parent = parent
        self.g = g

    def __hash__(self):
        # 将states转换为元组，然后计算其哈希值
        state_tuple = self.config
        return hash(state_tuple)


class m_star:
    def __init__(self, graph, start_points, end_points):
        """
        set up the initial setting
        Input:
               Graph G : as implement in Lib.py
               Start points: a tuple of joint coordination
               end points: a tuple of joint coordination
        """
        self.Explored = None
        self.graph = graph
        self.v_init = start_points  # initial configuration
        self.v_f = end_points  # goals configuration
        self.S_init = joint_Node(start_points, None, 0, set(), set())
        self.dist_table, self.policy_table = self.generate_distable()  # set up h value table and policy table
        self.collision_set = dict()
        self.backprop = dict()

    def generate_distable(self):
        """
        Bfs based
        set up the h value table and policy table for each agent to refer
        Useful for joint node h value and policy
        """
        # use BFS as the heuristic function value also the optimal policy for M*
        dis_Table = []
        policy_Table = []
        for i in range(len(self.v_init)):
            tmp = deque()
            tmp.append(self.v_f[i])
            inf = float('inf')
            dist_table = [[inf for _ in range(self.graph.width)] for _ in
                          range(self.graph.height)]  # initialize distance
            policy_table = [[None for _ in range(self.graph.width)] for _ in range(self.graph.height)]
            dist_table[self.v_f[i][0]][self.v_f[i][1]] = 0  # set initial point distance to 0
            policy_table[self.v_f[i][0]][self.v_f[i][1]] = self.v_f[i]  # set end point policy as itself
            while len(tmp) > 0:
                curr = tmp.pop()
                for neigh in self.graph.neighbors[curr]:
                    if dist_table[neigh[0]][neigh[1]] > dist_table[curr[0]][curr[1]] + 1:  # update neighbour's distance
                        dist_table[neigh[0]][neigh[1]] = dist_table[curr[0]][curr[1]] + 1
                        policy_table[neigh[0]][neigh[1]] = curr  # map the policy of neigh to the optimal path for M*
                        tmp.append(neigh)

            dis_Table.append(dist_table)
            policy_Table.append(policy_table)

        return dis_Table, policy_Table

    def solve(self):
        """
        Main solve function
        """
        self.Open = []
        self.closed = set()
        self.Explored = dict()
        heapq.heappush(self.Open, (self.S_init.g + self.get_h(self.S_init), self.S_init))
        self.Explored[self.S_init.config] = self.S_init

        while len(self.Open) > 0:
            f, state = heapq.heappop(self.Open)
            if state.config == self.v_f:
                return self.back_track(state)
            if state in self.closed:
                continue
            else:
                self.closed.add(state)

            # get neighbours of a
            Sngh = self.get_neighbours(state)
            for s in Sngh:
                self.Add_toBackprop(state, s)
                C_i = self.collision_detect(s)
                self.UnionColset(s,C_i)
                self.backprop(state, C_i)

                if self.compare(s, C_i):
                    heapq.heappush(self.Open, (s.g + self.get_h(s), s))

        print("So sorry bruh, it just fail")
        return None

    def get_h(self, state):
        """
        Get heuristic value of a State
        """
        config = state.config
        cost = 0
        for index, coord in enumerate(config):
            cost += self.dist_table[index][coord[0]][coord[1]]
        return cost

    def back_track(self, state):
        """
        Return a list of configuration from start points to goal points
        """
        path = []
        s_curr = state
        while s_curr.parent is not None:
            path.append(s_curr.config)
            s_curr = s_curr.parent
        reversed_list = list(reversed(path))
        return reversed_list

    def get_neighbours(self, state):
        """
        A method that generate a list of following vertex to visit
        may return a list of joint nodes to visit
        """
        total_ngh = 1
        new = dict()
        for i in self.collision_set.get(state):
            v = state.config[i]
            ngh = [v] + self.graph.neighbors[v]
            new[i] = ngh
            total_ngh = total_ngh * len(ngh)

        Ssample = [None] * len(state.config)
        for index, v in enumerate(state.config):
            if index not in self.collision_set.get(state):
                policy_next = self.get_policy(index, v)
                Ssample[index] = (policy_next)
        Snghstate = [copy.deepcopy(Ssample) for _ in range(total_ngh)]
        new_tuple = [(key, value) for key, value in new.items()]
        heapq.heapify(new_tuple)  # 这样在递归的时候可以去掉
        Snghstates = self.insert_neigh(new_tuple, Snghstate, state)
        Sngh = []
        for states in Snghstates:
            Sngh.append(joint_Node(states, state, state.g + self.get_edge(state.config, states)))
        return Sngh

    def get_policy(self, agent, vertex):
        """
        Return the policy of this vertex
        """
        return self.policy_table[agent][vertex[0]][vertex[1]]

    def backPropagation(self, s, C_k):
        """
        Doing backpropagation
        """
        if self.IsSubset(C_k, self.collision_set.get(s)):
            return
        self.UnionColset(s,C_k)
        self.reopen(s)

        if s.config == self.v_init:
            return
        for parent_s in self.backprop.get(s):
            self.backPropagation(parent_s, C_k)

    def IsSubset(self, C_k, curr_C):
        """
        As its name
        """
        return C_k.issubset(curr_C)

    def reopen(self, s):
        """
        Reinsert one node to open set
        """
        heapq.heappush(self.Open, (s.g + self.get_h(s), s))
        if s in self.closed:
            self.closed.remove(s)

    def Add_toBackprop(self, curr_s, ngh_s):
        """
        A method that add state to backpropagation
        """
        if self.backprop.get(ngh_s) is None:
            self.backprop[ngh_s] = set()
        self.backprop[ngh_s].add(curr_s)
        return

    def get_edge(self, config_from, config_to):
        """
        Return edge cost
        """
        cost = 0
        for i in range(len(self.v_f)):
            if not (self.v_f[i] == config_from[i] == config_to[i]):
                cost += 1
        return cost

    def insert_neigh(self, new_tuple, Snghstate, sk):
        """
        Insert collision neighbours to the following places
        """
        if len(new_tuple) == 0:
            return Snghstate

        re_nghs = []
        agent, nghs = heapq.heappop(new_tuple)
        division = self.divide_list(Snghstate, len(nghs))  # 把sngh拆分成nghs数量等同的部分

        for ngh, sub_Snghs in enumerate(division): #表示特定ngh的下标和分配到要填充的分部分
            v = nghs[ngh]
            for j in range(len(sub_Snghs)):  # j 表示要插的位置
                sub_Snghs[j][agent] = v

            new_tuple_copy = copy.deepcopy(new_tuple) #跟list一样防止反复调用
            re_nghs += tuple(self.insert_neigh(new_tuple_copy, sub_Snghs, sk))

        return re_nghs

    def divide_list(self, my_list, num_parts):
        """
        Divide the list to certain part to insert the neighbours
        """
        avg = len(my_list) // num_parts
        remainder = len(my_list) % num_parts
        result = [copy.deepcopy(my_list[i * avg + min(i, remainder):(i + 1) * avg + min(i + 1, remainder)]) for i in
                  range(num_parts)]
        return result

    def collision_detect(self, state):
        """
        Return a collision set
        """
        Occupied = dict()
        C_i = set()
        for index, v in enumerate(state.config):
            if Occupied.get(v) is None:
                Occupied[v] = set()
                Occupied[v].add(index)
            else:
                C_i.add(index)
                for i in Occupied.get(v):
                    C_i.add(i)
        return C_i

    def UnionColset(self, s, C_i):
        """
        Union the collision set
        """
        c = self.collision_set.get(s)
        if c is None:
            self.collision_set[s] = C_i
        else:
            union = C_i.union(c)
            self.collision_set[s] = union

    def compare(self, s, C_i):
        """
        Compare if a new node is better
        """
        if len(C_i) != 0:
            return False
        s_before = self.Explored.get(s.config)
        if s_before is None:
            self.Explored[s.config] = s
            return True
        else:
            if s.g < s_before.g:
                self.Explored[s.config] = s
                return True
            return False





