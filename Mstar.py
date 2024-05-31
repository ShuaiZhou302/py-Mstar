"""
A naive implementation of M* algorithm
"""
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

            # get neighbours of a
            Snbh = self.get_neighbours(state)
            for s in Snbh:
                self.Add_toBackprop(state, s)

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
        new = []

    def get_policy(self, agent, vertex):
        """
        Return the policy of this vertex
        """
        return self.policy_table[agent][vertex[0]][vertex[1]]

    def backPropagation(self, sl, C_k):
        pass

    def Add_toBackprop(self, curr_s, ngh_s):
        """
        A method that add state to backpropagation
        """
        if self.backprop.get(ngh_s) is None:
            self.backprop[ngh_s] = set()
        self.backprop[ngh_s].add(curr_s)
        return

