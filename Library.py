"""
A common library that contains following methods
1. read_map_file: takes in a map file and return height, width, 2-dimensional array of graph
2. read scenario: read scen file and get n numbers of agents with initial point and end points
3. generate a graph for planner to use.
"""
from matplotlib import animation, pyplot as plt
import numpy as np


def read_map_file(file_path):
    """
    A method that reads map file
    """
    with open(file_path, 'r') as f:
        lines = f.readlines()

    # 提取高度和宽度
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])

    # 提取地图数据
    map_data = []
    for line in lines[4:]:
        row = []
        for char in line.strip():
            if char == '.':
                row.append(0)  # available space
            elif char in ('@', 'T'):
                row.append(1)  # obstacles
        map_data.append(row)

    return Graph(height, width, map_data)

def read_start_and_goal(file_path, n):
    # n the numbers of agents you want to test in this map
    start_points = []
    goal_points = []
    with open(file_path, 'r') as f:
        # skip the first line
        next(f)
        for _ in range(n):
            line = f.readline()
            data = line.split()
            start_point = (int(data[4]), int(data[5]))
            goal_point = (int(data[6]), int(data[7]))
            start_points.append(start_point)
            goal_points.append(goal_point)
    return tuple(start_points), tuple(goal_points)


class Graph:
    def __init__(self, height, width, map):
        """
        A class represents the graph of the map
        The map is a 2-d array as output of read_map_file functions
        """
        self.height = height
        self.width = width
        if map is None:
            self.map = [[0 for _ in range(width)] for _ in range(height)]
        else:
            self.map = map
        self.start_points = []
        self.end_points = []
        self.neighbors = {}
        self.set_neighbors()

    def set_neighbors(self):
        for i in range(self.height):
            for j in range(self.width):
                neighbors = []
                if i > 0 and self.map[i - 1][j] != 1:
                    neighbors.append((i - 1, j))
                if i < self.height - 1 and self.map[i + 1][j] != 1:
                    neighbors.append((i + 1, j))
                if j > 0 and self.map[i][j - 1] != 1:
                    neighbors.append((i, j - 1))
                if j < self.width - 1 and self.map[i][j + 1] != 1:
                    neighbors.append((i, j + 1))
                self.neighbors[(i, j)] = neighbors




