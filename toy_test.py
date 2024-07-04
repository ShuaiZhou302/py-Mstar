from Library import *
from Mstar import *

"""
A toy example
and the optimal cost should be 53
"""
map_data = [[0, 1, 1, 1],
 [0, 0, 0, 0],
 [0, 1, 1, 1],
 [0, 1, 1, 1],
 [0, 1, 1, 1],
 [0, 1, 1, 1]]
map_data2 = [[0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0]]
#  input Choose tuple for it is hashable
graph = Graph(6,4, map_data)
Sinit = ((5, 0), (4, 0), (3, 0), (1, 0))
Send = ((2, 0), (3, 0), (4, 0), (5, 0))

map_data2 = [[0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0],
 [0, 0, 0, 0]]
planner = m_star(graph, Sinit, Send)
print(planner.solve())