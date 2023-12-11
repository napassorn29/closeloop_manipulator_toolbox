from clmanipulator_toolbox import link,closedLoopMani
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
import matplotlib.animation as animation

##### set closed loop manipulator ################################################################

# L1 = link('A','fixed',np.array(['a','b']),np.array([[0,3],[0,0]]))
# L2 = link('B','input',np.array(['a','e','g']),np.array([[0,6,2],[0,0,1]]))
# L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
# L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,4],[0,0]]))

# L1 = link('A','fixed',np.array(['a','b']),np.array([[0,10],[0,0]]))
# L2 = link('B','input',np.array(['a','e','g']),np.array([[0,3,2],[0,0,1]]))
# L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,10],[0,0]]))
# L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,3],[0,0]]))

# Robot = closedLoopMani([L1,L2,L4,L5])

# L1 = link('A','fixed',np.array(['a','b']),np.array([[0,3],[0,0]]))
# L2 = link('B','input',np.array(['a','e','g']),np.array([[0,4,2],[0,0,1]]))
# L3 = link('C','input',np.array(['b','c','f']),np.array([[0,4,2],[0,0,-1]]))
# L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
# L5 = link('E','intermediate',np.array(['c','d']),np.array([[0,4],[0,0]]))

# Robot = closedLoopMani([L1,L2,L3,L4,L5])

L1 = link('A','fixed',np.array(['a','b']),np.array([[0,61],[0,0]]))
L2 = link('B','input',np.array(['a','e','g']),np.array([[0,19.315,2],[0,0,1]]))
L3 = link('C','input',np.array(['b','c','f']),np.array([[0,16.762,2],[0,0,-1]]))
L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,31.616],[0,0]]))
L5 = link('E','intermediate',np.array(['c','d']),np.array([[0,31.342],[0,0]]))

Robot = closedLoopMani([L1,L2,L3,L4,L5])

###################################################################################################
##### forward kinematic of Robot ##################################################################
# q = [2.1267784793154387]
# mode = "positive"
# output = Robot.fk(q,'d',mode)

# q = [np.pi/2+np.pi/6,np.pi/2-np.pi/6]
# mode = "positive"
# output = Robot.fk([np.pi/2+np.pi/6,np.pi/2-np.pi/6],'d','positive')
# print(output)

# plt.figure(figsize=(6, 6))
# plt.grid(True)

# bound = Robot.boundary4()
# print(bound)

# Robot.plot([bound[1]],"positive")

# minmax4 = Robot.minmax4("positive")
# print("minmax",minmax4)
# Robot.animationfk(100, "positive")
# plt.show()

# Robot.animationik(0.01,0.005,1,[0],'d',[5.1,3.0],'positive')
# plt.show()

# output_fk = Robot.fk(q,'d',"positive")

# pid = Robot.P_control_ik4(0.01,0.005,1,[0],'d',[2,2],'positive')
# print(pid)
# # print(output_fk)
# T_desired = [5.1,3.0]
# q_sol = Robot.ik(T_desired,'d','positive',0.1)
# print("q_sol = ",q_sol)

q = [np.pi/2-np.pi/4,np.pi/2+np.pi/4]
q = [0.47118596694901044, -0.0006370324612307346]
mode = "positive"
output = Robot.fk(q,'d','positive')
print(output)

# T_desired = np.array([[48.69],[11.7]])
# # T_desired = [12.0,4.0]
# q_sol = Robot.ik(T_desired,'d',mode='++',method = 'geometrical')
# print("q_sol = ", q_sol)

# output = Robot.fk(q_sol,'d')
# Cspace_reshaped, q1_space, q2_space = Robot.boundary5()
# print(q1_space)

# cost = Robot.cost_intersection5(np.pi/2, np.pi*2)
# print('cost = ', cost)  

nei = Robot.grid_neighbors([5,5], 1)
print(nei)

a_star = Robot.plan_path(np.array([[48.69],[11.7]]), np.array([[32.97],[38.69]]), "d", '++',1)
print("a_star = ", a_star)

# a_star = Robot.a_star([48.69,11.7], [32.97,38.69], "d", "positive")
# print("a_star = ", a_star) 

# heu = Robot.heuristic([48.69,11.7], [32.97,38.69])
# print("heuristic cost : ", heu)

# planner = AStarPathPlanner(C_space, q1_space, q2_space, cost_function)
# start_point = (0, 0)
# goal_point = (2 * np.pi, 2 * np.pi)

# path = planner.plan_path(start_point, goal_point)

# if path:
#     print("Path found:", path)
# else:
#     print("No valid path found.")
# print(output)



# Robot.plot(q_sol,"positive")
# Robot.animationik(0.01,0.005,5,[0,0],[32.97,38.69],'d','positive',0.1)
# plt.show()


# path planning 

# import heapq

# def astar_search(start, goal, neighbors_fn, heuristic_fn):
#     # Function to create a node tuple with state, parent, cost, and heuristic
#     def make_node(state, parent=None, cost=0, heuristic=0):
#         return (state, parent, cost, heuristic)

#     # Priority queue to store nodes with their combined cost and heuristic values
#     open_set = []
#     # Set to keep track of explored states
#     closed_set = set()

#     # Create the start node and push it onto the priority queue
#     start_node = make_node(start, cost=0, heuristic=heuristic_fn(start))
#     heapq.heappush(open_set, (0, start_node))

#     # Main A* search loop
#     while open_set:
#         # Pop the node with the lowest combined cost and heuristic value
#         current_cost, current_node = heapq.heappop(open_set)

#         # Check if the goal is reached
#         if current_node[0] == goal:
#             path = []
#             # Reconstruct the path from goal to start
#             while current_node:
#                 path.append(current_node[0])
#                 current_node = current_node[1]
#             return path[::-1]

#         # Mark the current state as explored
#         closed_set.add(current_node[0])

#         # Explore neighbors of the current state
#         for neighbor in neighbors_fn(current_node[0]):
#             if neighbor in closed_set:
#                 continue

#             # Calculate the cost to reach the neighbor and the heuristic value
#             cost = current_node[2] + 1  # Assuming uniform cost for simplicity
#             heuristic = heuristic_fn(neighbor)
#             new_node = make_node(neighbor, parent=current_node, cost=cost, heuristic=heuristic)

#             # Check if the neighbor is not in the open set
#             if all(neighbor != node[0] for _, node in open_set):
#                 # Push the new node onto the priority queue
#                 heapq.heappush(open_set, (cost + heuristic, new_node))
#             # Check if the new path to the neighbor is better
#             elif cost < next(cost for c, n in open_set if n[0] == neighbor):
#                 # Update the node in the open set with the new path
#                 open_set = [(c, n) if n[0] != neighbor else (cost + heuristic, new_node) for c, n in open_set]

#     # No path found
#     return None


# Example usage:
# def grid_neighbors(state):
#     x, y = state
#     neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
#     return [(nx, ny) for nx, ny in neighbors if 0 <= nx < 10 and 0 <= ny < 10]  # Adjust grid size as needed

# def grid_neighbors(node):
#     neighbors = []
#     x, y = node
#     possible_neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1), (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)]
#     for neighbor in possible_neighbors:
#         if 0 <= neighbor[0] < np.pi*2 and 0 <= neighbor[1] < np.pi*2:
#             neighbors.append(neighbor)

#     return neighbors

# def grid_heuristic(state, goal):
#     return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

# start_position = (0, 0)
# goal_position = (1, 1)
# path = astar_search(start_position, goal_position, grid_neighbors, lambda state: grid_heuristic(state, goal_position))

# if path:
#     print("Path found:", path)
# else:
#     print("No path found.")
    
# print(grid_neighbors([1,2]))

# start_position = (0, 0)
# goal_position = (0.5, 0.5)
# resolution = 0.1

# path = Robot.astar_search(
#     start_position,
#     goal_position,
#     lambda node: Robot.grid_neighbors(node, resolution),
#     lambda node, goal: Robot.grid_heuristic(node, goal),
#     lambda q1, q2, links, links_type: Robot.cost_intersection5(q1, q2, links, links_type)
# )

# if path:
#     print("Path found:", path)
# else:
#     print("No path found.")