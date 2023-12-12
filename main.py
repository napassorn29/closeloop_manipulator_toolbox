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

# # L1 = link('A','fixed',np.array(['a','b']),np.array([[0,10],[0,0]]))
# # L2 = link('B','input',np.array(['a','e','g']),np.array([[0,3,2],[0,0,1]]))
# # L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,10],[0,0]]))
# # L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,3],[0,0]]))

# Robot = closedLoopMani([L1,L2,L4,L5])

L1 = link('A','fixed',np.array(['a','b']),np.array([[0,3],[0,0]]))
L2 = link('B','input',np.array(['a','e','g']),np.array([[0,4,2],[0,0,1]]))
L3 = link('C','input',np.array(['b','c','f']),np.array([[0,4,2],[0,0,-1]]))
L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
L5 = link('E','intermediate',np.array(['c','d']),np.array([[0,4],[0,0]]))

# Robot = closedLoopMani([L1,L2,L3,L4,L5])

# L1 = link('A','fixed',np.array(['a','b']),np.array([[0,61],[0,0]]))
# L2 = link('B','input',np.array(['a','e','g']),np.array([[0,19.315,2],[0,0,1]]))
# L3 = link('C','input',np.array(['b','c','f']),np.array([[0,16.762,2],[0,0,-1]]))
# L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,31.616],[0,0]]))
# L5 = link('E','intermediate',np.array(['c','d']),np.array([[0,31.342],[0,0]]))

Robot = closedLoopMani([L1,L2,L3,L4,L5])

###################################################################################################
##### forward kinematic of Robot ##################################################################
# q = [2.1267784793154387]
# mode = "positive"
# output = Robot.fk(q,'d',mode)

# q = [np.pi/2+np.pi/6,np.pi/2-np.pi/6]
# mode = "positive"
# output = Robot.fk([np.pi/2,np.pi/3],'d','positive')
# print(output)

plt.figure(figsize=(6, 6))

# print(output_fk)
T_desired = np.array([[2.832],[6.825]])
# T_desired = np.array([[32.97],[38.69]])
q_sol = Robot.ik(T_desired,'d','positive',0.1)
print("q_sol = ",q_sol)

Robot.plot(q_sol,"positive")

nei = Robot.grid_neighbors([0,0], 1)
print(nei)

traj_q,q1_values,q2_values,minmax = Robot.path_ik5(np.array([[4.714],[-0.5257]]),np.array([[2.832],[6.825]]),'d','++',0.01,0.01)
# traj_q,q1_values,q2_values,minmax = Robot.path_ik5(np.array([[48.69],[11.7]]),np.array([[32.97],[38.69]]),'d','++',0.01,0.01)
# print("q1:",q1)
# print("q2:",q2)
print("q_all:",minmax)
print("traj_q:",traj_q)

# Robot.animationik(0.01,0.005,5,np.array([[48.69],[11.7]]),np.array([[32.97],[38.69]]),'d','++',0.01,0.01)
Robot.animationik(0.01,0.005,5,np.array([[4.714],[-0.5257]]),np.array([[2.832],[6.825]]),'d','++',0.01,0.01)
# print("q1:",q1)
# print("q2:",q2)
plt.show()