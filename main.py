from clmanipulator_toolbox import link,closedLoopMani
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3

Xbar = 5
# q = [0.7589975036567245]
q = [1.6672289007241894, 1.1670998843915834]
mode = "negative"
T_des = np.array([[6.091],[3.895]])
outputJoint = 'd'

if Xbar == 4:
    L1 = link('A','fixed',np.array(['a','b']),np.array([[0,7],[0,0]]))
    L2 = link('B','input',np.array(['a','e','g']),np.array([[0,6,3],[0,0,1]]))
    L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
    L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,4],[0,0]]))

    Robot = closedLoopMani([L1,L2,L4,L5])
elif Xbar == 5:
    L1 = link('A','fixed',np.array(['a','b']),np.array([[0,8],[0,0]]))
    L2 = link('B','input',np.array(['a','e','g']),np.array([[0,8,4],[0,0,1]]))
    L3 = link('C','input',np.array(['b','c','f']),np.array([[0,8,4],[0,0,-1]]))
    L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,7],[0,0]]))
    L5 = link('E','intermediate',np.array(['c','d']),np.array([[0,7],[0,0]]))

    Robot = closedLoopMani([L1,L2,L3,L4,L5])

output = Robot.fk(q,outputJoint,mode)
print("The pose from forward kinematics ------------\n")
print(output)

# Plot Robot ----------------------------------------------------------------
Robot.plot(q,mode)
Robot.teach()

# Inverse Kinematic --------------------------------------------------------------------
print("Inverse Kinematic ---------------------------")
result, mode_flag = Robot.ik(T_des,outputJoint,'up', method = 'geometrical', tol = 0.01)
print('Inverse Kinematic Result: ',result, mode_flag)
print(Robot.fk(result,outputJoint, mode_flag))

Robot.animationik(0.01, 0.01, 1, np.array([[5.227],[3.586]]), np.array([[8.223],[3.808]]),'d','up', 0.5, 0.1)

path = Robot.path(0.01, 0.01, 1, np.array([[5.227],[3.586]]), np.array([[8.223],[3.808]]),'d', 'up', 0.5, 0.1)
print(path)
