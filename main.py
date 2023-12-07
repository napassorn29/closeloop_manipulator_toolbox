from clmanipulator_toolbox import link,closedLoopMani
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
import matplotlib.animation as animation

# L1 = link('A','fixed',np.array(['a','b']),np.array([[0,3],[0,0]]))
# L2 = link('B','input',np.array(['a','e','g']),np.array([[0,6,2],[0,0,1]]))
# L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
# L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,4],[0,0]]))

L1 = link('A','fixed',np.array(['a','b']),np.array([[0,10],[0,0]]))
L2 = link('B','input',np.array(['a','e','g']),np.array([[0,3,2],[0,0,1]]))
L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,10],[0,0]]))
L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,3],[0,0]]))

Robot = closedLoopMani([L1,L2,L4,L5])

q = [2.1267784793154387]
mode = "positive"
output = Robot.fk(q,'d',mode)

# L1 = link('A','fixed',np.array(['a','b']),np.array([[0,3],[0,0]]))
# L2 = link('B','input',np.array(['a','e','g']),np.array([[0,4,2],[0,0,1]]))
# L3 = link('C','input',np.array(['b','c','f']),np.array([[0,4,2],[0,0,-1]]))
# L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
# L5 = link('E','intermediate',np.array(['c','d']),np.array([[0,4],[0,0]]))

# Robot = closedLoopMani([L1,L2,L3,L4,L5])

# q = [np.pi/2+np.pi/6,np.pi/2-np.pi/6]
# mode = "positive"
# output = Robot.fk([np.pi/2+np.pi/6,np.pi/2-np.pi/6],'d','positive')
print(output)


# plt.figure(figsize=(6, 6))
# plt.grid(True)

bound = Robot.boundary4()
# print(bound)

# fig, ax = plt.subplots()
# bound = Robot.boundary4()
# q = np.linspace(bound[0], bound[1], 40)
# Robot_animate = Robot.plot([bound[0]],"positive")
# print("Robot animate : ",Robot.plot([bound[0]],"positive"))


plt.figure(figsize=(6, 6))
# plt.grid(True)

Robot.plot([bound[0]],"positive")
Robot.plot([bound[1]],"positive")

Robot.animationfk4(100, "positive",[-10,10],[-10,10])
plt.show()

