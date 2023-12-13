from clmanipulator_toolbox import link,closedLoopMani
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3

Xbar = 5
# q = [-1.318116071652818]
q = [0.85, 6.04]
mode = "positive"
T_des = np.array([[9.294],[3.277]])
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

def plotLink(jointCoordinates):
    x_coords = []
    y_coords = []

    for i in range(len(jointCoordinates)):
        x_coords.append(jointCoordinates[i][0][0])
        y_coords.append(jointCoordinates[i][1][0])

    # Combine x and y coordinates into a list of tuples (points)
    points = list(zip(x_coords, y_coords))

    # Calculate centroid (average of x and y coordinates)
    centroid = [sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points)]

    # Function to calculate the angle between two points and the centroid
    def angle_with_centroid(point):
        x_diff = point[0] - centroid[0]
        y_diff = point[1] - centroid[1]
        return (np.arctan2(y_diff, x_diff) + 2 * np.pi) % (2 * np.pi)

    # Sort the points based on their angles relative to the centroid
    sorted_points = sorted(points, key=angle_with_centroid)

    # Extract x and y coordinates from the sorted points
    x_sorted, y_sorted = zip(*sorted_points)

    # Closing the polygon by repeating the first point
    x_sorted += (x_sorted[0],)
    y_sorted += (y_sorted[0],)

    # Plotting the non-intersecting polygon
    plt.axis('equal')
    plt.plot(x_sorted, y_sorted)

# Plot Robot ----------------------------------------------------------------
plt.figure(figsize=(6, 6))
plt.grid(True)

for Link in Robot.links:
    jointCoor = []
    for Joint in Link.jointName:
        temp = Robot.fk(q,Joint,mode).A @ np.array([[0],[0],[0],[1]])
        jointCoor.append(np.array([[temp[0][0]],[temp[1][0]]]))
    plotLink(jointCoor)

plt.show()
# Extract np.ndarray of position from Honogeneous Transformation Matrix Pose--
posFilter = np.array([[0],[0],[0],[1]])
D = output.A @ posFilter
E = np.array([[D[0][0]],[D[1][0]]])
print("Position vector in 2D np.ndarray ------------")
print(E)

# Boundary --------------------------------------------------------------------
print("Boundary ------------------------------------")
if Xbar == 4:
    bound = Robot.boundary4()
    print(bound)
elif Xbar == 5:
    Cspace = Robot.plot_boundary5()

print(Robot.minmax4("positive"))

# Inverse Kinematic --------------------------------------------------------------------
print("Inverse Kinematic ---------------------------")
result = Robot.ik(T_des,outputJoint,'negative', method = 'numerical', tol = 0.05)
print('Inverse Kinematic Result: ',result)
print(Robot.fk([result],outputJoint,'positive'))

# print(Robot.ik(output,'d',0.1))
    


# # T_desired = [2,2]
# # q_sol = Robot.ik(T_desired,'d')
# # print("q_sol = ", q_sol)


# # plt.figure()
# # Robot.boundary5()

