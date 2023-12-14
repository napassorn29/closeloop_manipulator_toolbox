# closeloop_manipulator_toolbox : Project Kinematic (FRA333)
โปรเจคนี้เป็นโปรเจคสำหรับจัดทำ Toolbox เพื่อใช้ในการคำนวณตำแหน่งปลาย End Effector (Task Space) ของ 
Closed Loop Kinematic Chain สำหรับ 4 และ 5 Bar Linkage เมื่อทราบความยาวของ Link และชนิดของ Link 
ต่างๆ (Input Link, Intermediate Link, Fixed Link, Output Link) และเพื่อศึกษาการคำนวณของหา 
Task Space ของ Closed Loop Kinematic Chain โดยใช้ DH parameter โดยผลลัพธ์ที่ได้จะมีในส่วนของตำแหน่งปลาย 
End Effector (Task Space) จะทราบทั้ง Translation, Rotation และมีการแสดงผลเป็นภาพเพื่อให้สามารถง่ายต่อการสังเกต
## **Objective**
1) เพื่อศึกษา Closed Loop Kinematic Chain ในปริภูมิ 2 มิติ สำหรับ 4-Bar Linkage และ 5-Bar Linkage
2) เพื่อพัฒนา Toolbox สำหรับการทำ Forward Kinematics, Inverse Kinematics และ Simulation ของ Closed Loop Kinematic 
Chain ในปริภูมิ 2 มิติ สำหรับ 4-Bar Linkage และ 5-Bar Linkage
## **System Overview (Function and Argument)**
![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/119843578/5e3f9422-06f0-442e-a4d6-933726595912)
## **Installation**
download this file (closeloop_manipulator_toolbox) and place this file in the folder to be used.
## **Usage**
place this code (import) at the top of the file to use
```python
# Import file link and closedLoopMani from clmanipulator_toolbox
from clmanipulator_toolbox import link,closedLoopMani

# Import numpy
import numpy as np

# Import matplotlib for plot and animation
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Import spatialmath
from spatialmath import SE3
```
## **Function**
### **clmanipulator_toolbox.link(Object, Name: str, Type: str, Joint_name: (list,np.ndarray), Joint_pos: np.ndarray)**
define link properties such as joint name, joint position etc. to construct the link for close-loop kinematic chain class


**Variable**
- Name: Name of link.
- Type: Type of link (fixed, input, and intermediate).
- Joint_name: Name of the joint in each link.
- Joint_pos: Each joint's position is compared with the link's first joint.

**Method**
- #### *is_connectable(other_link) --> bool*:
  Checking the connection between self and other link class by using joint name
- #### *connect(other_link)* --> connected_name, connected_position, connect_dist_from_ref :
  Return connection property between self and other link

### **clmanipulator_toolbox.closedLoopMani(Object, Links: link)**
define close-loop kinematic chain that using link object to generated property of close-loop kinematic chain (4-bar linkage, 5-bar linkage).


**Variable**
- Links: Link in robot that is defined above.

**Method**
- #### *fk(q, outputJoint, mode) --> Homogeneous matrix of outputjoint*:
  Return Homogeneous matrix that describe position and orientation of the outputjoint that return from forward kinematic of the close-loop kinematic by define the rotation of the input joint (Note: require 1 input for 4-bar linkage and 2 input for 5-bar linkage)
    - q(list or np.ndarray): The angle of the input joint that user wants to rotate.
    - outputJoint(str): Name of joint that user wants to know the position.
    - mode(str): positive (Convex Solution), negative (Concave Solution)

  ![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/a984276a-fa37-43e7-b6f3-30dbc726ed75)
  
- #### *boundary(res) --> list that contain minimum and maximum input value for 4-bar linkage*:
  - res(float): rotate resolution of input joint.

- #### *plot_boundary5(res) --> figure of avaliable workspace in input domain*:
  Return figure that represent avaliable pair of input that the close-loop kinematic chain can approach and not break the chain.
  - res(float): rotate resolution of input joint.
      
- #### *ik(T_desired, outputJoint, mode, method) --> The configuration angle and mode of the input joint*:
  Return The configuration angle and mode of the input joint when determining the desired output joint position in the radian unit
  - T_desired(list, np.ndarray): list of output joint positions that are desired in x-y coordinated
  - method(str): geometrical (Use geometric and trigonometry to find q), numerical (Choose q that matched the minimize error)
  - mode(str): In numerical method can choose up or down.
               In geometrical method can choose up (++), down (--), (+-), (-+)
    
  ![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/96826b99-8b0e-4434-8a67-0c1d53d54b69)
    
- #### *plot(q, mode) --> Plot bar linkage*:
  Plot bar linkage of the input bar and joint including the angle at the user defined in order to display the image
  - q(list): The input angle of the input joint (between Fixlink and Inputlink), the 4-bar linkage has 1 input angle and the 5-bar linkage has 2 input angles.
  - mode(str): positive (Convex Solution), negative (Concave Solution)
 
- #### *teach (mode) --> Animation of forward kinematics by bar linkage and the user can adjust desired input angle by sliding slide bar*
  - mode(str): positive (Convex Solution), negative (Concave Solution)

- #### *animationik(dt, tol, kp, taskspace_init, taskspace_goal, joint_output, mode, tol_ik, res) --> Animation of inverse kinematic by bar linkage*:
  Animation plot of inverse kinematics, which is a continuous loop for visualizing the possible paths that can reach the desired position.
  - dt(float): a small change in the independent variable t (time).
  - tol(float): minimum tolerance (error) that occurs in the position control process and user can accept that error.
  - kp(float): proportional gain value in PID controller.
  - taskspace_init(np.ndarray): initial position of input joint in x-y plane.
  - taskspace_goal(np.ndarray): desired position in x-y plane.
  - joint_output(str): Name of joint that the user wants to know the position.
  - mode(str): positive (Convex Solution), negative (Concave Solution)
  - tol_ik(float): tolerance or error that the user can accept.
  - res(float): rotate resolution of input joint.

  - #### *path(dt, tol, kp, taskspace_init, taskspace_goal, joint_output, mode, tol_ik, res) --> list of possible path*
  - dt(float): a small change in the independent variable t (time).
  - tol(float): minimum tolerance (error) that occurs in the position control process and user can accept that error.
  - kp(float): proportional gain value in PID controller.
  - taskspace_init(np.ndarray): initial position of input joint in x-y plane.
  - taskspace_goal(np.ndarray): desired position in x-y plane.
  - joint_output(str): Name of joint that the user wants to know the position.
  - mode(str): positive (Convex Solution), negative (Concave Solution)
  - tol_ik(float): tolerance or error that the user can accept.
  - res(float): rotate resolution of input joint.

  
  
## **Use Case**

###  **Closed loop kinematic chain of 4 Bar linkage**
#### Set closed loop manipulator of Robot
```python
# Define links properties in link class
L1 = link('A','fixed',np.array(['a','b']),np.array([[0,7],[0,0]]))
L2 = link('B','input',np.array(['a','e','g']),np.array([[0,6,3],[0,0,1]]))
L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,4],[0,0]]))

# Constuct the robot
Robot = closedLoopMani([L1,L2,L4,L5])
```

#### Forward kinematic of Robot
```python
output = Robot.fk(q=[0.7589975036567245], outputJoint='d', mode='positive')
print(output)
```
result:
```python
   0.9112   -0.412     0         8.337     
   0.412     0.9112    0         3.77
   0         0         1         0
   0         0         0         1
```

#### 4-bar linkage plot
```python
Robot.plot(q=[0.7589975036567245], mode='positive')
```
result:

![Figure_1](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/e61dd20c-9b4d-44ec-b20b-37b8d6e12944)


#### Teach
```python
Robot.teach(mode='positive')
```
result:

https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/ce9b74ec-4f89-4f6c-a380-4d75c58c889c

#### Boundary of Robot
```python
bound = Robot.boundary()
print(bound)
```
result :
```python
(-1.318116071652818, 1.318116071652818)
```

#### Inverse kinematic of Robot
```python
# Defind Taskspace 
desired_position = np.array([[6.091],[3.895]])

# Inverse Kinematic using Geometrical Method to find configuration space
q_sol_geo = Robot.ik(desired_position, outputJoint='d', mode='up', method='geometrical')
print("Geometrical result: ",q_sol_geo)

# Inverse Kinematic using Numerical Method to find configuration space
q_sol_num = Robot.ik(desired_position, outputJoint='d', mode='up', method='numerical')
print("Numerical result: ",q_sol_num) 
```
result:
```python
Geometrical result:  ([1.1551963634025209], 'positive')
Numerical result:  (array([1.15522984]), 'positive')
```

#### Animation inverse kinematics 
```python
Robot.animationik(dt=0.01, tol=0.1, kp=1.0, taskspace_init=np.array([[8.21],[3.81]]), taskspace_goal=np.array([[6.81],[3.99]]), joint_output='d', mode='up', tol_ik=0.1, res=0.01)
plt.show()
```
result:

https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/1e5318f0-2567-45fa-8b02-6477448947db

#### path:
```python
path = Robot.path(dt=0.01, tol=0.1, kp=1.0, taskspace_init=np.array([[8.21],[3.81]]), taskspace_goal=np.array([[6.81],[3.99]]), joint_output='d', mode='up', tol_ik=0.1, res=0.01)
print(path)
```

result:

```python
[0.78994982 0.79252851 0.79508141 0.79760878 0.80011088 0.80258795
 0.80504026 0.80746804 0.80987155 0.81225102 0.81460669 0.81693881
 0.81924761 0.82153331 0.82379617 0.82603639 0.82825421 0.83044985
 0.83262354 0.83477549 0.83690592 0.83901505 0.84110308 0.84317024
 0.84521672 0.84724274 0.84924849 0.85123419 0.85320004 0.85514622
 0.85707294 0.8589804  0.86086878 0.86273828 0.86458908 0.86642138
 0.86823535 0.87003118 0.87180905 0.87356915 0.87531164 0.87703671
 0.87874453 0.88043527 0.8821091  0.88376619 0.88540672 0.88703083
 0.88863871 0.89023051 0.89180639 0.89336651 0.89491103 0.8964401
 0.89795389 0.89945253 0.90093619 0.90240502 0.90385915 0.90529875
 0.90672394 0.90813489 0.90953173 0.91091459 0.91228363 0.91363898
 0.91498078 0.91630915 0.91762425 0.91892619 0.92021511 0.92149115
 0.92275442 0.92400506 0.9252432  0.92646895 0.92768245 0.92888381
 0.93007315 0.93125061 0.93241629 0.93357031 0.93471279 0.93584385
 0.93696359 0.93807214 0.93916961 0.9402561  0.94133172 0.94239659
 0.94345081 0.94449448 0.94552772 0.94655063 0.94756331 0.94856586
 0.94955839]
```



###  **Closed loop kinematic chain of 5 Bar linkage**
#### Set closed loop manipulator of Robot
```python
# Define links properties in link class
L1 = link('A','fixed',np.array(['a','b']),np.array([[0,3],[0,0]]))
L2 = link('B','input',np.array(['a','e','g']),np.array([[0,4,2],[0,0,1]]))
L3 = link('C','input',np.array(['b','c','f']),np.array([[0,4,2],[0,0,-1]]))
L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,4],[0,0]]))
L5 = link('E','intermediate',np.array(['c','d']),np.array([[0,4],[0,0]]))

# Constuct the robot
Robot = closedLoopMani([L1,L2,L3,L4,L5])
```

#### Forward kinematic of Robot
```python
output = Robot.fk(q=[np.pi/2,np.pi/3], outputJoint='d', mode='positive')
print(output)
```
result:
```python
   0.3832   -0.9237    0         2.832     
   0.9237    0.3832    0         6.825
   0         0         1         0
   0         0         0         1
```

#### 5-bar linkage plot
```python
Robot.plot(q=[np.pi/4+np.pi/2,np.pi/3], mode='positive')
plt.show()
```
result:

![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122667170/f320b95c-70fc-44e1-a007-0c237ea1552e)


#### Teach
```python
Robot.teach(mode='negative')
```
result:

https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/9167d179-346a-4dcc-983d-deb17723177d

#### Boundary of Robot
```python
Robot.plot_boundary5() 
```
result: the white area in the figure represent available configuration space and black area represent the unavailable configuration space.


![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122667170/b53ae8ad-3d7f-4590-a67b-3fe736eaa924)

#### Inverse kinematic of Robot
```python
# Defind Taskspace 
desire_position = np.array([[2.83],[6.82]])

# Inverse Kinematic using Geometrical Method to find configuration space
q_sol_geo = Robot.ik(T_desired=desire_position, outputJoint='d', tol=0.2, method='geometrical')
print("Geometrical result :",q_sol_geo)

# Inverse Kinematic using Numerical Method to find configuration space
q_sol_num = Robot.ik(T_desired=desire_position, outputJoint='d', tol=0.2, method='numerical')
print("Numerical result :",q_sol_num)
```
result:
```python
Geometrical result : ([1.5725021852414964, 1.046177731542845], 'positive')
Numerical result : ([1.6, 1.0], 'positive')
```

#### Animation inverse kinematics
```python
Robot.animationik(dt=0.01, tol=0.005, kp=5, taskspace_init=np.array([[4.714],[-0.5257]]), taskspace_goal=np.array([[2.832],[6.825]]), joint_output='d', mode='++', tol_ik=0.01, res=0.01)
plt.show()
```
result:

https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/a0aaf070-1c0d-465a-8595-6b736fb746f3

#### path:
```python
path = Robot.path(dt=0.01, tol=0.005, kp=5, taskspace_init=np.array([[4.714],[-0.5257]]), taskspace_goal=np.array([[2.832],[6.825]]), joint_output='d', mode='++', tol_ik=0.01, res=0.01)
print(path)
```

result:

```python
[[0.83, 0.0], [0.84, 0.01], [0.85, 0.02], [0.86, 0.03], [0.87, 0.04], [0.88, 0.05], [0.89, 0.06], [0.9, 0.07], [0.91, 0.08], [0.92, 0.09], [0.93, 0.1], [0.94, 0.11], [0.95, 0.12], [0.96, 0.13], [0.97, 0.14], [0.98, 0.15], [0.99, 0.16], [1.0, 0.17], [1.01, 0.18], [1.02, 0.19], [1.03, 0.2], [1.04, 0.21], [1.05, 0.22], [1.06, 0.23], [1.07, 0.24], [1.08, 0.25], [1.09, 0.26], [1.1, 0.27], [1.11, 0.28], [1.12, 0.29], [1.13, 0.3], [1.14, 0.31], [1.15, 0.32], [1.16, 0.33], [1.17, 0.34], [1.18, 0.35], [1.19, 0.36], [1.2, 0.37], [1.21, 0.38], [1.22, 0.39], [1.23, 0.4], [1.24, 0.41], [1.25, 0.42], [1.26, 0.43], [1.27, 0.44], [1.28, 0.45], [1.29, 0.46], [1.3, 0.47], [1.31, 0.48], [1.32, 0.49], [1.33, 0.5], [1.34, 0.51], [1.35, 0.52], [1.36, 0.53], [1.37, 0.54], [1.38, 0.55], [1.39, 0.56], [1.4, 0.57], [1.41, 0.58], [1.42, 0.59], [1.43, 0.6], [1.44, 0.61], [1.45, 0.62], [1.46, 0.63], [1.47, 0.64], [1.48, 0.65], [1.49, 0.66], [1.5, 0.67], [1.51, 0.68], [1.52, 0.69], [1.53, 0.7], [1.54, 0.71], [1.55, 0.72], [1.56, 0.73], [1.57, 0.74], [1.57, 0.75], [1.57, 0.76], [1.57, 0.77], [1.57, 0.78], [1.57, 0.79], [1.57, 0.8], [1.57, 0.81], [1.57, 0.82], [1.57, 0.83], [1.57, 0.84], [1.57, 0.85], [1.57, 0.86], [1.57, 0.87], [1.57, 0.88], [1.57, 0.89], [1.57, 0.9], [1.57, 0.91], [1.57, 0.92], [1.57, 0.93], [1.57, 0.94], [1.57, 0.95], [1.57, 0.96], [1.57, 0.97], [1.57, 0.98], [1.57, 0.99], [1.57, 1.0], [1.57, 1.01], [1.57, 1.02], [1.57, 1.03], [1.57, 1.04], [1.57, 1.05]]  
```




