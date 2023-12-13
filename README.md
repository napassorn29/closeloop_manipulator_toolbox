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
    - q(float type in 4-bar linkage, list type in 5-bar linkage): The angle of the input joint that user wants to rotate.
    - outputJoint(str): Name of joint that user wants to know the position.
    - mode(str): positive (Convex Solution), negative (Concave Solution)

  ![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/a984276a-fa37-43e7-b6f3-30dbc726ed75)

  
- #### *boundary4() --> list that contain minimum and maximum input value for 4-bar linkage*:

- #### *plot_boundary5(res) --> figure of avaliable workspace in input domain*:
  Return figure that represent avaliable pair of input that the close-loop kinematic chain can approach and not break the chain.
  - res(float): rotate resolution of input joint.
      
- #### *ik(T_desired, outputJoint, mode, tol, method) --> The configuration angle and mode of the input joint*:
  Return The configuration angle and mode of the input joint when determining the desired output joint position in the radian unit
  - T_desired(list): list of output joint positions that desired in x-y coordinated
  - method(str): numerical (Choose q that matched the minimize error), geometrical (Use geometric and trigonometry to find q)
  - mode(str): In numerical method can choose positive, negative.
               In geometrical method can choose positive (++), negative (--), (+-), (-+)
    
  ![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/96826b99-8b0e-4434-8a67-0c1d53d54b69)
  
  - tol(float): tolerance or error that the user can accept.
    
- #### *plot(q, mode) --> Plot bar linkage*:
  Plot bar linkage of the input bar and joint including the angle at the user defined in order to display the image
  - q(float): The input angle of the input joint (between Fixlink and Inputlink), the 4-bar linkage has 1 input angle and the 5-bar linkage has 2 input angles.
  - mode(str): positive (Convex Solution), negative (Concave Solution)
 
- #### *teach (mode) --> Animation of forward kinematics by bar linkage and the user can adjust desired input angle by sliding slide bar*
  - mode(str): positive (Convex Solution), negative (Concave Solution)

- #### *animationik(dt, tol, kp, q_init, taskspace_goal, joint_output, mode, tol_ik, res) --> Animation of inverse kinematic by bar linkage*:
  Animation plot of inverse kinematics, which is a continuous loop for visualizing the possible paths that can reach the desired position.
  - dt(float): small change in the independent variable t (time).
  - tol(float): minimun tolerance (error) that occurs in the position control process and user can accept that error.
  - kp(float): proportional gain value in PID controller.
  - q_init(list): initial position of input joint.
  - taskspace_goal(list): desired position in x-y plane.
  - joint_output(str): Name of joint that user wants to know the position.
  - mode(str): positive (Convex Solution), negative (Concave Solution)
  - tol_ik(float): tolerance or error that the user can accept.
  - res(float): rotate resolution of input joint.

  
  
## **Use Case**

###  **Closed loop kinematic chain of 4 Bar linkage**
#### Set closed loop manipulator of Robot
```python
# Define links properties in link class
L1 = link('A','fixed',np.array(['a','b']),np.array([[0,10],[0,0]]))
L2 = link('B','input',np.array(['a','e','g']),np.array([[0,3,2],[0,0,1]]))
L4 = link('D','intermediate',np.array(['e','d']),np.array([[0,10],[0,0]]))
L5 = link('E','intermediate',np.array(['b','d']),np.array([[0,3],[0,0]]))

# Constuct the robot
Robot = closedLoopMani([L1,L2,L4,L5])
```

#### Forward kinematic of Robot
```python
output = Robot.fk([2.1267784793154387],'d','positive')
print(output)
```
result:
```python
   0.9571   -0.2898    0         8.417     
   0.2898    0.9571    0         2.548     
   0         0         1         0
   0         0         0         1
```

#### 4-bar linkage plot
```python
Robot.plot([np.pi/4],'positive')
plt.show()
```
result:

![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122667170/6d469813-4516-411a-b2ab-60d2642e9c97)

#### Teach
```python
Robot.teach('positive')
```
result:

https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/c6a4276d-d811-432b-91d3-8fa062801bbc

#### Boundary of Robot
```python
bound = Robot.boundary4()
print(bound)
```
result :
```python
(-3.141592653589793, 3.141592653589793)
```

#### Inverse kinematic of Robot
```python
# Defind Taskspace 
desired_position = np.array([[10],[3]])

# Inverse Kinematic using Geometrical Method to find configuration space
q_sol_geo = Robot.ik(desired_position,'d',mode='up',method='geometrical')
print("Geometrical result: ",q_sol_geo)

# Inverse Kinematic using Numerical Method to find configuration space
q_sol_num = Robot.ik(desired_position,'d',mode='up',method='numerical')
print("Numerical result: ",q_sol_num)
```
result:
```python
Geometrical result:  ([1.5707963267948966], 'positive')
```

#### Animation inverse kinematics 
```python

```
result:

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
output = Robot.fk([np.pi/2,np.pi/3],'d','positive')
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
Robot.plot([np.pi/4+np.pi/2,np.pi/3],'positive')
plt.show()
```
result:

![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122667170/f320b95c-70fc-44e1-a007-0c237ea1552e)


#### Animation forward kinematics
```python

```
result:
#### Boundary of Robot
```python
Robot.plot_boundary5() 
```
result: the white area in the figure represent available configuration space and black area represent the unavailable configuration space.


![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122667170/b53ae8ad-3d7f-4590-a67b-3fe736eaa924)

#### Inverse kinematic of Robot
```python

```

#### Animation inverse kinematics
```python
Robot.animationik(0.01, 0.005, 5, np.array([[4.714],[-0.5257]]), np.array([[2.832],[6.825]]),'d','++', 0.01, 0.01)
plt.show()
```
result:

https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/a0aaf070-1c0d-465a-8595-6b736fb746f3




