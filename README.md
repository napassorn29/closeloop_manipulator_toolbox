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
define link property such as joint name, joint position etc. to construct close-loop kinematic chain class


**Variable**
- Name: Name of link.
- Type: Type of link (fixed, input, and intermediate).
- Joint_name: Name of the joint in each link.
- Joint_pos: Position of each joint compared with the first joint in the link.

**Method**
- #### *is_connectable(other_link) --> bool*:
  Checking connection between self and other link class by using joint name
- #### *connect(other_link)* --> connected_name, connected_position, connect_dist_from_ref :
  Return connection property between self and other link

### **clmanipulator_toolbox.closedLoopMani(Object, Links: link)**
define close-loop kinematic chain that using link object to generated property of close-loop kinematic chain (4-bar linkage, 5-bar linkage).


**Variable**
- Links: Link in robot that is defined above.

**Method**
- #### *fk(q, outputjoint, mode) --> Homogeneous matrix of outputjoint*:
  Return Homogeneous matrix that describe position and orientation of the outputjoint that return from forward kinematic of the close-loop kinematic by define the rotation of the input joint (Note: require 1 input for 4-bar linkage and 2 input for 5-bar linkage)
  
- #### *boundary4() --> list that contain minimum and maximum input value for 4-bar linkage*:
  
- #### *plot_boundary5(res) --> figure of avaliable workspace in input domain*:
  Return figure that represent avaliable pair of input that the close-loop kinematic chain can approach and not break the chain.
  
- #### *ik(T_desired, outputJoint, mode, tol, method) --> The angle of the input joint*:
  
  Return The angle of the input joint when determining the desired output joint position in the radian unit.
  
  T_desired: Position of desired output joint
  
  method: numerical, geometrical
  
  mode: In numerical method can choose positive, negative,
        In geometrical method can choose positive (++), negative (--), (+-), (-+) 
  ![image](https://github.com/napassorn29/closeloop_manipulator_toolbox/assets/122891621/96826b99-8b0e-4434-8a67-0c1d53d54b69)
  
  tol: tolerance or error that the user can accept.
  
  
## **Use Case**

