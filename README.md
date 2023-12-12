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
**Variable**
- Name: Name of link.
- Type: Type of link (fixed, input, and intermediate).
- Joint_name: Name of the joint in each link.
- Joint_pos: Position of each joint compared with the first joint in the link.

**Method**
- # *is_connectable(other_link) --> bool*:
  Checking connection between self and other link class by using joint name
- # *connect(other_link)* --> connected_name, connected_position, connect_dist_from_ref :
  Return connection property between self and other link

### **clmanipulator_toolbox.closedLoopMani(Object, Links: link)**
**Variable**
- Links: Link in robot that is defined above.

**Method**

## **Use Case**

