import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
import heapq
import math
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from scipy.optimize import minimize

class link():
    def __init__(self, Name: str, Type: str, Joint_name: (list,np.ndarray), Joint_pos: np.ndarray):
        # Name: Check if 'Name' is string
        if isinstance(Name,str):
            self.name = Name
        else:
            raise ValueError('Invalid link name. Link name must be string.')

        # Type: Check if 'Type' is either 'fixed', 'input', or 'intermediate'
        if Type == 'fixed' or Type == 'input' or Type == 'intermediate':
            self.type = Type
        else:
            raise ValueError('Invalid link type. Only \'fixed\', \'input\' and \'intermediate\' link types are possible.')
        
        # Joint_name: Check if 'Joint_name'is a list or numpy.ndarray of string
        if isinstance(Joint_name,(list,np.ndarray)):
            check_str = True
            for i in Joint_name:
                check_str = check_str and isinstance(i,str)
                if not check_str:
                    raise ValueError('Invalid joint name. Joint name must be list or numpy.ndarray of string.')
            
            if type(Joint_name) == np.ndarray:
                Joint_name = Joint_name.tolist()

            self.jointName = Joint_name
        else:
            raise ValueError('Invalid joint name. Joint name must be list or numpy.ndarray of string.')
        
        # Joint_pos: Check if 'Joint_pos'is a numpy.ndarray of numbers with matched shape and initial position
        if isinstance(Joint_pos,np.ndarray):
            if Joint_pos.shape[0] != 2 or Joint_pos.shape[1] != len(self.jointName):
                raise ValueError(f'Invalid joint position. Joint position must shape must be (2, {len(Joint_name)}). Current shape: {Joint_pos.shape}')
            if Joint_pos.dtype != int and Joint_pos.dtype != float:
                raise ValueError('Invalid joint position. Joint position must be numbers')
            if Joint_pos[0][0] == 0 and Joint_pos[1][0] == 0:
                self.jointPos = Joint_pos
            else:
                raise ValueError('Invalid joint position. Initial joint position must be (0, 0)')
        else:
            raise ValueError('Invalid joint position. Joint position must be numpy.ndarray.')
    
    def is_connectable(self, other_link)->bool:
    # Check if 2 Links have the same joints
        connectability = False
        for joint in self.jointName:
            if joint in other_link.jointName:
                connectability = True
                break
        return connectability
    
    def connect(self, other_link):
        connectability = False
        for joint in self.jointName:
            if joint in other_link.jointName:
                connected_name = joint
                connected_index = self.jointName.index(joint)
                connectability = True
                break
        if not(connectability):
            raise ValueError('Unable to connect links.')
        connected_position = np.array([[self.jointPos[0][connected_index]],[self.jointPos[1][connected_index]]])
        connected_dist_from_ref = (connected_position[0][0]**2 + connected_position[1][0]**2)**0.5

        return connected_name, connected_position, connected_dist_from_ref

    def jointPosition(self,j):
        if not(j in self.jointName):
            raise ValueError('Joint does not exist.')
        j_ind = self.jointName.index(j)
        return np.array([[self.jointPos[0][j_ind]],[self.jointPos[1][j_ind]]])
    
    def relative_jointPosition(self,j1,j2):
    # Calculate j2 position relative to j1
        if not(j1 in self.jointName) or not(j2 in self.jointName):
            raise ValueError('Joint does not exist.')
        return self.jointPosition(j2) - self.jointPosition(j1)

    def jointDist(self, j1, j2):
    # Calculate distance between 2 joints
        if not(j1 in self.jointName) or not(j2 in self.jointName):
            raise ValueError('Joint does not exist.')

        j1_pos = self.jointPosition(j1)
        j2_pos = self.jointPosition(j2)

        return ((j2_pos[1][0] - j1_pos[1][0])**2 + (j2_pos[0][0] - j1_pos[0][0])**2)**0.5
    
    def jointOrien(self, j1, j2):
    # Calculate oreintation vector j1 -> j2 reference by x-axis of defined jlink
        if not(j1 in self.jointName) or not(j2 in self.jointName):
            raise ValueError('Joint does not exist.')

        j1_pos = self.jointPosition(j1)
        j2_pos = self.jointPosition(j2)

        return np.arctan2((j2_pos[1][0] - j1_pos[1][0]),(j2_pos[0][0] - j1_pos[0][0]))
    
    def rotateJoint(self,j,angle):
        if not(j in self.jointName):
            raise ValueError('Joint does not exist.')

        j_pos = self.jointPosition(j)
        rotation_matrix = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])

        return rotation_matrix @ j_pos

class closedLoopMani():
    def __init__(self, Links: link):
        self.nlinks = len(Links)
        self.links_type = []

        for Link in Links:
            self.links_type.append(Link.type)
        
        if self.links_type.count('fixed') != 1:
            raise ValueError('Closed-loop manipulator require only 1 fixed link.')
        
        if self.nlinks == 4:
            if self.links_type.count('input') != 1:
                raise ValueError(f'4-Bar manipulator require 1 input link. Current number of input link: {self.links_type.count("input")}')
            
            self.links = Links
            self.fix_link = self.links[self.links_type.index('fixed')]
            self.input_link = self.links[self.links_type.index('input')]

            intermediate_link = self.links.copy()
            intermediate_link.remove(self.fix_link)
            intermediate_link.remove(self.input_link)

            self.j1, self.j1_pos, j1_refDist = self.fix_link.connect(self.input_link)
            for Link in intermediate_link:
                if self.fix_link.is_connectable(Link):
                    self.j2, self.j2_pos, j2_refDist = self.fix_link.connect(Link)
                    self.intermediate_link_1 = Link
                if self.input_link.is_connectable(Link):
                    self.j3 = self.input_link.connect(Link)[0]
                    self.intermediate_link_2 = Link
            self.j4 = intermediate_link[0].connect(intermediate_link[1])[0]

            self.l1 = self.fix_link.jointDist(self.j1,self.j2)
            self.l2 = self.input_link.jointDist(self.j1,self.j3)
            self.l3 = self.intermediate_link_1.jointDist(self.j2,self.j4)
            self.l4 = self.intermediate_link_2.jointDist(self.j3,self.j4)

        elif self.nlinks == 5:
            if self.links_type.count('input') != 2:
                raise ValueError(f'5-Bar manipulator require 2 input link. Current number of input link: {self.links_type.count("input")}')
            
            self.links = Links
            self.fix_link = self.links[self.links_type.index('fixed')]
            self.input_link_1 = self.links[self.links_type.index('input')]
            self.input_link_2 = self.links[self.links_type.index('input',self.links_type.index('input')+1)]

            intermediate_link = self.links.copy()
            intermediate_link.remove(self.fix_link)
            intermediate_link.remove(self.input_link_1)
            intermediate_link.remove(self.input_link_2)

            self.j1, self.j1_pos, j1_refDist = self.fix_link.connect(self.input_link_1)
            self.j2, self.j2_pos, j2_refDist = self.fix_link.connect(self.input_link_2)
            for Link in intermediate_link:
                if self.input_link_1.is_connectable(Link):
                    self.j3 = self.input_link_1.connect(Link)[0]
                    self.intermediate_link_1 = Link
                if self.input_link_2.is_connectable(Link):
                    self.j4 = self.input_link_2.connect(Link)[0]
                    self.intermediate_link_2 = Link
            self.j5 = intermediate_link[0].connect(intermediate_link[1])[0]

            self.l1 = self.fix_link.jointDist(self.j1,self.j2)
            self.l2 = self.input_link_1.jointDist(self.j1,self.j3)
            self.l3 = self.input_link_2.jointDist(self.j2,self.j4)
            self.l4 = self.intermediate_link_1.jointDist(self.j3,self.j5)
            self.l5 = self.intermediate_link_2.jointDist(self.j4,self.j5)

        Joints = []
        for Link in self.links:
            for joint in Link.jointName:
                Joints.append(joint)
        self.joints = list(set(Joints))

    def fk(self, q, outputJoint, mode = "positive"):
        if not(outputJoint in self.joints):
            raise ValueError(f'Output joint does not exist.')
        
        if self.nlinks == 4:
            if len(q) != 1:
                raise ValueError(f'4-Bar manipulator require 1 joint configuration.')
            return self.__fk4(q, outputJoint, mode)
        if self.nlinks == 5:
            if len(q) != 2:
                raise ValueError(f'5-Bar manipulator require 2 joint configurations.')
            return self.__fk5(q, outputJoint, mode)
            
    def __fk4(self, q, outputJoint, mode = "positive"):
        R1 = np.array([[self.j1_pos[0][0] + self.l2*np.cos(q[0])],[self.j1_pos[1][0] + self.l2*np.sin(q[0])]])
        R2 = np.array([[self.j2_pos[0][0]],[self.j2_pos[1][0]]])

        j3_pos = R1
        j4_pos = self.__circle_intersection(R1, R2, self.l4, self.l3, mode)

        if   outputJoint == self.j1: return self.__HMpose(self.j1_pos,np.arctan2((j3_pos[1][0] - self.j1_pos[1][0]),(j3_pos[0][0] - self.j1_pos[0][0])))
        elif outputJoint == self.j2: return self.__HMpose(self.j2_pos,np.arctan2((j4_pos[1][0] - self.j2_pos[1][0]),(j4_pos[0][0] - self.j2_pos[0][0])))
        elif outputJoint == self.j3: return self.__HMpose(j3_pos,np.arctan2((j4_pos[1][0] - j3_pos[1][0]),(j4_pos[0][0] - j3_pos[0][0])))
        elif outputJoint == self.j4: return self.__HMpose(j4_pos,np.arctan2((j4_pos[1][0]),(j4_pos[0][0])))
        else:
            if   outputJoint in self.fix_link.jointName           :  return self.fix_link.jointPosition(outputJoint)
            elif outputJoint in self.input_link.jointName         :  target_link = self.input_link         ; base_joint_pos = self.j1_pos; direction_joint_pos = j3_pos; original_orien = target_link.jointOrien(self.j1,self.j3); base_joint = self.j1
            elif outputJoint in self.intermediate_link_1.jointName:  target_link = self.intermediate_link_1; base_joint_pos = self.j2_pos; direction_joint_pos = j4_pos; original_orien = target_link.jointOrien(self.j2,self.j4); base_joint = self.j2
            elif outputJoint in self.intermediate_link_2.jointName:  target_link = self.intermediate_link_2; base_joint_pos = j3_pos; direction_joint_pos = j4_pos; original_orien = target_link.jointOrien(self.j3,self.j4); base_joint = self.j3
            else:                                               raise ValueError(f'Something went wrong.')
            
            orien = np.arctan2((direction_joint_pos[1][0] - base_joint_pos[1][0]),(direction_joint_pos[0][0] - base_joint_pos[0][0]))
            angle = orien - original_orien
            rotated_relative_position = target_link.rotateJoint(outputJoint,angle) - target_link.rotateJoint(base_joint,angle)

            return self.__HMpose(base_joint_pos + rotated_relative_position,angle)
    
    def __fk5(self, q, outputJoint, mode = "positive"):
        R1 = np.array([[self.j1_pos[0][0] + self.l2*np.cos(q[0])],[self.j1_pos[1][0] + self.l2*np.sin(q[0])]])
        R2 = np.array([[self.j2_pos[0][0] + self.l3*np.cos(q[1])],[self.j2_pos[1][0] + self.l3*np.sin(q[1])]])

        j3_pos = R1
        j4_pos = R2
        j5_pos = self.__circle_intersection(R1, R2, self.l4, self.l5, mode)

        if   outputJoint == self.j1: return self.__HMpose(self.j1_pos,np.arctan2((j3_pos[1][0] - self.j1_pos[1][0]),(j3_pos[0][0] - self.j1_pos[0][0])))
        elif outputJoint == self.j2: return self.__HMpose(self.j2_pos,np.arctan2((j4_pos[1][0] - self.j2_pos[1][0]),(j4_pos[0][0] - self.j2_pos[0][0])))
        elif outputJoint == self.j3: return self.__HMpose(j3_pos,np.arctan2((j5_pos[1][0] - j3_pos[1][0]),(j5_pos[0][0] - j3_pos[0][0])))
        elif outputJoint == self.j4: return self.__HMpose(j4_pos,np.arctan2((j5_pos[1][0] - j4_pos[1][0]),(j5_pos[0][0] - j4_pos[0][0])))
        elif outputJoint == self.j5: return self.__HMpose(j5_pos,np.arctan2((j5_pos[1][0]),(j5_pos[0][0])))
        else:
            if   outputJoint in self.fix_link.jointName           :  return self.fix_link.jointPosition(outputJoint)
            elif outputJoint in self.input_link_1.jointName       :  target_link = self.input_link_1       ; base_joint_pos = self.j1_pos; direction_joint_pos = j3_pos; original_orien = target_link.jointOrien(self.j1,self.j3); base_joint = self.j1
            elif outputJoint in self.input_link_2.jointName       :  target_link = self.input_link_2       ; base_joint_pos = self.j2_pos; direction_joint_pos = j4_pos; original_orien = target_link.jointOrien(self.j2,self.j4); base_joint = self.j2
            elif outputJoint in self.intermediate_link_1.jointName:  target_link = self.intermediate_link_1; base_joint_pos = j3_pos; direction_joint_pos = j5_pos; original_orien = target_link.jointOrien(self.j3,self.j5); base_joint = self.j3
            elif outputJoint in self.intermediate_link_2.jointName:  target_link = self.intermediate_link_2; base_joint_pos = j4_pos; direction_joint_pos = j5_pos; original_orien = target_link.jointOrien(self.j4,self.j5); base_joint = self.j4
            else:                                               raise ValueError(f'Something went wrong.')
            
            orien = np.arctan2((direction_joint_pos[1][0] - base_joint_pos[1][0]),(direction_joint_pos[0][0] - base_joint_pos[0][0]))
            angle = orien - original_orien
            rotated_relative_position = target_link.rotateJoint(outputJoint,angle) - target_link.rotateJoint(base_joint,angle)
            return self.__HMpose(base_joint_pos + rotated_relative_position,angle)

    def __circle_intersection(self, o1: np.ndarray, o2: np.ndarray, r1: (float, int), r2: (float, int), mode: str):
        # Center of circle 1
        h1 = o1[0][0]
        k1 = o1[1][0]
        # Center of circle 2
        h2 = o2[0][0]
        k2 = o2[1][0]

        # Coefficient calculation
        v1 = (r1**2 - r2**2 - h1**2 + h2**2 - k1**2 + k2**2)/(2*(h2 - h1))
        v2 = (k2 - k1)/(h2 - h1)
        v3 = 1 + v2**2
        v4 = 2*(v2*h1 - v1*v2 - k1)
        v5 = v1**2 + h1**2 - 2*v1*h1 + k1**2 - r1**2

        # Check if not intersected
        inRoot = v4**2 - 4*v3*v5
        if inRoot < 0:
            raise ValueError(f'Inputed circles do not intersected')

        # Calculate y according to mode of the solution
        if mode == "positive":
            y = (-v4 + (v4**2 - 4*v3*v5)**0.5)/(2*v3)
        elif mode == "negative":
            y = (-v4 - (v4**2 - 4*v3*v5)**0.5)/(2*v3)
        else:
            raise ValueError(f'Mode must be only "positive" or "negative". Current mode: {mode}')

        # Calculate x
        x = v1 - v2*y

        return np.array([[x],[y]]) 
    
    def is_circle_intersection(self, o1: np.ndarray, o2: np.ndarray, r1: (float, int), r2: (float, int)):
        # Center of circle 1
        h1 = o1[0][0]
        k1 = o1[1][0]
        # Center of circle 2
        h2 = o2[0][0]
        k2 = o2[1][0]

        # Coefficient calculation
        v1 = (r1**2 - r2**2 - h1**2 + h2**2 - k1**2 + k2**2)/(2*(h2 - h1))
        v2 = (k2 - k1)/(h2 - h1)
        v3 = 1 + v2**2
        v4 = 2*(v2*h1 - v1*v2 - k1)
        v5 = v1**2 + h1**2 - 2*v1*h1 + k1**2 - r1**2

        # Check if not intersected
        inRoot = v4**2 - 4*v3*v5
        if inRoot < 0:
            return False # Not Intersected
        else:
            return True # Intersected
    
    def __HMpose(self, pos, orien):
        T = SE3(pos[0][0], pos[1][0], 0)
        Rz = SE3.Rz(orien)
        return T * Rz
    
    def boundary4(self):
        if self.l1+self.l2 < self.l3+self.l4:
            q_min = 0
            q_max = 2 * np.pi
        else:
            q_max_pos = self.__circle_intersection(self.j1_pos, self.j2_pos, self.l2, self.l3+self.l4, "positive")
            q_min_pos = self.__circle_intersection(self.j1_pos, self.j2_pos, self.l2, self.l3+self.l4, "negative")

            q_min = np.arctan2(q_min_pos[1][0],q_min_pos[0][0])
            q_max = np.arctan2(q_max_pos[1][0],q_max_pos[0][0])

        return (q_min,q_max)
    
    def boundary5(self, res = 0.01):
        if self.l1+self.l2 < self.l3+self.l4+self.l5:
            q1_min = 0
            q1_max = 2 * np.pi
        else:
            q1_min = -np.arccos(((self.l1**2)+(self.l2**2)-((self.l3+self.l4+self.l5)**2))/(2*self.l1*self.l2))
            q1_max = np.arccos(((self.l1**2)+(self.l2**2)-((self.l3+self.l4+self.l5)**2))/(2*self.l1*self.l2))

        if self.l1+self.l3 < self.l2+self.l4+self.l5:
            q2_min = 0
            q2_max = 2 * np.pi
        else:
            q2_min = -np.arccos(((self.l1**2)+(self.l3**2)-((self.l2+self.l4+self.l5)**2))/(2*self.l1*self.l3))
            q2_max = np.arccos(((self.l1**2)+(self.l3**2)-((self.l2+self.l4+self.l5)**2))/(2*self.l1*self.l3))
        
        Cspace = []
        q1_space = []
        q2_space = []
        for q2 in range (int(2*np.pi/res)):
            CspaceRow = []
            for q1 in range (int(2*np.pi/res)):
                R1 = np.array([[self.j1_pos[0][0] + self.l2*np.cos(q1*res)],[self.j1_pos[1][0] + self.l2*np.sin(q1*res)]])
                R2 = np.array([[self.j2_pos[0][0] + self.l3*np.cos(q2*res)],[self.j2_pos[1][0] + self.l3*np.sin(q2*res)]])
                if self.is_circle_intersection(R1, R2, self.l4, self.l5) == True: # Intersected
                    CspaceRow.append(255)
                    q1_space.append(q1*res)
                    q2_space.append(q2*res)
                else: # Not Intersected
                    CspaceRow.append(0)
            Cspace.append(CspaceRow)
        Cspace = np.array(Cspace)
        
        Cspace_reshaped = Cspace[:, :, np.newaxis]
        Cspace_reshaped = np.concatenate((Cspace_reshaped, Cspace_reshaped, Cspace_reshaped), axis=2)

        return Cspace_reshaped, q1_space, q2_space
    
    def plot_boundary5(self, res = 0.01):
        Cspace_reshaped, q1_space, q2_space = self.boundary5(res)
        plt.imshow(Cspace_reshaped, aspect='equal',origin='lower',extent=(0, int(2*np.pi/res)*res, 0, int(2*np.pi/res)*res))
        plt.xlabel('q1')
        plt.ylabel('q2')
        plt.xticks([0,int(np.pi/(2*res))*res,int(np.pi/res)*res,int(3*np.pi/(2*res))*res,int(2*np.pi/res)*res],[0,'π/2','π','3π/2','2π'])
        plt.yticks([0,int(np.pi/(2*res))*res,int(np.pi/res)*res,int(3*np.pi/(2*res))*res,int(2*np.pi/res)*res],[0,'π/2','π','3π/2','2π'])
        plt.grid(False)
        plt.show()
    
    def ik(self, T_desired : (list,np.ndarray), outputJoint, mode = 'positive', tol = 0.01, method = 'geometrical'):
        if self.nlinks == 4:
            if method == 'numerical':
                return self.__ik4_num(T_desired, outputJoint, tol)
            elif method == 'geometrical':
                return self.__ik4_geo(T_desired, outputJoint, mode, tol)
            else: 
                raise ValueError(f'Unavailable method.')
        if self.nlinks == 5:
            if method == 'numerical':
                return self.__ik5_num(T_desired, outputJoint, mode, tol)
            elif method == 'geometrical':
                if mode == 'positive':
                    mode = '++'
                elif mode == 'negative':
                    mode = '--'
                return self.__ik5_geo(T_desired, outputJoint, mode, tol)
            else: 
                raise ValueError(f'Unavailable method.')
    
    def __ik4_num(self, T_desired, outputJoint, mode = 'positive', tol = 0.01):
        if not(outputJoint in self.joints):
            raise ValueError(f'Output joint does not exist.')
        
        boundary = self.boundary4()
        initial_guess = [boundary[0]]

        def _objective(q):
            T_actual = self.fk(q,outputJoint)
            return np.linalg.norm(T_actual.A[0:2,3] - T_desired)
        
        q_min = boundary[0]
        q_max = boundary[1]
        for init_guess in initial_guess:
            result = minimize(_objective,init_guess, bounds=[(q_min, q_max) for _ in initial_guess])
        
        if tol >= _objective(result.x) :
            return result.x[0]
        else :
            raise ValueError(f'T_desired is out of workspace. Error reported: {_objective(result.x)}')

    def __ik4_geo(self, T_desired, outputJoint, mode = 'positive', tol = 0.01):
        if outputJoint == self.j1 or outputJoint == self.j2:
            raise ValueError(f'Specified outputJoint are on fixed link.')
        elif outputJoint == self.j3:
            if np.linalg.norm(T_desired - self.j1_pos) > self.l2 - tol and np.linalg.norm(T_desired - self.j1_pos) < self.l2 + tol:
                check = self.__ik2link(self.j2_pos, T_desired, self.l3, self.l4, mode)
                q1_neg, q1_pos = np.arccos((T_desired - self.j1_pos)[0][0],np.linalg.norm(T_desired - self.j1_pos))
                if T_desired[1][0] > 0: return q1_pos
                else: return q1_neg
            else:
                raise ValueError(f'T_desired is out of workspace')
        elif outputJoint == self.j4:
            if np.linalg.norm(T_desired - self.j2_pos) >= self.l3 - tol and np.linalg.norm(T_desired - self.j2_pos) < self.l3 + tol:
                q1, q3 = self.__ik2link(self.j1_pos, T_desired, self.l2, self.l4, mode)
                return q1
            else:
                raise ValueError(f'T_desired is out of workspace')
    
    def __ik5_num(self, T_desired, outputJoint, mode = 'positive', tol = 0.01):
        if not(outputJoint in self.joints):
            raise ValueError(f'Output joint does not exist.')
        
        C_space, q1_space, q2_space = self.boundary5()
        error = []

        for q1, q2 in zip(q1_space,q2_space):
            T_actual = self.fk([q1, q2], outputJoint, mode).A @ np.array([[0],[0],[0],[1]])
            error.append(np.linalg.norm(np.array([[T_actual[0][0]],[T_actual[1][0]]]) - T_desired))
            
        if tol >= min(error):
            min_error_index = error.index(min(error))
            result = [q1_space[min_error_index], q2_space[min_error_index]]
            return result
        else:
            raise ValueError(f'T_desired is out of workspace. Error reported: {min(error)}')
        
    def __ik5_geo(self, T_desired, outputJoint, mode = '++', tol = 0.01):
        if outputJoint != self.j5:
            raise ValueError(f'Geometric method for solving inverse kinematics of 5 bars closed-loop manipulator only capable for solving the end-effector position.')

        if mode == '++':
            mode1 = 'positive'
            mode2 = 'negative'
        elif mode == '+-':
            mode1 = 'positive'
            mode2 = 'positive'
        elif mode == '-+':
            mode1 = 'negative'
            mode2 = 'negative'
        elif mode == '--':
            mode1 = 'negative'
            mode2 = 'positive'

        q1, q3 = self.__ik2link(self.j1_pos, T_desired, self.l2, self.l4, mode1)
        q2, q4 = self.__ik2link(self.j2_pos, T_desired, self.l3, self.l5, mode2)

        return [q1, q2]

    def __ik2link(self, origin: np.ndarray, endEffector: np.ndarray , l1: (int, float), l2: (int, float),mode: str = 'up'):
        if np.linalg.norm(endEffector - origin) > (l1 + l2) or np.linalg.norm(endEffector - origin) < (l1 - l2):
            raise ValueError(f'T_desired is out of workspace')
        
        c2 = (- l1**2 - l2**2 + np.linalg.norm(endEffector - origin)**2)/(2*l1*l2)
        if mode == 'negative':
            s2 = np.sqrt(1 - c2**2)
        elif mode == 'positive':
            s2 = - np.sqrt(1 - c2**2)
        else:
            raise ValueError(f'Mode must be only "positive" or "negative". Current mode: {mode}')
        cs1 = np.array([[l1 + l2*c2, l2*s2],[-l2*s2, l1 + l2*c2]])/((l1 + l2*c2)**2 + (l2*s2)**2) @ (endEffector - origin)

        q1 = np.arctan2(cs1[1][0],cs1[0][0])
        q2 = np.arctan2(s2,c2)

        return (q1, q2)
    
    def minmax4(self, mode:str):
        posFilter = np.array([[0],[0],[0],[1]])
        bound = self.boundary4()
        q_value = np.linspace(bound[0],bound[1],100)
        minx,maxx,miny,maxy = 0,0,0,0
        joint = []
        for q in range(len(q_value)):
            for Link in self.links:
                for Joint in Link.jointName:
                    q_find = [q_value[q]]
                    output = self.fk(q_find,Joint,mode)
                    D = output.A @ posFilter
                    if D[0][0] < minx:
                        minx = D[0][0]
                    elif D[0][0] > maxx:
                        maxx = D[0][0]
                    if D[1][0] < miny:
                        miny = D[1][0]
                    elif D[1][0] > maxy:
                        maxy = D[1][0]
                    else:
                        continue
                # joint = joint.append(Joint)
        minmax = [minx,maxx,miny,maxy]
        return minmax
         
    def __plotLink(self,jointCoordinates:list):
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
        plt.plot(x_sorted, y_sorted)
        
    def plot(self,q:list,mode:str):
        plt.grid(True)
        plt.axis('equal')
        posFilter = np.array([[0],[0],[0],[1]])
        for Link in self.links:
            jointCoor = []
            for Joint in Link.jointName:
                output = self.fk(q,Joint,mode)
                D = output.A @ posFilter
                E = np.array([[D[0][0]],[D[1][0]]])
                jointCoor.append(E)
            self.__plotLink(jointCoor)
    
    def animationfk(self, frequency:int, mode:str):
        if self.nlinks == 4:
            return self.__animationfk4(frequency, mode)
        if self.nlinks == 5:
            return self.__animationfk4(frequency, mode)
    
    def __animationfk4(self, frequency:int, mode:str):
        fig, ax = plt.subplots()
        plt.grid(True)
        plt.axis('equal')
        minmax = self.minmax4(mode)
        bound = self.boundary4()
        q_values = np.linspace(bound[1],bound[0], frequency)
        posFilter = np.array([[0], [0], [0], [1]])
        
        plt.grid(True)
        ax.set_xlim(minmax[0]-1, minmax[1]+1)  # set x-axis limits 
        ax.set_ylim(minmax[2]-1, minmax[3]+1)  # set y-axis limits 

        def update(frame):
            ax.clear()  # Clear the previous frame
            plt.grid(True)
            plt.axis('equal')
            ax.set_xlim(minmax[0]-1, minmax[1]+1)  # Reset x-axis limits
            ax.set_ylim(minmax[2]-1, minmax[3]+1)  # Reset y-axis limits
            q = [q_values[frame]]  # Change the joint angles in each frame
            for Link in self.links:
                jointCoor = []
                for Joint in Link.jointName:
                    output = self.fk(q, Joint, mode)
                    D = output.A @ posFilter
                    E = np.array([[D[0][0]], [D[1][0]]])
                    jointCoor.append(E)
                self.__plotLink(jointCoor)

        # Create the animation
        frames = len(q_values)
        animation = FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
        plt.show()
        
    def animationik(self,dt:float, tol:float, kp:float, q_init:list,taskspace_goal:list,joint_output:str, mode:str, tol_ik:float, res:float = 0.01):
        if self.nlinks == 4:
            return self.__animationik4(dt, tol, kp, q_init,taskspace_goal,joint_output, mode, tol_ik)
        if self.nlinks == 5:
            return self.__animationik5(q_init,taskspace_goal,joint_output, mode, tol_ik, res)
    
    def P_control_ik4(self,dt:float, tol:float, kp:float, init:list,taskspace_goal:list,joint_output:str, mode:str, tol_ik:float):
        traj_q = []
        q = self.ik(init,joint_output, mode, tol_ik, method = 'geometrical' )
        q_goal = self.ik(taskspace_goal,joint_output, mode, tol_ik, method = 'geometrical' )
        error = q - q_goal
        while abs(error) > tol:
            error = q - q_goal
            v = (-error) * kp
            
            q = q + (v*dt)
            
            traj_q.append(q)
        
        traj_q = np.array(traj_q)
        return traj_q
    
    def __animationik4(self,dt:float, tol:float, kp:float, init:list,taskspace_goal:list,joint_output:str, mode:str, tol_ik:float):
        fig, ax = plt.subplots()
        plt.grid(True)
        plt.axis('equal')
        minmax = self.minmax4(mode)
        path = self.P_control_ik4(dt,tol,kp,init,taskspace_goal,joint_output,mode,tol_ik)
        posFilter = np.array([[0], [0], [0], [1]])
        
        ax.set_xlim(minmax[0]-2,minmax[1]+2)  # set x-axis limits 
        ax.set_ylim(minmax[2]-2,minmax[3]+2)  # set y-axis limits 

        def update(frame):
            ax.clear()  # Clear the previous frame
            plt.grid(True)
            plt.axis('equal')
            ax.set_xlim(minmax[0]-2,minmax[1]+2)  # Reset x-axis limits in 
            ax.set_ylim(minmax[2]-2,minmax[3]+2)  # Reset y-axis limits in
            q = [path[frame][0]]  # Change the joint angles in each frame
            for Link in self.links:
                jointCoor = []
                for Joint in Link.jointName:
                    output = self.fk(q, Joint, mode)
                    D = output.A @ posFilter
                    E = np.array([[D[0][0]], [D[1][0]]])
                    jointCoor.append(E)
                self.__plotLink(jointCoor)
            if frame == len(path) - 1:
                animation.event_source.stop()

        # Create the animation
        frames = len(path)
        animation = FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
        plt.show()
        
    def path_ik5(self, start: list, goal: list, joint_output: str, mode: str, tol_ik: float, res:float = 0.01):
        traj_q = self.plan_path(start, goal, joint_output, mode, res * 100)
        q1_values = [point[0] for point in traj_q]
        q2_values = [point[1] for point in traj_q]
        
        posFilter = np.array([[0],[0],[0],[1]])     
        minx,maxx,miny,maxy = 0,0,0,0
        joint = []
        for order in range(len(traj_q)):
            for Link in self.links:
                for Joint in Link.jointName:
                    q_find = traj_q[order]
                    output = self.fk(q_find,Joint,mode="positive")
                    D = output.A @ posFilter
                    if D[0][0] < minx:
                        minx = D[0][0]
                    elif D[0][0] > maxx:
                        maxx = D[0][0]
                    if D[1][0] < miny:
                        miny = D[1][0]
                    elif D[1][0] > maxy:
                        maxy = D[1][0]
                    else:
                        continue
        minmax = [minx,maxx,miny,maxy]
        return traj_q,q1_values,q2_values,minmax
    
    def __animationik5(self,start:list,goal:list,joint_output:str, mode:str, tol_ik:float, res:float = 0.01):
        fig, ax = plt.subplots()
        plt.grid(True)
        plt.axis('equal')
        traj_q,q1_values,q2_values,minmax = self.path_ik5(start,goal,joint_output, mode, tol_ik)
        frames = len(q1_values)
        posFilter = np.array([[0], [0], [0], [1]]) 
        ax.set_xlim(minmax[0]-2, minmax[1]+2) 
        ax.set_ylim(minmax[2]-2, minmax[3]+2) 
        def update(frame):
            ax.clear()  # Clear the previous frame
            plt.grid(True)
            plt.axis('equal')
            ax.set_xlim(minmax[0]-2, minmax[1]+2) 
            ax.set_ylim(minmax[2]-2, minmax[3]+2) 
            q1 = q1_values[frame]
            q2 = q2_values[frame]
            q_all = [traj_q[frame][0]]
            for Link in self.links:
                jointCoor = []
                for Joint in Link.jointName:
                    output = self.fk([q1,q2], Joint, mode = "positive")
                    D = output.A @ posFilter
                    E = np.array([[D[0][0]], [D[1][0]]])
                    jointCoor.append(E)
                self.__plotLink(jointCoor)
                
            if frame == len(traj_q) - 1:
                animation.event_source.stop()  # Stop the animation

        # Create the animation
        frames = len(q1_values)
        animation = FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
        plt.show()
        
    def cost_intersection5(self, q1:float, q2:float):
        R1 = np.array([[self.j1_pos[0][0] + self.l2*np.cos(q1)],[self.j1_pos[1][0] + self.l2*np.sin(q1)]])
        R2 = np.array([[self.j2_pos[0][0] + self.l3*np.cos(q2)],[self.j2_pos[1][0] + self.l3*np.sin(q2)]])
        if self.is_circle_intersection(R1, R2, self.l4, self.l5) == True: # Intersected
            cost = 1
        else: # Not Intersected
            cost = 1000000 
            
        return cost
    
    def grid_heuristic(self, start:list, goal:list):
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    
    def grid_neighbors(self, node:list, res:float):
        neighbors = []
        x, y = node
        multiplier = 100
        possible_neighbors = [(x + res, y), (x - res, y), (x, y + res), (x, y - res), (x + res, y + res), (x - res, y - res), (x + res, y - res), (x - res, y + res)]
        for neighbor in possible_neighbors:
            if 0 <= neighbor[0] < np.pi*2*multiplier and 0 <= neighbor[1] < np.pi*2*multiplier:
                neighbors.append(neighbor)
            else:
                neighbor0 = neighbor[0]
                neighbor1 = neighbor[1]
                if 0 > neighbor0:
                    neighbor0 = neighbor0 + (np.pi * 2 * multiplier)
                elif neighbor0 > np.pi*2*multiplier :
                    neighbor0 = neighbor0 - (np.pi * 2 * multiplier)
                elif  0 > neighbor1:
                    neighbor1 = neighbor1 + (np.pi * 2 * multiplier)
                elif neighbor1 > np.pi*2*multiplier:
                    neighbor1 = neighbor1 - (np.pi * 2 * multiplier)
                neighbors.append((neighbor0,neighbor1))
        return neighbors
    
    def a_star(self, start: list, goal: list, outputJoint: str, mode: str, res: float):
        def make_node(state, parent=None, cost=0, heuristic=0):
            return (tuple(state), parent, cost, heuristic)

        open_set = []
        closed_set = set()

        heuristic_fn = self.grid_heuristic(start, goal)
        start_node = make_node(start, cost=0, heuristic=heuristic_fn)

        heapq.heappush(open_set, (0, start_node))
        start = tuple(start)
        goal = tuple(goal)

        while open_set:
            current_cost, current_node = heapq.heappop(open_set)

            if current_node[0] == goal:
                path = []
                while current_node:
                    path.append(list(current_node[0]))  # Convert back to list for the final path
                    current_node = current_node[1]
                    path_list = path[::-1]
                    result_list = [[x / 100 for x in inner_list] for inner_list in path_list]
                return result_list
                # return current_node

            closed_set.add(tuple(current_node[0]))  # Convert to tuple before adding to set

            for neighbor in self.grid_neighbors(list(current_node[0]), res):  # Convert to list for the grid_neighbors call
                if tuple(neighbor) in closed_set:
                    continue
                cost_next = self.cost_intersection5(current_node[0][0],current_node[0][1])
                cost = current_node[2] + cost_next
                # cost = current_node[2] + self.cost_intersection5(*current_node[0], *neighbor)
                # print("current_node : ",current_node[0])
                heuristic_fn = self.grid_heuristic(neighbor, goal)
                new_node = make_node(neighbor, parent=current_node, cost=cost, heuristic=heuristic_fn)

                if all(tuple(neighbor) != node[0] for _, node in open_set):
                    heapq.heappush(open_set, (cost + heuristic_fn, new_node))
                elif cost < next(cost for c, n in open_set if n[0] == tuple(neighbor)):
                    open_set = [(c, n) if n[0] != tuple(neighbor) else (cost + heuristic_fn, new_node) for c, n in open_set]

        return None
        
    def plan_path(self, start:list, goal:list, outputJoint:str, mode:str,res:float):
        Cspace_reshaped, q1_space, q2_space = self.boundary5()
        q_start = np.array(self.ik(start, outputJoint, mode, 0.05, method = 'geometrical'))
        q_goal = np.array(self.ik(goal, outputJoint, mode, 0.05, method = 'geometrical'))
        
        start_index = (np.argmin(np.abs(q1_space - q_start[0])), np.argmin(np.abs(q2_space - q_start[1])))
        goal_index = (np.argmin(np.abs(q1_space - q_goal[0])), np.argmin(np.abs(q2_space - q_goal[1])))
        
        num_round = 2
        multiplier = 100
        start = [round(q1_space[start_index[0]],num_round)*multiplier, round(q2_space[start_index[1]],num_round)*multiplier]
        goal = [round(q1_space[goal_index[0]],num_round)*multiplier, round(q2_space[goal_index[1]],num_round)*multiplier]
        
        path_indices = self.a_star(start, goal, outputJoint, mode,res)

        return path_indices


    

        