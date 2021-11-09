import numpy as np
from numpy import linalg as LA
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy

from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.IK_velocity import IK_velocity

#from sklearn.neighbors import NearestNeighbors
 
 
fk = FK()
def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # initialize path
    path = []
    start_path = [start]
    goal_path = [goal]

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
    radius = np.array([0.088,0.088,0.088,0.088,0.088,0.088,0.088])

    def node():   #checked, works well
    
        # Creates a random configuration q that respects the joint limits
        # No input needed
        
        random_node = random.uniform(lowerLim,upperLim)
        return random_node
        
    def d(a,b):   #checked, works well
    
       # Euclidean distance between a and b
       # a & b : (N,) numpy arrays
       
       distance = LA.norm(a - b)  
       return distance  
       
    def nearestneighbors(q_list,q_candidate):   #checked, works well
    # Naive nearest neighbors
        nn = 0
        dmin = d(q_list[0],q_candidate)
        for i in range(0,len(q_list)):  
           d_check =  d(q_candidate,q_list[i])
           if d_check < dmin:
               dmin = d_check
               nn = i 
        return nn       
     
    def conservative_node(q_candidate,nn):
           pass
 
    def line_closest_to_obstacle(A,B,diagonal,R):  #checked, works well
    
       # Finds the segment perpendicular to both the axis of the cylinder and the diagonal of the box. This is the direction of shortest distance. It then translates the axis to surface of the cylinder to check for collisions.
       # A : 1st joint coordinates, (3,) numpy array
       # B : 2nd joint coordinates, (3,) numpy array
       # diagonal : min and max coordinates of the digonal of the box, (6,) numpy array
       # R: assumed cylinder's radius, conservative approach
     
       diagA = diagonal[:3]
       diagB = diagonal[3:6]
       N1 = (B - A) / d(B,A)   
       N2 = (diagB - diagA) / d(diagB,diagA)
       N3 = np.cross(N2, N1)
       N3 = N3 / LA.norm(N3)  
       A_new = A + R*N3
       B_new = B + R*N3
       
       return A_new , B_new
       
    def isRobotCollided(q,map):     #NOT CHECKED YET EXTENSIVELY
    
       collision = False
       joint_positions, T0e = fk.forward(q)
       end_effector_position = np.array([T0e[:3,3]])
       points_of_interest = np.append(joint_positions,end_effector_position,axis=0)
       
       for i in range(0,7): 
         for j in range(0,len(map)):
         
           pt1,pt2=              line_closest_to_obstacle(points_of_interest[i],points_of_interest[i+1],map[j],R[i])
           
           collsion = detectCollision(pt1,pt2,map[j])
           
           if collision:
              break
         if collision:
              break
       
       
 
     
     
     ##IGNORE EVERYTHING BELOW THIS POINT
    #print(line_closest_to_obstacle( np.array([1.125,4.525,4.572]), np.array([1.471,1.78,1.747]), np.array([1.75,0.5,-4.5,1.5,7.0,-0.36]),0.14))
    #print(fk.forward(node())[0])
    #print(map[0][0])
   # line_closest_to_obstacle(fk.forward(node())[0][0],fk.forward(node())[0][1],map[0][0])
    #print(upperLim+lowerLim)
    #print(upperLim)
    #print(lowerLim)
    #print(node_gen())
   # print(LA.norm(np.array([1,1.2,1,10]) - np.array([0.6,0.89,0.12,0.16])))
   # print(np.sqrt((1-0.6)**2+(1.2-0.89)**2+(1-0.12)**2+(10.-0.16)**2))
    #print (d(node(),node()))
    #print(np.sqrt((start_path[0][0]-x[0])**2+(start_path[0][1]-x[1])**2+(start_path[0][2]-x[2])**2+(start_path[0][3]-x[3])**2+(start_path[0][4]-x[4])**2+(start_path[0][5]-x[5])**2+(start_path[0][6]-x[6])**2))

   
    #for i in range(0,len(start_path)):
      #  print(start_path[i])

    #check_path =[node(),node(),node(),node(),node()]
    #print(check_path)
    #x=node()
    #print(x)
    #print(nearestneighbors(check_path,x))
    #print(d(check_path[0],x),d(check_path[1],x),d(check_path[2],x),d(check_path[3],x),
    #d(check_path[4],x))
 

    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
