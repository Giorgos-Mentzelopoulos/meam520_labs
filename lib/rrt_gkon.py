import numpy as np
from numpy import linalg as LA
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy

from lib.calculateFK import FK

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

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
    radius = np.array([0.07,0.07,0.07,0.1,0.07,0.1,0.13,0.07,0.11])
    length = np.array([0.0,0.24,0.00,0.0,0.24,0.0,0.0,0.22,0.13])

    radius = 1.0*np.array([0.07,0.07,0.1,0.1,0.13,0.11,0.07,0.07,0.07])
    length = np.array([0.0,0.0,0.0,0.0,0.0,0.13,0.24,0.24,0.24])

    def node():   #checked, works well

        # Creates a random configuration q that respects the joint limits
        # No input needed

        random_node = np.random.uniform(0.95*lowerLim,0.95*upperLim)
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

    def expand(q_list,q_candidate, expansion_type, step_percent):

      # Expands the tree randomly if expansion_type == random or
      # with bias else.

        if expansion_type == "random" :
           nn = nearestneighbors(q_list,q_candidate)
           l = d(q_list[nn],q_candidate)
           #print(q_candidate)
           #print(q_list[nn])
           N = (q_candidate - q_list[nn]) / l
           q_candidate = q_list[nn] + step_percent*l*N
           #print(q_candidate)
        else:
           nn = nearestneighbors(q_list,goal)
           l = d(q_list[nn],goal)
           N = (goal - q_list[nn]) / l
           q_candidate = q_list[nn] + step_percent*l*N
        return q_candidate , nn

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
       #print(N1)
       N3 = np.cross(N2, N1)
       if LA.norm(N3) > 0.0 :
          N3 = N3 / LA.norm(N3)
          A_new = A + R*N3
          B_new = B + R*N3
       #print((B_new - A_new) / d(B_new,A_new))
       #print(d(A_new,A),d(B_new,B))
       else:
          A_new = A
          B_new = B
       return A_new , B_new

    def create_poi_obstacle_collision(q_candidate,map):

      joint_positions,T0e,T0,T1,T2,T3,T4,T5,T6,Te = fk.forward(q_candidate)

      end_effector_position = np.array([T0e[:3,3]])

      #joint_positions[-1] = np.dot(T0@T1@T2@T3@T4@T5@T6,np.array([0.0,0.0,-0.11,1.0]))[:3]
      joint_positions = np.append(joint_positions , end_effector_position,axis=0)

      joint_positions = np.append(joint_positions , np.array([np.dot(T0@T1,np.array([0.0,0.0,0.12,1.0]))[:3]]),axis=0)
      joint_positions = np.append(joint_positions , np.array([np.dot(T0@T1,np.array([0.0,0.0,-0.12,1.0]))[:3]]),axis=0)

      joint_positions = np.append(joint_positions , np.array([np.dot(T0@T1@T2@T3,np.array([0.0,0.0,0.12,1.0]))[:3]]),axis=0)
      joint_positions = np.append(joint_positions , np.array([np.dot(T0@T1@T2@T3,np.array([0.0,0.0,-0.12,1.0]))[:3]]),axis=0)

      joint_positions = np.append(joint_positions , np.array([np.dot(T0@T1@T2@T3@T4@T5,np.array([0.0,0.0,0.14,1.0]))[:3]]),axis=0)
      joint_positions = np.append(joint_positions , np.array([np.dot(T0@T1@T2@T3@T4@T5,np.array([0.0,0.0,-0.14,1.0]))[:3]]),axis=0)



      return joint_positions

    def isRobotCollided(q_candidate,map):

       collision = []

       poi = create_poi_obstacle_collision(q_candidate,map)
       poi_new =np.zeros((9,6))

       for j in range(0,len(map[0])):
         for i in range(0,9):
           if i <= 4 :
             poi_new[i,:3] = poi[i]
             poi_new[i,3:6] = poi[i+1]
           else:
             poi_new[5,:3] = poi[6]
             poi_new[5,3:6] = poi[7]

             poi_new[6,:3] = poi[8]
             poi_new[6,3:6] = poi[9]

             poi_new[7,:3] = poi[10]
             poi_new[7,3:6] = poi[11]

             poi_new[8,:3] = poi[12]
             poi_new[8,3:6] = poi[13]
             break

         # check for obstacles collisions
         diagonal =  map[0][j]
         diagonal_new = np.zeros(diagonal.shape)
         diagA = diagonal[:3]
         diagB = diagonal[3:6]
         N = (diagB - diagA) / d(diagB,diagA)
         diagonal_new[:3] = diagA - 0.05*d(diagB,diagA)
         diagonal_new[3:6] = diagB + 0.05*d(diagB,diagA)

         collision=detectCollision(poi_new[:,:3],poi_new[:,3:6],diagonal_new)

         if any(collision):
              break
       return collision


    def goal_reached(q_candidate):
       flag = False
       #print(d(goal,q_candidate))
       if d(goal,q_candidate) < 0.1: # < 0.01:
          flag = True
       return flag

    #Main program

    start_path = [start]
    parent = [0]
    path_found = False
    k=1    #iteration number

    while not path_found:
           q = node()

           if k % 10 == 0:
            if  k<10000:
             q , nn = expand(start_path,q,"biased",0.4)
            else:
             q , nn = expand(start_path,q,"biased",0.06)
           else:
             q , nn = expand(start_path,q,"random",0.8)
           k=k+1
           if not any(isRobotCollided(q,map)):
                parent.append(nn)
                start_path.append(q)
                #######################
                print("Iteration: ", k, "New Node Appended :)")
                nn = nearestneighbors(start_path,goal)
                print("Distance to goal: ", d(start_path[nn], goal))
                ########################
                if k>50000:
                    break
                if goal_reached(q) :
                    path_found = True


    if   path_found :
          nn = nearestneighbors(start_path,goal)
          start_path.append(goal)
          parent.append(nn)
          nn = parent[-1]
          path.append(goal)
          creation_done = False

          while not creation_done :

           if nn == 0:
             creation_done = True

           path.append(start_path[nn])
           nn = parent[nn]

    else:
         path = [goal,start]

    path.reverse()
    print("Congradulations! Path has been found")
    print("Path is: ")
    print(path,len(path))



    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map2.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    #start = np.array([0,0,0,-np.pi/2,0,np.pi/2,np.pi/4])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
