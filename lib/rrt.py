import numpy as np
from numpy import linalg as LA
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy

#from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
#from lib.IK_velocity import IK_velocity
#from lib.solveIK import IK

#from sklearn.neighbors import NearestNeighbors


fk = FK()
#ik = IK()

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # Parameters:
    max_num_iter = 10000
    epsilon = 0.15


    # initialize path
    path = []
    start_path = [start]
    goal_path = [goal]

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
    radius = np.array([0.088,0.088,0.088,0.088,0.088,0.088,0.088])

    # get the obstacles within your map:
    obstacles = map.obstacles
    box_increase_offset = 0.12

    def make_obstacle_robust(obstacles):
        # print(obstacles)
        if isinstance(obstacles, np.ndarray) == False:
            return obstacles
        else:
            for j in range(np.size(obstacles,0)):
                obstacles[:,0:3] = obstacles[:,0:3] - box_increase_offset
                obstacles[:,3:] = obstacles[:,3:] + box_increase_offset
        return obstacles

    # print(obstacles)
    obstacles = make_obstacle_robust(obstacles)
    # print(obstacles)

##### Blow things work: #############


    class node:
        '''
        Class for RRT node
        '''

        def __init__(self,q):
            self.q = np.array(q)
            self.final_node = False
            self.parent = None

        def make_final_node(self):
            self.final_node = True

    def get_distance_to_goal(node):
        return LA.norm(node.q-goal)

    def get_node_distance(node1, node2):
        return LA.norm(node1.q-node2.q)

    def check_if_configuration_is_allowed(node):
        #assert(len(node.q) == len(lowerLim)) ########## For testing only

        for i in range(len(lowerLim)):
            if node.q[i] < lowerLim[i] or node.q[i] > upperLim[i]:
                return False

        return True



    def get_nearest_node_index(node_list, random_node):
        dlist = [LA.norm(node.q-random_node.q) for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def generate_final_path(node_list): #Most likely works
        index_of_final_node = -1
        for i in range(len(node_list)):
            if node_list[i].final_node == True:
                index_of_final_node = i
                break
        if index_of_final_node == -1:
            print('No node has been marked as Final Node')
            assert(False)

        path = goal.reshape((7,1))
        node = node_list[index_of_final_node]
        while node.parent is not None:
            path = np.append(path, node.q.reshape((7,1)), axis=1)
            node = node_list[node.parent]

        path = np.append(path,node.q.reshape((7,1)), axis=1)

        path = np.fliplr(path)
        path = np.transpose(path)

        return path


    def is_robot_collided(node, obstacles):
        #Check whether the given configuration is allowed (ensure no self-collision)
        if check_if_configuration_is_allowed(node) == False:
            return True
        #Check for collision with an obstacle
        joint_positions,_ = fk.forward(node.q)
        for j in range(np.size(obstacles,0)):
            #check each pair of joint_positions
            for i in range(len(node.q)-1):
                linePt1 = np.array(joint_positions[i,:]).reshape((1,3))
                linePt2 = np.array(joint_positions[i+1,:]).reshape((1,3))
                box = np.array(obstacles[j,:])
                #print(linePt1)
                #print(linePt2)
                #print(box)
                iscollided = detectCollision(linePt1,linePt2, box)
                if iscollided == True:
                    return True

        return False

    def check_if_robot_will_collide_in_path(node1, node2):
        joint_positions_1,_ = fk.forward(node1.q)
        joint_positions_2,_ = fk.forward(node2.q)

        t = 0
        dt = 0.2
        while (t+dt) <= 1:
            q_0 = joint_positions_1+(joint_positions_2-joint_positions_1)*t
            q_1 = joint_positions_1+(joint_positions_2-joint_positions_1)*(t+dt)

            for j in range(np.size(obstacles,0)):
                #check each pair of joint_positions
                for i in range(len(node1.q)):
                    linePt1 = np.array(q_0[i,:]).reshape((1,3))
                    linePt2 = np.array(q_1[i,:]).reshape((1,3))
                    box = np.array(obstacles[j,:])
                    iscollided = detectCollision(linePt1,linePt2, box)

                    if iscollided[0] == True:
                        #print('Collision Detected')
                        return True
            t += dt
        return False

    def planning(max_num_iter, epsilon):
        node_list = []
        node_list.append(node(np.array(start)))

        for i in range(max_num_iter):
            random_node = generate_informed_random_node(node_list)
            nearest_index = get_nearest_node_index(node_list,random_node)
            nearest_node = node_list[nearest_index]

            new_node = steer(nearest_node, random_node)

            if not is_robot_collided(new_node, obstacles):
                #print('Happy')
                nears_idx = get_nearest_node_index(node_list, new_node)
                new_node.parent = nears_idx
                if not check_if_robot_will_collide_in_path(node_list[new_node.parent], new_node):
                    node_list.append(new_node)
                    print('Iteration: ', i, 'New Node Identified :)')
                    #print('Tree Length = ', len(node_list))
                    #########################################
                    node_nearest_to_goal_index = get_nearest_node_index(node_list, node(goal))
                    distance_to_goal = get_node_distance(node_list[node_nearest_to_goal_index], node(goal))
                    print('Remaining distance to goal: ', distance_to_goal)
                    ########################################
                    if get_distance_to_goal(new_node) < epsilon:
                        print('Converged')
                        node_list[-1].make_final_node()
                        configurations_from_start_to_goal = generate_final_path(node_list)

                        #return node_list, configurations_from_start_to_goal
                        return configurations_from_start_to_goal
                else:
                    print('Iteration: ', i, ' New Node results in Collision :(')
            if i == max_num_iter-1:
                node_nearest_to_goal_index = get_nearest_node_index(node_list, node(goal))
                node_list[node_nearest_to_goal_index].make_final_node()
                configurations_from_start_to_goal = generate_final_path(node_list)
                print('Did Not Converged - Better Luck Next Time :) ')
                return configurations_from_start_to_goal

        return None

    def generate_informed_random_node(node_list):
        if np.divmod(len(node_list),10) == 0:
            candidate_node = node(np.array(goal))
        elif np.divmod(len(node_list),2) == 0:
            # margin = 3
            # lowerLim = goal - margin
            # upperLim = goal + margin
            random_configuration = np.random.randint(lowerLim*100,high = upperLim*100)/100
            #print(random_configuration)
            candidate_node = node(random_configuration)
        else:
            margin = 0.2
            lowerLim = goal - margin
            upperLim = goal + margin
            random_configuration = np.random.randint(lowerLim*100,high = upperLim*100)/100
            candidate_node = node(random_configuration)
        #     #random_configuration = np.random.randint(lowerLim*100,high = upperLim*100)/100
        #     random_configuration = np.random.uniform(lowerLim, upperLim)
        #     candidate_node = node(random_configuration)


    # def generate_informed_random_node(node_list):
    #     if np.divmod(len(node_list),10) == 0:
    #         candidate_node = node(np.array(goal))
    #     else:
    #         margin = 0.2
    #         lowerLim = goal - margin
    #         upperLim = goal + margin
    #         random_configuration = np.random.randint(lowerLim*100,high = upperLim*100)/100
    #         #print(random_configuration)
    #         candidate_node = node(random_configuration)
    #     # else:
    #     #     #random_configuration = np.random.randint(lowerLim*100,high = upperLim*100)/100
    #     #     random_configuration = np.random.uniform(lowerLim, upperLim)
    #     #     candidate_node = node(random_configuration)

        return candidate_node

    def steer(from_node, to_node, max_extend_length=get_node_distance(node(goal), node(start))): #
        new_node = node(to_node.q)
        d = from_node.q-to_node.q
        dist = get_node_distance(from_node, to_node)
        if dist > max_extend_length:
            new_node.q = from_node.q - d /dist *max_extend_length
        return new_node




    # Here is where the magic happens:
    path = planning(max_num_iter, epsilon)


    # works very well
    # def check_if_robot_will_collide_in_path(node1, node2):
    #     joint_positions_1,_ = fk.forward(node1.q)
    #     joint_positions_2,_ = fk.forward(node2.q)
    #     for j in range(np.size(obstacles,0)):
    #         #check each pair of joint_positions
    #         for i in range(len(node1.q)):
    #             linePt1 = np.array(joint_positions_1[i,:]).reshape((1,3))
    #             linePt2 = np.array(joint_positions_2[i,:]).reshape((1,3))
    #             box = np.array(obstacles[j,:])
    #
    #             iscollided = detectCollision(linePt1,linePt2, box)
    #             if iscollided == True:
    #                 return True
    #
    #     return False


    # def get_random_node():
    #     random_configuration = np.random.randint(lowerLim*100,high = upperLim*100)/100
    #     #print(random_configuration)
    #     random_node = node(random_configuration)
    #     return random_node
    #
    # def get_random_node_near_goal(margin = 0.1):
    #     lowerLim = goal - margin
    #     upperLim = goal + margin
    #     random_configuration = np.random.randint(lowerLim*100,high = upperLim*100)/100
    #     #print(random_configuration)
    #     random_node = node(random_configuration)
    #     return random_node

    # def generate_informed_random_node(node_list, step_percent_bias,step_percent_random):
    #     goal_node = node(np.array(goal))
    #     if np.divmod(len(node_list),5) == 0:
    #         node_index_near_goal = get_nearest_node_index(node_list, goal_node)
    #         l = get_node_distance(node_list[node_index_near_goal], goal_node)
    #         N = (goal_node.q-node_list[node_index_near_goal].q)/l
    #         q_candidate = node_list[node_index_near_goal].q+step_percent_bias*l*N
    #         candidate_node = node(np.array(q_candidate))
    #     else:
    #         candidate_node = get_random_node()
    #         nearest_node_index = get_nearest_node_index(node_list, candidate_node)
    #         l = get_node_distance(candidate_node, node_list[nearest_node_index])
    #         N = (candidate_node.q-node_list[nearest_node_index].q)/l
    #         candidate_config = node_list[nearest_node_index].q+step_percent_random*l*N
    #         candidate_node = node(np.array(candidate_config))
    #
    #     return candidate_node


    # def generate_informed_random_node(number_of_nodes_in_tree, previous_node):
    #     #previous_node_distance_to_goal = get_distance_to_goal(previous_node)
    #     candidate_random_node = get_random_node()
    #     while get_node_distance(candidate_random_node, previous_node) > 0.1 and get_node_distance(candidate_random_node, previous_node) < 1 and get_distance_to_goal(candidate_random_node) > 100/(number_of_nodes_in_tree):
    #         candidate_random_node = get_random_node()
    #         #print(candidate_random_node.q)
    #     #print('Better Node Identified')
    #     print('Distance to goal from previous node: ', get_distance_to_goal(previous_node))
    #     print('Distance to goal from candidate random node: ', get_distance_to_goal(candidate_random_node))
    #     print('How close new node must be to goal: ', get_distance_to_goal(candidate_random_node)*100/(number_of_nodes_in_tree))
    #     return candidate_random_node

    # Works very well
    # def planning(max_num_iter, epsilon):
    #     node_list = []
    #     node_list.append(node(np.array(start)))
    #     # count the number of nodes that are added to the tree
    #     number_of_nodes_in_tree = 1
    #     for i in range(max_num_iter):
    #         #random_node = get_random_node()
    #         #print('Iteration XXXXXXXXXXXX')
    #         #print('______________________________________________')
    #         random_node = generate_informed_random_node(node_list, 0.4,0.5)
    #
    #         if is_robot_collided(random_node, obstacles) == False:
    #             nearest_index = get_nearest_node_index(node_list,random_node)
    #             nearest_node = node_list[nearest_index]
    #             random_node.parent = nearest_index
    #             node_list.append(random_node)
    #             if get_distance_to_goal(random_node) < epsilon:
    #                 node_list[-1].make_final_node()
    #                 configurations_from_start_to_goal = generate_final_path(node_list)
    #                 print('Converged')
    #                 #return node_list, configurations_from_start_to_goal
    #                 return configurations_from_start_to_goal
    #         number_of_nodes_in_tree += 1
    #     return None

    # def planning(max_num_iter, epsilon):
    #     node_list = []
    #     node_list.append(node(np.array(start)))
    #     # count the number of nodes that are added to the tree
    #     #number_of_nodes_in_tree = 1
    #     for i in range(max_num_iter):
    #         #random_node = get_random_node()
    #         #print('Iteration XXXXXXXXXXXX')
    #         #print('______________________________________________')
    #         random_node = generate_informed_random_node(node_list, 0.2, 0.5)
    #         nearest_index = get_nearest_node_index(node_list,random_node)
    #         nearest_node = node_list[nearest_index]
    #         random_node.parent = nearest_index
    #         if is_robot_collided(random_node, obstacles) == False and check_if_robot_will_collide_in_path(random_node, nearest_node) == False:
    #
    #             node_list.append(random_node)
    #             if get_distance_to_goal(random_node) < epsilon:
    #                 node_list[-1].make_final_node()
    #                 configurations_from_start_to_goal = generate_final_path(node_list)
    #                 print('Converged')
    #                 #return node_list, configurations_from_start_to_goal
    #                 return configurations_from_start_to_goal
    #         #number_of_nodes_in_tree += 1
    #         else:
    #             print('Colision ', i, ' Detected')
    #         if i == max_num_iter-1:
    #             node_list[-1].make_final_node()
    #             configurations_from_start_to_goal = generate_final_path(node_list)
    #             print('Did Not Converged - Maybe you are stupid')
    #             return configurations_from_start_to_goal
    #     return None

    ##################### Testing ############################################
    # nlist, configurations = planning(1000,0.1)
    # i = 0
    # for i in range(len(nlist)):
    #    print('Configuration ', i, ': ', nlist[i].q.reshape((1,7)))
    #    print('Distance to goal of child: ', get_distance_to_goal(nlist[i]))
    #    if nlist[i].parent is not None:
    #        print('Distance to goal of parent: ', get_distance_to_goal(nlist[nlist[i].parent]))
    #
    #
    # #print('Nodes List:', nlist)
    # print('Start: ', start)
    # print('Goal: ', goal)
    # print('Configurations:')
    # print(configurations)


    #final_list = planning(500, 0.1)
    #print(final_list)

    #Testing Below
    '''
    node1 = node(np.array([1,0,1.3,-1.23, 1.7, 0,1]))
    node2 = node(np.array([2,0.1,1.3,-1.23, 1.7, 0,1]))
    node3 = node(np.array([3,0,1.3,-1.23, 1.7, 100,1]))
    node4 = node(np.array([4,5,5,-2, -4, 0,1]))
    node5 = node(np.array([5,-3,2,-0, 1, 0,3]))
    list = []

    list.append(node1)
    list.append(node2)
    list.append(node3)
    list.append(node4)
    list.append(node5)

    list[0].final_node = True

    for i in range(len(list)-1):
        list[i].parent = i+1

    for i in range(len(list)):
        print(list[i].q, list[i].parent, list[i].final_node)

    print('Running generate_final_path')
    print("___________________________")
    path = generate_final_path(list)

    print(path)

    print(start)
    print(goal)


    assert(False)
    '''

    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map2.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print(path)
