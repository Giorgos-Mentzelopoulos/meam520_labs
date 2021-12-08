import numpy as np
from math import pi, acos
from scipy.linalg import null_space

from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.IK_velocity import IK_velocity

class IK:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self,linear_tol=1e-4, angular_tol=1e-3, max_steps=1500, min_step_size=1e-5):
        """
        Constructs an optimization-based IK solver with given solver parameters.
        Default parameters are tuned to reasonable values.

        PARAMETERS:
        linear_tol - the maximum distance in meters between the target end
        effector origin and actual end effector origin for a solution to be
        considered successful
        angular_tol - the maximum angle of rotation in radians between the target
        end effector frame and actual end effector frame for a solution to be
        considered successful
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # solver parameters
        self.linear_tol = linear_tol
        self.angular_tol = angular_tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################

    @staticmethod
    def displacement_and_axis(target, current):
        """
        Helper function for the End Effector Task. Computes the displacement
        vector and axis of rotation from the current frame to the target frame

        This data can also be interpreted as an end effector velocity which will
        bring the end effector closer to the target position and orientation.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        current - 4x4 numpy array representing the "current" end effector orientation

        OUTPUTS:
        displacement - a 3-element numpy array containing the displacement from
        the current frame to the target frame, expressed in the world frame
        axis - a 3-element numpy array containing the axis of the rotation from
        the current frame to the end effector frame. The magnitude of this vector
        must be sin(angle), where angle is the angle of rotation around this axis
        """

        ## STUDENT CODE STARTS HERE

        displacement = np.zeros(3)
        axis = np.zeros(3)

        displacement= target[0:3,3]-current[0:3,3]


        # unit vectors of current and target frames

        xc = current[0:3,0]
        yc = current[0:3,1]
        zc = current[0:3,2]
        xt = target[0:3,0]
        yt = target[0:3,1]
        zt = target[0:3,2]

        R = np.array([[xt@xc,xt@yc,xt@zc],[yt@xc,yt@yc,yt@zc],[zt@xc,zt@yc,zt@zc]])


        #print('.....TARGET.........')
        #print(target)
        #print('.....CURRENT...........')
        #print(current)
        #print('.....R...........')
        #print(R)
        S = (1/2)*(R-np.transpose(R))
        #print('.....S...........')
        #print(displacement)

        #axis = np.array([S[2,1],S[0,2],S[1,0]])
        #print('.......axis......')

        axis = np.array([-S[2,1],-S[0,2],-S[1,0]])
        axis = target[0:3,0:3]@axis
        #print(axis)
        #print(axis,np.sqrt(axis[0]**2+axis[1]**2+axis[2]**2))




        ## END STUDENT CODE

        return displacement, axis

    @staticmethod
    def distance_and_angle(G, H):
        """
        Helper function which computes the distance and angle between any two
        transforms.

        This data can be used to decide whether two transforms can be
        considered equal within a certain linear and angular tolerance.

        Be careful! Using the axis output of displacement_and_axis to compute
        the angle will result in incorrect results when |angle| > pi/2

        INPUTS:
        G - a 4x4 numpy array representing some homogenous transformation
        H - a 4x4 numpy array representing some homogenous transformation

        OUTPUTS:
        distance - the distance in meters between the origins of G & H
        angle - the angle in radians between the orientations of G & H


        """

        ## STUDENT CODE STARTS HERE

        distance = 0
        angle = 0

        distance = np.linalg.norm(G[0:3,3]-H[0:3,3])

        # unit vectors of current and target frames

        xG = G[0:3,0]
        yG = G[0:3,1]
        zG = G[0:3,2]
        xH = H[0:3,0]
        yH = H[0:3,1]
        zH = H[0:3,2]
        #print(H)
        R = np.array([[xG@xH,xG@yH,xG@zH],[yG@xH,yG@yH,yG@zH],[zG@xH,zG@yH,zG@zH]])
        #print(R,".............")
        trace = np.trace(R)

        if trace > 3 or trace < -1 :
          trace = np.trunc(trace)

        angle = acos((trace-1)/2)

        ## END STUDENT CODE

        return distance, angle

    def is_valid_solution(self,q,target):
        """
        Given a candidate solution, determine if it achieves the primary task
        and also respects the joint limits.

        INPUTS
        q - the candidate solution, namely the joint angles
        target - 4x4 numpy array representing the desired transformation from
        end effector to world

        OUTPUTS:
        success - a Boolean which is True if and only if the candidate solution
        produces an end effector pose which is within the given linear and
        angular tolerances of the target pose, and also respects the joint
        limits.
        """

        ## STUDENT CODE STARTS HERE

        success = False


        T0e_q = IK.fk.forward(q)[1]


        distance, angle = IK.distance_and_angle(target,T0e_q)


        if distance <= self.linear_tol:
            # print('condition1')

            if angle <= self.angular_tol:
               # print('condition2')


               joint_limits_satisfied = True

               for i in range(0,7):
                  if q[i] > IK.upper[i]:
                    joint_limits_satisfied = False

                    break

                  if q[i] < IK.lower[i]:
                    joint_limits_satisfied = False
                    break

               if  joint_limits_satisfied :
                    # print('condition3')
                    success = True

        ## END STUDENT CODE

        return success

    ####################
    ## Task Functions ##
    ####################

    @staticmethod
    def end_effector_task(q,target):
        """
        Primary task for IK solver. Computes a joint velocity which will reduce
        the error between the target end effector pose and the current end
        effector pose (corresponding to configuration q).

        INPUTS:
        q - the current joint configuration, a "best guess" so far for the final answer
        target - a 4x4 numpy array containing the desired end effector pose

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        ## STUDENT CODE STARTS HERE

        dq = np.zeros(7)
        #print('......................')
        #print(IK.fk.forward(q)[1])
        #print(q)
        #print('......................')
        v , omega = IK.displacement_and_axis(target,IK.fk.forward(q)[1])
        #print(q, v, omega)
        dq = IK_velocity(q, v, omega)
        #print(dq)
        #dq = np.fmod(dq,2*pi)
        #print(dq)
        ## END STUDENT CODE

        return dq

    @staticmethod
    def joint_centering_task(q,rate=5e-1):
        """
        Secondary task for IK solver. Computes a joint velocity which will
        reduce the offset between each joint's angle and the center of its range
        of motion. This secondary task acts as a "soft constraint" which
        encourages the solver to choose solutions within the allowed range of
        motion for the joints.

        INPUTS:
        q - the joint angles
        rate - a tunable parameter dictating how quickly to try to center the
        joints. Turning this parameter improves convergence behavior for the
        primary task, but also requires more solver iterations.

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # normalize the offsets of all joints to range from -1 to 1 within the allowed range
        offset = 2 * (q - IK.center) / (IK.upper - IK.lower)
        dq = -rate * offset # proportional term (implied quadratic cost)

        return dq

    ###############################
    ## Inverse Kinematics Solver ##
    ###############################

    def inverse(self, target, seed):
        """
        Uses gradient descent to solve the full inverse kinematics of the Panda robot.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        seed - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with optimization

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.
        success - True if the IK algorithm successfully found a configuration
        which achieves the target within the given tolerance. Otherwise False
        rollout - a list containing the guess for q at each iteration of the algorithm
        """

        # q = seed #np.array([-0.85566999, -1.00246097,  1.86687916, -1.49717099,  0.96796238,  1.77792418, 1.72581234])
        q = np.array([-0.85566999, -1.00246097,  1.86687916, -1.49717099,  0.96796238,  1.77792418, 1.72581234])

        rollout = []

        while True:

            rollout.append(q)

            # Primary Task - Achieve End Effector Pose
            dq_ik = self.end_effector_task(q,target)

            # Secondary Task - Center Joints
            dq_center = self.joint_centering_task(q)
            #print(dq_center)
            ## STUDENT CODE STARTS HERE

            # Task Prioritization
            dq = np.zeros(7) # TODO: implement me!
            #find the projection matrix to the null-space of the Jacobian
            J = calcJacobian(q)

            N = np.transpose(null_space(J))
            dq_center_proj = (N@dq_center)*N[0]
            #print(J@dq_ik)
            #print(J@(dq_ik + dq_center_proj))
            dq = dq_ik + np.linalg.norm(dq_ik)*dq_center_proj
            #print(dq)



            # Termination Conditions
            #upper_limit_violation = dq>IK.upper
           # lower_limit_violation = dq<IK.lower
           # for i in range(7):
           #   if upper_limit_violation[i]:
             #     dq[i] = IK.upper[i]

            #  if lower_limit_violation[i]:
             #     dq[i] = IK.lower[i]
            #print(dq)
            #print(all(np.abs(dq) < self.min_step_size))
            print("norm(dq):", np.linalg.norm(dq))
            if len(rollout) == self.max_steps or (all(np.abs(dq) < self.min_step_size) and self.is_valid_solution(q,target) == True) :
                #check termination conditions
                break # exit the while loop if conditions are met!

            ## END STUDENT CODE

            q =  q + dq

        success = self.is_valid_solution(q,target)
        # print("success:", success)
        # print("q: ", q)
        return q, success, rollout

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    ik = IK()

    # matches figure in the handout
    #seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    # seed = np.array([0.0,0,0,-pi/4,0,pi/2,pi/4])
    seed = np.array([-0.85566999, -1.00246097,  1.86687916, -1.49717099,  0.96796238,  1.77792418, 1.72581234])

    target = np.array([
            [1.0,0.0,0.0,0.1],
            [0.0,-1.0,0.0,0.75],
            [0.0,0.0,-1.0,0.22],
            [0.0,0.0,0.0,1.0]])

    # target = np.array([
    #    [0,-1,0,0.3],
    #    [-1,0,0,0],
    #    [0,0,-1,.5],
    #    [0,0,0, 1],
    # ])

    # target = np.array([
    #     [1.0,0.0,0.0,0.554],
    #     [0.0,-1.0,0.0,0.0],
    #     [0.0,0.0,-1.0,0.522],
    #     [0.0,0.0,0.0, 1.0]
    # ])

    #target = np.array([
    #    [0.965926,-0.258819,0.0,0.554],
    #    [-0.258819,-0.965926,0.0,0.0],
    #    [0.0,0.0,-1.0,0.522],
    #    [0.0,0.0,0.0, 1.0]
    #])

    q, success, rollout = ik.inverse(target, seed)

    for i, q in enumerate(rollout):
        joints, pose = ik.fk.forward(q)
        d, ang = IK.distance_and_angle(target,pose)
        #print('iteration:',i,' q =',q, ' d={d:3.4f}  ang={ang:3.3f}'.format(d=d,ang=ang))

    print("Success: ",success)
    print("Solution: ",q)
    print("Iterations:", len(rollout))
