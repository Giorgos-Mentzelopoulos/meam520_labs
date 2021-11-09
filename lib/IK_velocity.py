import numpy as np
from lib.calcJacobian import calcJacobian

def IK_velocity(q_in, v_in, omega_in):
    """
    :param q: 0 x 7 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 0 x 7 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error. If v
         and omega have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE

    Jacobian = calcJacobian(q_in)
    
    xi = np.array([[v_in[0]], [v_in[1]], [v_in[2]], [omega_in[0]], [omega_in[1]], [omega_in[2]]])

    augmented_Jacobian = np.hstack((Jacobian, xi))
    #print(augmented_Jacobian)

    mask = np.isnan(augmented_Jacobian).any(axis = 1)
    #print(mask)
    augmented_Jacobian = augmented_Jacobian[~mask]
    #print(augmented_Jacobian)

    Jacobian_no_nan = augmented_Jacobian[:,0:-1]

    xi_no_nan = augmented_Jacobian[:,-1]

    #print(Jacobian == Jacobian_no_nan)
    #print(augmented_Jacobian)
    #print(Jacobian_no_nan)
    #print(xi)
    #print(xi_no_nan)

    dq, residuals, rank, s = np.linalg.lstsq(Jacobian_no_nan, xi_no_nan, rcond=None)

    return dq

#Testing
#pi = np.pi
#q_in = [0,0,0,-pi/4,0,pi/2,pi/4]
#v_in = [0.1, 0.3, 0.2]
#omega_in = [0.1, 0.4, 0.35]
#print(q_in,v_in,omega_in)
#dq = IK_velocity(q_in, v_in, omega_in)

#print(dq)
