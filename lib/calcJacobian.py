import numpy as np
#from lib.calculateFK import FK

def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q: 0 x 7 configuration vector (of joint angles) [q0,q1,q2,q3,q4,q5,q6]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
      
    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE

    ## Forward Kinematics information
    
    d1  =  0.141
    d2  =  0.192
    d3  =  0.195
    d4  =  0.121
    d5  =  0.082
    d6  =  0.125
    d7  =  0.083
    d8  =  0.259
    d9  =  0.088
    d10 =  0.051
    d11 =  0.159
    y_offset = 0.015
    offset_angle_3 = -np.pi/2
    offset_angle_5 =  np.pi/2
    offset_angle_6 =  np.pi/4
    
    T0 = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,d1], [0.0,0.0,0.0,1.0]])
    
    T1 = np.array([[np.cos(q[0]), 0.0, -np.sin(q[0]), 0.0], [np.sin(q[0]),0.0, np.cos(q[0]), 0.0], [0.0, -1.0, 0.0, d2], [0.0, 0.0, 0.0, 1.0]])
    
    T2 = np.array([[np.cos(q[1]), 0.0, np.sin(q[1]), 0.0], [np.sin(q[1]),0.0, -np.cos(q[1]), 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    
    T3 = np.array([[np.cos(q[2]), 0.0, np.sin(q[2]), (d5)*np.cos(q[2])], [np.sin(q[2]),0.0, -np.cos(q[2]), (d5)*np.sin(q[2])], [0.0, 1.0, 0.0, d3+d4], [0.0, 0.0, 0.0, 1.0]])
    
    T4 = np.array([[np.sin(q[3]-offset_angle_3), 0.0, np.cos(q[3]-offset_angle_3), (d6)*np.cos(q[3]-offset_angle_3)-(d7)*np.sin(q[3]-offset_angle_3)], [-np.cos(q[3]-offset_angle_3),0.0, np.sin(q[3]-offset_angle_3), (d6)*np.sin(q[3]-offset_angle_3)+(d7)*np.cos(q[3]-offset_angle_3)], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    
    T5 = np.array([[np.cos(q[4]), 0.0, np.sin(q[4]), -(y_offset)*np.sin(q[4])], [np.sin(q[4]),0.0, -np.cos(q[4]), (y_offset)*np.cos(q[4])], [0.0, 1.0, 0.0, d8], [0.0, 0.0, 0.0, 1.0]])
    
    T6 = np.array([[-np.sin(q[5]-offset_angle_5), 0.0, np.cos(q[5]-offset_angle_5), (d10)*np.cos(q[5]-offset_angle_5)-(d9)*np.sin(q[5]-offset_angle_5)], [np.cos(q[5]-offset_angle_5),0.0, np.sin(q[5]-offset_angle_5), (d10)*np.sin(q[5]-offset_angle_5)+(d9)*np.cos(q[5]-offset_angle_5)], [0.0, 1.0, 0.0, y_offset], [0.0, 0.0, 0.0, 1.0]])
    
    Te = np.array([[np.cos(q[6]-offset_angle_6), -np.sin(q[6]-offset_angle_6), 0.0, 0.0], [np.sin(q[6]-offset_angle_6), np.cos(q[6]-offset_angle_6), 0.0, 0.0], [0.0, 0.0, 1.0, d11], [0.0, 0.0, 0.0, 1.0]])


    ## Positions of frames' origin w.r.t. world frame
    
    tJ0 = T0
    tJ1 = T0@T1
    tJ2 = T0@T1@T2
    tJ3 = T0@T1@T2@T3      
    tJ4 = T0@T1@T2@T3@T4
    tJ5 = T0@T1@T2@T3@T4@T5
    tJ6 = T0@T1@T2@T3@T4@T5@T6
    tJe = T0@T1@T2@T3@T4@T5@T6@Te
     
    o0 = np.array([0,0,0])
    o1 = np.array(tJ0[0:3,3])
    o2 = np.array(tJ1[0:3,3])
    o3 = np.array(tJ2[0:3,3])
    o4 = np.array(tJ3[0:3,3])
    o5 = np.array(tJ4[0:3,3])
    o6 = np.array(tJ5[0:3,3])
    o7 = np.array(tJ6[0:3,3])
    oe = np.array(tJe[0:3,3])
    
    ## Coordinates of z_i w.r.t. world frame
    
    z1 = np.array(tJ0[0:3,2])
    z2 = np.array(tJ1[0:3,2])
    z3 = np.array(tJ2[0:3,2])
    z4 = np.array(tJ3[0:3,2])
    z5 = np.array(tJ4[0:3,2])
    z6 = np.array(tJ5[0:3,2])
    z7 = np.array(tJ6[0:3,2])
    
    ## Populate Jacobian
    
    J[:3,0] = np.cross(z1,oe-o1)
    J[:3,1] = np.cross(z2,oe-o2)
    J[:3,2] = np.cross(z3,oe-o3)
    J[:3,3] = np.cross(z4,oe-o4)
    J[:3,4] = np.cross(z5,oe-o5)
    J[:3,5] = np.cross(z6,oe-o6)
    J[:3,6] = np.cross(z7,oe-o7)
    
    J[3:,0] = z1
    J[3:,1] = z2
    J[3:,2] = z3
    J[3:,3] = z4
    J[3:,4] = z5
    J[3:,5] = z6
    J[3:,6] = z7
    
    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))
