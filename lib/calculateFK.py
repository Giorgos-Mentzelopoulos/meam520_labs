import numpy as np
from math import pi

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        self.d1  =  0.141
        self.d2  =  0.192
        self.d3  =  0.195
        self.d4  =  0.121
        self.d5  =  0.082
        self.d6  =  0.125
        self.d7  =  0.083
        self.d8  =  0.259
        self.d9  =  0.088
        self.d10 =  0.051
        self.d11 =  0.159
        self.y_offset = 0.015
        self.offset_angle_3 = -np.pi/2
        self.offset_angle_5 =  np.pi/2
        self.offset_angle_6 =  np.pi/4






    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your code starts here

        # Transformation matrices

        T0 = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,self.d1], [0.0,0.0,0.0,1.0]])

        T1 = np.array([[np.cos(q[0]), 0.0, -np.sin(q[0]), 0.0], [np.sin(q[0]),0.0, np.cos(q[0]), 0.0], [0.0, -1.0, 0.0, self.d2], [0.0, 0.0, 0.0, 1.0]])

        T2 = np.array([[np.cos(q[1]), 0.0, np.sin(q[1]), 0.0], [np.sin(q[1]),0.0, -np.cos(q[1]), 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

        T3 = np.array([[np.cos(q[2]), 0.0, np.sin(q[2]), (self.d5)*np.cos(q[2])], [np.sin(q[2]),0.0, -np.cos(q[2]), (self.d5)*np.sin(q[2])], [0.0, 1.0, 0.0, self.d3+self.d4], [0.0, 0.0, 0.0, 1.0]])

        T4 = np.array([[np.sin(q[3]-self.offset_angle_3), 0.0, np.cos(q[3]-self.offset_angle_3), (self.d6)*np.cos(q[3]-self.offset_angle_3)-(self.d7)*np.sin(q[3]-self.offset_angle_3)], [-np.cos(q[3]-self.offset_angle_3),0.0, np.sin(q[3]-self.offset_angle_3), (self.d6)*np.sin(q[3]-self.offset_angle_3)+(self.d7)*np.cos(q[3]-self.offset_angle_3)], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

        T5 = np.array([[np.cos(q[4]), 0.0, np.sin(q[4]), -(self.y_offset)*np.sin(q[4])], [np.sin(q[4]),0.0, -np.cos(q[4]), (self.y_offset)*np.cos(q[4])], [0.0, 1.0, 0.0, self.d8], [0.0, 0.0, 0.0, 1.0]])

        T6 = np.array([[-np.sin(q[5]-self.offset_angle_5), 0.0, np.cos(q[5]-self.offset_angle_5), (self.d10)*np.cos(q[5]-self.offset_angle_5)-(self.d9)*np.sin(q[5]-self.offset_angle_5)], [np.cos(q[5]-self.offset_angle_5),0.0, np.sin(q[5]-self.offset_angle_5), (self.d10)*np.sin(q[5]-self.offset_angle_5)+(self.d9)*np.cos(q[5]-self.offset_angle_5)], [0.0, 1.0, 0.0, self.y_offset], [0.0, 0.0, 0.0, 1.0]])

        Te = np.array([[np.cos(q[6]-self.offset_angle_6), -np.sin(q[6]-self.offset_angle_6), 0.0, 0.0], [np.sin(q[6]-self.offset_angle_6), np.cos(q[6]-self.offset_angle_6), 0.0, 0.0], [0.0, 0.0, 1.0, self.d11], [0.0, 0.0, 0.0, 1.0]])


        # Joint positions initialization

        jointPositions = np.zeros((7,3))
        jointPositions_augmented = np.zeros((7,4))
        T0e = np.identity(4)

        jointPositions_augmented[::,3] = 1.0

        # Joint 3 is the only joint who is not at the origin of the intermediate coordinate system fixed on the link. It is however trnaslated d3 on the z axis

        jointPositions_augmented[2,2] = self.d3

        jointPositions[0] = np.dot(T0,jointPositions_augmented[0])[:3]
        jointPositions[1] = np.dot(T0@T1,jointPositions_augmented[1])[:3]
        jointPositions[2] = np.dot(T0@T1@T2,jointPositions_augmented[2])[:3]
        jointPositions[3] =   np.dot(T0@T1@T2@T3,jointPositions_augmented[3])[:3]
        jointPositions[4] = np.dot(T0@T1@T2@T3@T4,jointPositions_augmented[4])[:3]
        jointPositions[5] = np.dot(T0@T1@T2@T3@T4@T5,jointPositions_augmented[5])[:3]
        jointPositions[6] = np.dot(T0@T1@T2@T3@T4@T5@T6,jointPositions_augmented[6])[:3]



        T0e = T0@T1@T2@T3@T4@T5@T6@Te

        # Your code ends here

        #return jointPositions, T0e,T0,T1,T2,T3,T4,T5,T6,Te
        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution

if __name__ == "__main__":

    fk = FK()

#####Important for the testing procedure below#####
##### I don't take into consideration the joint limits

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    # arm fully extended upwards
    #q = np.array([0.0, 0.0,0.0,0.0,0.0,pi/2,0.0])
    # arm fully extended to the right
    #q = np.array([ 0, pi/2, 0, 0, 0, pi, pi/4 ])
    # arm fully extended to the left
    #q = np.array([ 0, -pi/2, 0, 0, 0, pi, pi/4 ])
    # arm directed at y direction
    #q = np.array([ pi/2, 0, 0, -pi/2,  0, pi/2, pi/4 ])
    # arm partially folded
    #q =np.array([ pi/2, pi/2, pi, -pi,   0, pi, pi/4 ])
    # arm zic-zac pattern
    #q =np.array([ pi/2, pi/2, pi, -pi/2,  0, pi, pi/4 ])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
