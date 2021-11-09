from lib.calculateFK import FK
from core.interfaces import ArmController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

  
# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
dof=7
resolution = 5
increment = dof*[0.0]
x=[];
y=[];
z=[];
for i in range(0,dof):
 increment[i] = (limits[i]['upper']-limits[i]['lower'])/resolution
 
for i in range(0,resolution+1):
      for j in range(0,resolution+1):
        for k in range(0,resolution+1):
         for l in range(0,resolution+1):
          for m in range(0,resolution+1):
           for n in range(0,resolution+1):
            for o in range(0,resolution+1):
              q=[limits[0]['lower']+increment[0]*i,limits[1]['lower']+increment[1]*j,limits[2]['lower']+increment[2]*k,limits[3]['lower']+increment[3]*l,limits[4]['lower']+increment[4]*m,limits[5]['lower']+increment[5]*n,limits[6]['lower']+increment[6]*o]
             
              end_effector_position = fk.forward(q)[0][6]
              x.append(end_effector_position[0])
              y.append(end_effector_position[1])
              z.append(end_effector_position[2])
            
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html

fig= plt.figure()
ax= fig.add_subplot(111, projection= '3d')

# TODO: update this with real results
ax.scatter(x,y,z)

#Project the workspace on the 3 planes

ax.plot(x, z, 'r', zdir='y', zs=1.5)
ax.plot(y, z, 'g', zdir='x', zs=-1.5)
ax.plot(x, y, 'b', zdir='z', zs=-1.5)

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])

plt.show()
