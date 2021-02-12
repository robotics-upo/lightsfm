# lightsfm

A lightweight implementation of the Social Force Model for Social Local Navigation. 
It is based on the model proposed by Helbing and Molnar [1] and extended for social groups by Moussaid et Al. [2]:

- **[1]** Helbing, Dirk & Molnar, Peter. (1998). *Social Force Model for Pedestrian Dynamics*. Physical Review E. 51. 10.1103/PhysRevE.51.4282. 
- **[2]** Moussaïd, Mehdi & Perozo, Niriaska & Garnier, Simon & Helbing, Dirk & Theraulaz, Guy. (2010). *The Walking Behaviour of Pedestrian Social Groups and Its Impact on Crowd Dynamics*. PloS one. 5. e10047. 10.1371/journal.pone.0010047. 

The model consists on the definition of different attractive and repulsive forces that describe the local navigation behavior of pedestrians.

<img src="https://render.githubusercontent.com/render/math?math=F_{total} = f_{goal} %2B \sum f_{obs} %2B \sum f_{ped} %2B f_{group}">

## Social Forces
### 1. Attractive force to reach the goal (DesiredForce f<sub>goal</sub>)

<img src="https://render.githubusercontent.com/render/math?math=f_{goal} = \alpha_{g} \frac{1}{\sigma_{g}} (V_{desired} - V_{actual})">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\alpha_{g}+"> Strength Factor of the desire to reach the goal (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\sigma_{g}+"> Relaxation time (*default: 0.5 seg*).
- <img src="https://render.githubusercontent.com/render/math?math=V_{desired}+"> Desired velocity
- <img src="https://render.githubusercontent.com/render/math?math=V_{actual}+"> Actual velocity

### 2. Repulsive force of obstacles (ObstacleForce f<sub>obs</sub>)

We use a monotonic decreasing potential of the force (an exponential in our case) based on the distance between the agent and the obstacles.

<img src="https://render.githubusercontent.com/render/math?math=f_{goal} = \alpha_{o} e^{(-d_{po} / \sigma_{o})}  \left \| d_{po} \right \|">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\alpha_{o}+"> Strength Factor of the desire to walk away the obstacles (*default: 10.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\sigma_{o}+"> Exponential parameter (*default: 0.2*).
- <img src="https://render.githubusercontent.com/render/math?math=\d_{po}+"> Distance between the pedestrian *p* and the obstacle *o*.


### 3. Respulsive force of other pedestrians (SocialForce f<sub>ped</sub>)

Other pedestrian will prokove a repulsive effect on the agent based on the distance, velocity and direction of their movements. Again we use exponential potentials

TODO

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\alpha_{s}+"> Strength Factor of ... (*default: 2.1*).
- <img src="https://render.githubusercontent.com/render/math?math=\gamma_{s}+"> (*default: 0.35*).
- <img src="https://render.githubusercontent.com/render/math?math=\lamdda_{s}+"> (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\n_{s}+"> (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\n'_{s}+"> (*default: 3.0*).


### 4. Force of interaction groups (GroupForce f<sub>group</sub>)

This force is a combination of another subforces that keeps the formation of the social group.

<img src="https://render.githubusercontent.com/render/math?math=f_{group} = f_{ggaze} %2B f_{gcoh} %2B f_{grep}">


#### 4.1. Force of vision field of the group (GroupGazeForce f<sub>ggaze</sub>)

#### 4.2. Attraction force to the group's center of mass (GroupCoherenceForce f<sub>gcoh</sub>)

#### 4.3. Respulsion force of overlaping group members (GroupRespulsionForce f<sub>grep</sub>)


## Code indications

- **sfm.hpp** contains the methods for computating of all the forces.
- **angle.hpp and vector2d.hpp** contain different help structures and methods that are employed in the forces computation.
- **map.hpp** contains a structure for representing the obstacles of the static map, and some virtual methods to check obstacles.
- **rosmap.hpp** is a class that implements the virtual methods of map.hpp and uses ROS to obtain the static navigation map. It also implement a kd-tree to perform a nearest-neighbor search on the obstacle map.
- **cmd_vel.hpp** make use of the forces in order to compute a velocity command to be sent to the robot. A aproximation based on Dynamic Window Approach is employed.
- **astar.hpp** is an auxiliary class that contains the implementation of a Astar path planning algorithm.

 

