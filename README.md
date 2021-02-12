# lightsfm

A lightweight implementation of the Social Force Model for Social Local Navigation. 
It is based on the model proposed by Helbing and Molnar [1] and extended for social groups by Moussaid et Al. [2]:

- **[1]** Helbing, Dirk & Molnar, Peter. (1998). *Social Force Model for Pedestrian Dynamics*. Physical Review E. 51. 10.1103/PhysRevE.51.4282. 
- **[2]** Moussaïd, Mehdi & Perozo, Niriaska & Garnier, Simon & Helbing, Dirk & Theraulaz, Guy. (2010). *The Walking Behaviour of Pedestrian Social Groups and Its Impact on Crowd Dynamics*. PloS one. 5. e10047. 10.1371/journal.pone.0010047. 

The model consists on the definition of different attractive and repulsive forces that describe the local navigation behavior of pedestrians.

<img src="https://render.githubusercontent.com/render/math?math=F_{total} = F_{goal} %2B \sum F_{obs} %2B \sum F_{ped} %2B F_{group}">

**Note: vectors are indicated with capital letters*

## Social Forces
### 1. Attractive force to reach the goal (DesiredForce F<sub>goal</sub>)

<img src="https://render.githubusercontent.com/render/math?math=F_{goal} = \omega_{g}+\frac{1}{\sigma_{g}}+(V_{desired} - V_{actual})">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{g}+"> Strength Factor of the desire to reach the goal (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\sigma_{g}+"> Relaxation time (*default: 0.5 seg*).
- <img src="https://render.githubusercontent.com/render/math?math=V_{desired}+"> Desired velocity vector
- <img src="https://render.githubusercontent.com/render/math?math=V_{actual}+"> Actual velocity vector

### 2. Repulsive force of obstacles (ObstacleForce F<sub>obs</sub>)

We use a monotonic decreasing potential of the force (an exponential in our case) based on the distance between the agent and the obstacles.

<img src="https://render.githubusercontent.com/render/math?math=F_{goal} = \omega_{o}+e^{(-R_{po} / \sigma_{o})}  \left \| R_{po} \right \|">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{o}+"> Strength Factor of the desire to walk away the obstacles (*default: 10.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\sigma_{o}+"> Exponential parameter (*default: 0.2*).
- <img src="https://render.githubusercontent.com/render/math?math=\R_{po}+">  Position of the obstacle *o* relative to the pedestrian *p*.


### 3. Respulsive force of other pedestrians (SocialForce F<sub>ped</sub>)

Other pedestrian will prokove a repulsive effect on the agent based on the distance, velocity and direction of their movements. Again we use exponential potentials

TODO

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{s}+"> Strength Factor of ... (*default: 2.1*).
- <img src="https://render.githubusercontent.com/render/math?math=\gamma_{s}+"> (*default: 0.35*).
- <img src="https://render.githubusercontent.com/render/math?math=\lambda_{s}+"> (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=n_{s}+"> (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math={n'}_{s}+"> (*default: 3.0*).


### 4. Force of interaction groups (GroupForce F<sub>group</sub>)

This force is a combination of another subforces that describes the formation of the social group.

<img src="https://render.githubusercontent.com/render/math?math=F_{group} = F_{ggaze} %2B F_{gcoh} %2B \sum F_{grep}">


#### 4.1. Force of vision field of the group (GroupGazeForce F<sub>ggaze</sub>)

It is based on the rotation angle of the pedestrian's head (gazing direction) so that the group's mass center is included in the vision field (~90º).
The greater the head rotation the less comfortable is the turning for walking, and therefore the pedestrian adjusts its position to reduce the head rotation.
In our implementation we can not detect the gaze direction, so we check if the group's mass center is beyond the vision field of the pedestrian according to his/her movement direction. Therefore, if the angle to communicate with the group (look to the mass center) is greater than the vision field we apply the following force:

<img src="https://render.githubusercontent.com/render/math?math=f_{ggaze} = \omega_{gz}+\alpha+V_{desired}">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{gz}+"> Strength Factor of the vision of the group (*default: 3.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\alpha+"> Value based on the desired direction adn the distante to the mass center.
	<img src="https://render.githubusercontent.com/render/math?math=\alpha = V_{desired}.R_{pg} / (\left \| V_{desired} \right \|)^2+">
Where <img src="https://render.githubusercontent.com/render/math?math=R_{pg} +"> is the position of the group's mass center *g* relative to the pedestrian *p*.

- <img src="https://render.githubusercontent.com/render/math?math=\V_{desired}+"> Desired velocity vector.


#### 4.2. Attraction force to the group's center of mass (GroupCoherenceForce F<sub>gcoh</sub>)

The agent feels an attraction to keep close to its interaction group (to keep the coherence of the group).
<img src="https://render.githubusercontent.com/render/math?math=F_{gcoh} = \omega_{gc}+q_{a}+R_{pg}">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{gc}+"> Strength Factor of the desire to walk with the group (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\R_{pg}+"> is the position of the group's mass center *g* relative to the pedestrian *p*.
- <img src="https://render.githubusercontent.com/render/math?math=q_{a}+"> is a saturation factor in the range [-1,1]  based on the distance to the group's center and a distance threshold.
	<img src="https://render.githubusercontent.com/render/math?math=q_{a} = tanh(d_{pg}-((n-1)/2)+"> width <img src="https://render.githubusercontent.com/render/math?math=d_{pg}+"> the Euclidean distance between the agent *p* and the group's center *g*; and *n* the number of agents in the group.

#### 4.3. Respulsion force of overlaping group members (GroupRespulsionForce F<sub>grep</sub>)
Repulsion effect so that the group members do not overlap each other.

<img src="https://render.githubusercontent.com/render/math?math=F_{grep} = \omega_{gr}+q_{r}+R_{pi}">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{gr}+"> Strength Factor for not overlapping the group's members (*default: 1.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\R_{pi}+"> is the position of the group member *i* relative to the group member *p*.
- <img src="https://render.githubusercontent.com/render/math?math=q_{r}+"> is an activation parameter that takes value *1* when the Euclidean distance between the agents *p* and *i* is smaller than a threshold value *d*, that is one body diameter plus some safety distance, otherwise *0*.

## Code indications

- **sfm.hpp** contains the methods for computating of all the described forces.
- **angle.hpp and vector2d.hpp** contain different help structures and methods that are employed in the forces computation.
- **map.hpp** contains a structure for representing the obstacles of the static map, and some virtual methods to check obstacles.
- **rosmap.hpp** is a class that implements the virtual methods of map.hpp and uses ROS to obtain the static navigation map. It also implement a kd-tree to perform a nearest-neighbor search on the obstacle map.
- **cmd_vel.hpp** make use of the forces in order to compute a velocity command to be sent to the robot. A aproximation based on Dynamic Window Approach is employed.
- **astar.hpp** is an auxiliary class that contains the implementation of a Astar path planning algorithm.
