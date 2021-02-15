# lightsfm

A lightweight implementation of the Social Force Model for Social Local Navigation. 
It is based on the model proposed by Helbing and Molnar [1] and extended for social groups by Moussaid et Al. [2]:

- **[1]** Helbing, Dirk & Molnar, Peter. (1998). *Social Force Model for Pedestrian Dynamics*. Physical Review E. 51. 10.1103/PhysRevE.51.4282. 
- **[2]** Moussaïd, Mehdi & Perozo, Niriaska & Garnier, Simon & Helbing, Dirk & Theraulaz, Guy. (2010). *The Walking Behaviour of Pedestrian Social Groups and Its Impact on Crowd Dynamics*. PloS one. 5. e10047. 10.1371/journal.pone.0010047. 

The model consists on the definition of different attractive and repulsive forces that describe the local navigation behavior of pedestrians.

<img src="https://render.githubusercontent.com/render/math?math=F_{total} = F_{goal} %2B \sum F_{obs} %2B \sum F_{soc} %2B F_{group}">

***Note: vectors are indicated with capital letters**

## Social Forces
### 1. Attractive force to reach the goal (DesiredForce F<sub>goal</sub>)

<img src="https://render.githubusercontent.com/render/math?math=F_{goal} = \omega_{g}+\frac{1}{\sigma_{g}}+(v_{desired}E_{desired} - V_{actual})">

With:

- <img src="https://render.githubusercontent.com/render/math?math=v_{desired}+"> Desired velocity value.
- <img src="https://render.githubusercontent.com/render/math?math=E_{desired}+"> Desired direction vector with:

	<img src="https://render.githubusercontent.com/render/math?math=E_{desired} = \frac{R_{goal} - R_{current}}{(\left \| R_{goal} - R_{current} \right \|}+">
	- <img src="https://render.githubusercontent.com/render/math?math=R_{goal}+"> the desired goal position.
	- <img src="https://render.githubusercontent.com/render/math?math=R_{current}+"> the current agent position.
- <img src="https://render.githubusercontent.com/render/math?math=V_{actual}+"> Actual velocity vector.

And parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{g}+"> Strength Factor of the desire to reach the goal (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\sigma_{g}+"> Relaxation time (*default: 0.5 seg*).


### 2. Repulsive force of obstacles (ObstacleForce F<sub>obs</sub>)

We use a monotonic decreasing potential of the force (an exponential in our case) based on the distance between the agent and the obstacles.

<img src="https://render.githubusercontent.com/render/math?math=F_{obs} = - \omega_{o}+e^{(-R_{po} / \sigma_{o})} U_{R_{po}}">

With:

- <img src="https://render.githubusercontent.com/render/math?math=U_{R_{po}}=\frac{R_{po}} {\left \| R_{po} \right \|}+">Unit vector in the direction from the pedestrian *p* to the obstacle *o*.
- <img src="https://render.githubusercontent.com/render/math?math=\R_{po} = R_{p} - R_{o}+">  Position of the obstacle *o* relative to the pedestrian *p*.

And parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{o}+"> Strength Factor of the desire to walk away the obstacles (*default: 10.0*).
- <img src="https://render.githubusercontent.com/render/math?math=\sigma_{o}+"> Exponential parameter (*default: 0.2*).



### 3. Respulsive force of other pedestrians (SocialForce F<sub>soc</sub>)

Other pedestrian will prokove a repulsive effect on the agent based on the distance, velocity and direction of their movements. 
In [1], the direction of movement of the pedestrian is used to compute the respulsive potential <img src="https://render.githubusercontent.com/render/math?math=W_{pi}+"> 

<img src="https://render.githubusercontent.com/render/math?math=F_{soc} = - \omega_{s} R_{pi} W_{pi} [b(R_{pi})]  +"> 
with <img src="https://render.githubusercontent.com/render/math?math=R_{pi} = (R_{p} - R_{i}) +"> 

And where the monotonic decreasing function of *b* has the form of an ellipse directed into the direction of motion 
<img src="https://render.githubusercontent.com/render/math?math=2b = \sqrt{({\left \| R_{pi} \right \|} %2B \left \| R_{pi} - v_{i} \Delta t E_{i} \right \| )^2 - (v_{i} \Delta t)^2 }  +">

However, in this implementation we compute two forces that are based on the interaction direction of the pedestrians.
<img src="https://render.githubusercontent.com/render/math?math=F_{soc} = \omega_{s} (F_{Ivel} %2B F_{Iangle})  +">

<img src="https://render.githubusercontent.com/render/math?math=F_{Ivel} = w_{vel} * I_{dpi}+">
<img src="https://render.githubusercontent.com/render/math?math=F_{angle} = w_{ang} * I_{dpi}+">(¿¿¿normal vector to Idpi instead of ldpi vector???)

in which the interaction direction between the agent *p* and the pedestrian *i*, is the unit vector <img src="https://render.githubusercontent.com/render/math?math=I_{dpi} =  \frac{I_{vpi}}{\left \| I_{vpi} \right \|}+"> 

With the interaction vector computed as:

<img src="https://render.githubusercontent.com/render/math?math=I_{vpi} = \lambda_{s}(V_{p} - V_{i}) %2B \frac{R_{i}-R_{p}}{\left \| R_{i}-R_{p} \right \|}   +">

And the monotonic descreasing functions in the form of exponentials:

<img src="https://render.githubusercontent.com/render/math?math=w_{vel}  = - e^{-\left \| R_{i}-R_{p} \right \| / ((\gamma_{s} \left \| I_{vpi} \right \| ) - n\' \gamma_{s} \left \| I_{vpi} \right \| \theta)^2}+">
<img src="https://render.githubusercontent.com/render/math?math=w_{ang}  = - e^{-\left \| R_{i}-R_{p} \right \| / ((\gamma_{s} \left \| I_{vpi} \right \| ) - n \gamma_{s} \left \| I_{vpi} \right \| \theta)^2}+">

Note: the only differences are the parameteres *n'* and *n*

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{s}+"> Strength Factor of the desire to walk away the other pedestrians (*default: 2.1*).
- <img src="https://render.githubusercontent.com/render/math?math=\gamma_{s}+"> (*default: 0.35*).
- <img src="https://render.githubusercontent.com/render/math?math=\lambda_{s}+"> (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math=n_{s}+"> (*default: 2.0*).
- <img src="https://render.githubusercontent.com/render/math?math={n_{s}}\' "> (*default: 3.0*).


### 4. Force of interaction groups (GroupForce F<sub>group</sub>)

This force is a combination of another subforces that describes the formation of the social group.

<img src="https://render.githubusercontent.com/render/math?math=F_{group} = F_{ggaze} %2B F_{gcoh} %2B \sum F_{grep}">


#### 4.1. Force of vision field of the group (GroupGazeForce F<sub>ggaze</sub>)

It is based on the rotation angle of the pedestrian's head (gazing direction) so that the group's mass center is included in the vision field (~90º).
The greater the head rotation the less comfortable is the turning for walking, and therefore the pedestrian adjusts its position to reduce the head rotation.
In our implementation we can not detect the gaze direction, so we check if the group's mass center is beyond the vision field of the pedestrian according to his/her movement direction. Therefore, if the angle to communicate with the group (look to the mass center) is greater than the vision field we apply the following force:

<img src="https://render.githubusercontent.com/render/math?math=F_{ggaze} = \omega_{gz}+\alpha+V_{desired}">

With:

- <img src="https://render.githubusercontent.com/render/math?math=\alpha+"> Value based on the desired direction and the distante to the mass center.

	<img src="https://render.githubusercontent.com/render/math?math=\alpha = V_{desired}\cdot R_{pg} / (\left \| V_{desired} \right \|)^2+">

Where <img src="https://render.githubusercontent.com/render/math?math=R_{pg} +"> is the position of the group's mass center *g* relative to the pedestrian *p*.

- <img src="https://render.githubusercontent.com/render/math?math=\V_{desired}+"> Desired direction vector.

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{gz}+"> Strength Factor of the vision of the group (*default: 3.0*).


#### 4.2. Attraction force to the group's center of mass (GroupCoherenceForce F<sub>gcoh</sub>)

The agent feels an attraction to keep close to its interaction group (to keep the coherence of the group).
<img src="https://render.githubusercontent.com/render/math?math=F_{gcoh} = \omega_{gc}+q_{a}+R_{pg}">

With:

- <img src="https://render.githubusercontent.com/render/math?math=\R_{pg}+"> is the position of the group's mass center *g* relative to the pedestrian *p*.
- <img src="https://render.githubusercontent.com/render/math?math=q_{a}+"> is a saturation factor in the range [-1,1]  based on the distance to the group's center and a distance threshold.
	<img src="https://render.githubusercontent.com/render/math?math=q_{a} = tanh(d_{pg}-((n-1)/2))+"> with <img src="https://render.githubusercontent.com/render/math?math=d_{pg}+"> the Euclidean distance between the agent *p* and the group's center *g*; and *n* the number of agents in the group.
	
	Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{gc}+"> Strength Factor of the desire to walk with the group (*default: 2.0*).

#### 4.3. Respulsion force of overlaping group members (GroupRespulsionForce F<sub>grep</sub>)
Repulsion effect so that the group members do not overlap each other.

<img src="https://render.githubusercontent.com/render/math?math=F_{grep} = - \omega_{gr}+q_{r}+R_{pi}">

With:

- <img src="https://render.githubusercontent.com/render/math?math=\R_{pi}+"> is the position of the group member *i* relative to the group member *p*.
- <img src="https://render.githubusercontent.com/render/math?math=q_{r}+"> is an activation parameter that takes value *1* when the Euclidean distance between the agents *p* and *i* is smaller than a threshold value *d*, that is one body diameter plus some safety distance, otherwise *0*.

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\omega_{gr}+"> Strength Factor for not overlapping the group's members (*default: 1.0*).

## Code indications

- **sfm.hpp** contains the methods for computating of all the described forces.
- **angle.hpp and vector2d.hpp** contain different help structures and methods that are employed in the forces computation.
- **map.hpp** contains a structure for representing the obstacles of the static map, and some virtual methods to check obstacles.
- **rosmap.hpp** is a class that implements the virtual methods of map.hpp and uses ROS to obtain the static navigation map. It also implement a kd-tree to perform a nearest-neighbor search on the obstacle map.
- **cmd_vel.hpp** make use of the forces in order to compute a velocity command to be sent to the robot. A aproximation based on Dynamic Window Approach is employed.
- **astar.hpp** is an auxiliary class that contains the implementation of a Astar path planning algorithm.

