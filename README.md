# lightsfm

A lightweight implementation of the Social Force Model for Social Local Navigation. 
It is based on the initial model proposed by Helbing and Molnar [1] and extended for social groups by Moussaid et Al. [2][3]:

- **[1]** Helbing, Dirk & Molnar, Peter. (1998). *Social Force Model for Pedestrian Dynamics*. Physical Review E. 51. 10.1103/PhysRevE.51.4282. 
- **[2]** Moussaid M, Helbing D, Garnier S, Johansson A, Combe M, et al. (2009) *Experimental study of the behavioural mechanisms underlying self-organization in human crowds*. Proceedings of the Royal Society B: Biological Sciences 276: 2755–2762.
- **[3]** Moussaïd, Mehdi & Perozo, Niriaska & Garnier, Simon & Helbing, Dirk & Theraulaz, Guy. (2010). *The Walking Behaviour of Pedestrian Social Groups and Its Impact on Crowd Dynamics*. PloS one. 5. e10047. 10.1371/journal.pone.0010047. 

## Acknowledgement

This work has been financed by the European Regional Development Fund (FEDER) and by the Ministry of Economy, Knowledge, Business and University, of the Government of Andalucía , within the framework of the FEDER Andalucía 2014-2020 operational program. Specific objective 1.2.3. "Promotion and generation of frontier knowledge and knowledge oriented to the challenges of society, development of emerging technologies" within the framework of the reference research project UPO-1264631.
FEDER co-financing percentage 80%
<img src="https://github.com/robotics-upo/lightsfm/blob/master/media/logos_repo_github.jpg" width="600">

## Social Forces

The model consists on the definition of different attractive and repulsive forces that describe the local navigation behavior of pedestrians.

$F_{total} = F_{goal} + \sum F_{obs} + \sum F_{soc} + F_{group}$

***Note: vectors are indicated with capital letters**

### 1. Attractive force to reach the goal (DesiredForce F<sub>goal</sub>)

$F_{goal} = \omega_{g}+\frac{1}{\sigma_{g}}+(v_{desired}E_{desired} - V_{actual})$

With:

- $v_{desired}+$ Desired velocity value.
- $E_{desired}+$ Desired direction vector with:

	$E_{desired} = \frac{R_{goal} - R_{current}}{(\left \| R_{goal} - R_{current} \right \|}+$

	- $R_{goal}+$ the desired goal position.
	- $R_{current}+$ the current agent position.

- $V_{actual}+$ Actual velocity vector.

And parameters:

- $\omega_{g}+$ Strength Factor of the desire to reach the goal (*default: 2.0*).
- $\sigma_{g}+$ Relaxation time (*default: 0.5 seg*).


### 2. Repulsive force of obstacles (ObstacleForce F<sub>obs</sub>)

We use a monotonic decreasing potential of the force (an exponential in our case) based on the distance between the agent and the obstacles.

$F_{obs} = - \omega_{o}+e^{(-R_{po} / \sigma_{o})} U_{R_{po}}$

With:

- $U_{R_{po}}=\frac{R_{po}} {\left \| R_{po} \right \|}+$ Unit vector in the direction from the pedestrian *p* to the obstacle *o*.
- $R_{po} = R_{p} - R_{o}+$  Position of the obstacle *o* relative to the pedestrian *p*.

And parameters:

- $\omega_{o}+$ Strength Factor of the desire to walk away the obstacles (*default: 10.0*).
- $\sigma_{o}+$ Exponential parameter (*default: 0.2*).



### 3. Respulsive force of other pedestrians (SocialForce F<sub>soc</sub>)

Other pedestrians will prokove a repulsive effect on the agent.
In the initial model [1], a potential with the form of an ellipse directed in the motion direction of the pedestrian is proposed. However, in the empirical study carried out in [2], the authors specify a more advanced interaction function based on two components, $f_{v}$ and $f_{\theta}$, describing the *deceleration* along the interaction direction $I_{pi}$ and directional changes along the normal vector to the interaction direction oriented to the left, $N_{pi}$.

$F_{soc} = \omega_{s} (f_{v} I_{pi} + f_{\theta} N_{pi})$


in which the interaction direction between the agent *p* and the pedestrian *i*, is the unit vector $I_{pi} =  \frac{I_{vpi}}{\left \| I_{vpi} \right \|}$ 

And the interaction vector computed as a composition of the direction of relative motion and the direction in which the interaction pedestrian *i* is located:

$I_{vpi} = \lambda_{s}(V_{p} - V_{i}) + \frac{R_{i}-R_{p}}{\left \| R_{i}-R_{p} \right \|}$


If $d_{pi}$ denotes the distance between two pedestrians *p* and *i* and $\theta$ the angle between the interaction direction $I_{pi}$ and the vector pointing from agent *p* to pedestrian *i*, we have:

$f_{v}  = - e^{(-d/B - (n_{s}\'B\theta)^2)}$

This represents an exponential decay of the deceleration with distance *d*.

$f_{\theta}  = - k e^{(-d/B - (n_{s}B\theta)^2)}$

With: 
- $k  = \frac{\theta}{\| \theta \|}$ the sign of the angle $\theta$ (=[0, 1, -1]). It takes into account the discontinuity in the angular motion, reflecting the binary decision to evade the other pedestrian either to the left or to the right.
- $B  = \gamma_{s} \| I_{vpi} \|$

Parameters:

- $\omega_{s}$ Strength Factor of the desire to walk away the other pedestrians (*default: 2.1*).
- $\lambda_{s}$ reflects the relative importance of the two directions (*default: 2.0*).
- $\gamma_{s}$ increases in the interaction direction by large relative speeds, while the repulsion towards the sides is reduced (*default: 0.35*).
- $n_{s}$ (*default: 2.0*).
- ${n_{s}}\' $, n' > n, which corresponds to a larger angular interaction range. (*default: 3.0*). 


### 4. Force of interaction groups (GroupForce F<sub>group</sub>)

This force is a combination of another subforces that describes the formation of the social group, as described in [3].

$F_{group} = F_{ggaze} + F_{gcoh} + \sum F_{grep}$


#### 4.1. Force of vision field of the group (GroupGazeForce F<sub>ggaze</sub>)

It is based on the rotation angle of the pedestrian's head (gazing direction) so that the group's mass center is included in the vision field (~90º).
The greater the head rotation the less comfortable is the turning for walking, and therefore the pedestrian adjusts its position to reduce the head rotation.
In our implementation we can not detect the gaze direction, so we check if the group's mass center is beyond the vision field of the pedestrian according to his/her movement direction. Therefore, if the angle to communicate with the group (look to the mass center) is greater than the vision field we apply the following force:

$F_{ggaze} = \omega_{gz}+\alpha+V_{desired}$

With:

- $\alpha$ Value based on the desired direction and the distante to the mass center.

	$\alpha = V_{desired}\cdot R_{pg} / (\left \| V_{desired} \right \|)^2$

Where $R_{pg}$ is the position of the group's mass center *g* relative to the pedestrian *p*.

- $V_{desired}$ Desired direction vector.

Parameters:

- $\omega_{gz}$ Strength Factor of the vision of the group (*default: 3.0*).


#### 4.2. Attraction force to the group's center of mass (GroupCoherenceForce F<sub>gcoh</sub>)

The agent feels an attraction to keep close to its interaction group (to keep the coherence of the group).
$F_{gcoh} = \omega_{gc}+q_{a}+R_{pg}$

With:

- $R_{pg}$ is the position of the group's mass center *g* relative to the pedestrian *p*.
- $q_{a}$ is a saturation factor in the range [-1,1]  based on the distance to the group's center and a distance threshold.
	$q_{a} = tanh(d_{pg}-((n-1)/2))$ with $d_{pg}$ the Euclidean distance between the agent *p* and the group's center *g*; and *n* the number of agents in the group.
	
	Parameters:

- $\omega_{gc}$ Strength Factor of the desire to walk with the group (*default: 2.0*).

#### 4.3. Respulsion force of overlaping group members (GroupRespulsionForce F<sub>grep</sub>)
Repulsion effect so that the group members do not overlap each other.

$F_{grep} = - \omega_{gr}+q_{r}+R_{pi}$

With:

- $R_{pi}$ is the position of the group member *i* relative to the group member *p*.
- $q_{r}$ is an activation parameter that takes value *1* when the Euclidean distance between the agents *p* and *i* is smaller than a threshold value *d*, that is one body diameter plus some safety distance, otherwise *0*.

Parameters:

- $\omega_{gr}$ Strength Factor for not overlapping the group's members (*default: 1.0*).

## Code indications

This is header-only library that does not depend on ROS. All the agents positions, velocities, obstacles, etc must be provided according to the same selected coordinate frame. 

- **sfm.hpp** contains the methods for computating of all the described forces.
- **angle.hpp and vector2d.hpp** contain different help structures and methods that are employed in the forces computation.
- **map.hpp** contains an optional structure for representing the obstacles of the static map, and some virtual methods to check obstacles.
<!--- **rosmap.hpp** is a class that implements the virtual methods of map.hpp and uses ROS to obtain the static navigation map. It also implement a kd-tree to perform a nearest-neighbor search on the obstacle map.-->
<!--- **cmd_vel.hpp** make use of the forces in order to compute a velocity command to be sent to the robot. A aproximation based on Dynamic Window Approach is employed. -->
<!--- **astar.hpp** is an auxiliary class that contains the implementation of a Astar path planning algorithm.-->


## How to use it

First, you need to compile and install the library:

`make`

`sudo make install`

The library will be installed in the directory */usr/local/include/lightsfm*

To use the library in you project, you need two things mainly:


- First, you need to use the structure Agent (sfm.hpp) to represent your pedestrians.
- Secondly, to define you agents' obstacles. Two options: by implementing the virtual methods of the map.hpp header (to pass them through Map* map). Or by filling the obstacles vectors of the agents.
- Finally, with your defined obstacles and agents, you can call different methods:

  `std::vector<Agent>& computeForces(std::vector<Agent>& agents, Map* map) const;` To compute the social forces for all the agents.

  `void computeForces(Agent& me, std::vector<Agent>& agents, Map* map);` To compute the social forces for the indicated agent.

  `std::vector<Agent>& updatePosition(std::vector<Agent>& agents, double dt) const;` To update the state of all the agents after a time step indicated by `dt`.

  `void updatePosition(Agent& me, double dt) const;` To update the state of the indicated agent after a time step indicated by `dt`.

An example of the use of the library can be seen in:

- The walking pedestrian plugin for Gazebo: https://github.com/robotics-upo/gazebo_sfm_plugin

