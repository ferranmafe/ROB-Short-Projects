# Short Projects of Robotics subject (GEI - FIB)
Complete deliverables for the Robotics subject (GEI - FIB), made in the Q1 2018/2019.

* **Authors:** Daniel Martínez Bordes, Ferran Martínez Felipe

## Description

### Short Project 1 (P1)
Simulation of the movement of a 6R arm made using MATLAB. The idea behind this project is the understanding of
the usage of coordinates transforms to define coordinates in each of the frames defined using the world frame as base (0, 0, 0) and
the usage of inverse kinematics.

Al the code related to the short project (the one made by the students) is stored in the folder _ShortProject1_Ferran_Martinez_Daniel_Martinez.m_.

### Short Project 2 (P2)
Robot localization of a NEATO in the world by using odometry. The idea behind this project is be able to know the position of a robot in a room from a
reference frame, and show the path made for the robot using a web server. In addition, the robot has to be able to park between two boxes without touching them,
using the NEATO laser as a LIDAR.

All the code related to the short project (the one made by the students) is stored in the folder _practica2.py_.

### Short Project 3 (P3)
Set of tasks done by a NEATO robot related with navigation and localization using linear equations.

#### Basic Tasks (basic_features folder)
##### Obstacle Avoidance
This task is based in using NEATO laser in order to avoid obstacles. The robot moves forward
until the moment it found an obstacle in a cone in front of it.
In this moment, the robot evades the obstacle by changing it direction as much as possible and
continue moving forward (but not in the original direction).

##### Wall Following
This task is based in using NEATO laser in order to follow a wall. The robot moves forward until it
found a wall in front of it. At this moment, the robot turns right or left and starts following the wall
(always at the same distance). If the wall direction changes (a corner) the robot algorithm follow the wall
(by doing the corner).

##### Meet An Object
This task is based in using NEATO laser in order to go to an object. The robot moves forward until
the moment that detect an object in front of him. At this point the robot starts moving towards the object
until the moment that he reach it.

#### Advanced Tasks (advaced_features_folder)
##### Obstacle Avoidance with Line Following
The idea behind this task is do the same as in Obstacle Avoidance, but in this case keeping all the time the position and
direction of the robot in the space by using odometry. After avoid the obstacle, the robot continues moving in the same direction
as before avoiding the obstacle (basically the robot is following a line as much as possible).

##### Escape the Maze
Code without changes from the task Wall Following (no improvement is needed).

##### Prey/Predator
Combination of the tasks Obstacle Avoidance and Meet An Object. If the robot acts as prey the behaviour is the same as Obstacle Avoidance (but evading from
every direction instead of evading only the objects in front of it). If the robot acts as predator the behaviour is the same as Meet an Object.

##### Race
Code without changes from the task Wall Following (no improvement is needed). The robot goes around a wall in order to do X laps before the other NEATO robots.
