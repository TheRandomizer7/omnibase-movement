# omnibase-movement

Overview of how the program works:
1) final_hackathon_realistic.launch - When launched, the user is prompted to enter the goal coordinate and the path of the robot will be quickly planned due to the speed of RRT. If the controller detects a sudden change in position, it will stop the robot by changing it's velocity to 0, and the controller will stop following it's old path, until a new path is planned. When the velocity of the robot is 0, the planner detects that, and plans and publishes a new path. The controller detects that the path that it was supposed to follow, has been changed and thus, it resumes following the new path. The controller also stops the robot when it has reached the final node of the path. The planner also detects that the final node has been reached and that the robot has stopped. After that, the planner prompts the user to enter a new goal coordinate.
3) final_hackathon.launch - When launched, the user is prompted to enter the goal coordinate and the path of the robot takes a long time to plan because the planner uses RRT*, because of RRT* the path that the robot follows is very smooth which also causes the robot to reach it's goal faster. Also, the number of points sampled in RRT* is directly proportional to the area contained by the region in which points are sampled. This region can be controlled by the variable ```bounds_of_plane```. After the goal is reached, the controller as described above, stops the movement of the robot until a new path is planned. But since the planner is not designed to plan a new path, the robot never starts moving again. And, the user is not prompted to enter a new goal coordinate. The program has ended.

Code structure: 
1) obstacle_detector.py - publishes an array with the x-coordinates and y-coordinates of the centres of the cylindrical obstacles. These obstacles are fixed, so it just publishes the same array every single time. A little bit redundant since obstacles are not moving, but just to have a more usable code structure. (publishing to topic obstacle_coords)
2) path_planner.py - plans a path from the start point to the goal point using RRT* path planning algorithm by sampling 5000 points. It plans the path only once and keeps on publishing the same path. (subscribing to topics, obstacle_coords and odom. publishing to topic, target_path)
3) path_planner_RRT.py - plans a path from the position of the robot to the goal using RRT algorithm. It checks whether the robot is at rest, once every 2 seconds and if the robot is at rest, it plans a path. (subscribing to topics, obstacle_coords, odom and cmd_vel. publishing to topic, target_path)
4) controller.py - it controls the robot based on the path published by the path_planner. If it detects a sudden change in position or if the robot reaches the goal. It stops the robot completely and waits for a new path to be planned. (subscribing to topics, target_path and odom. publishing to topic, cmd_vel)
5) final_hackathon_realistic.launch - runs the nodes, obstacle_detector.py, path_planner_RRT.py, controller.py
6) final_hackathon.launch - runs the nodes, obstacle_detector.py, path_planner.py, controller.py

[IMPORTANT]: 

You will be prompted to specify the goal coordinates every time the launch files are run. After you enter the goal coordinates, the program performs certain checks to ensure that there will be no errors during the running of the program. If any one of the conditions is not fullfilled, you will be prompted to re-enter the goal coordinates. The following are the checks, that the program performs: 
1) The goal should be such, that the robot does not collide with the obstacle (distance from the center of each cylindrical obstacle should exceed 7.0 units)
2) The goal should be in float format, string or non-numeric data is not permitted
3) The goal should be confined to the ```bounds_of_plane``` variable in the code.

Setup and running: 

First make a workspace with a src directory. Clone this repository in the src directory of your workspace and run catkin_make as shown below.
```
cd ~/<name of workspace>/src/
git clone https://github.com/TheRandomizer7/omnibase-movement.git
cd ..
catkin_make
```
Next, you need to give permissions for the nodes,
```
cd ~/<name of workspace>/src/omnibase-movement/movement_nodes/scripts/
chmod u+x obstacle_detector.py
chmod u+x path_planner.py
chmod u+x path_planner_RRT.py
chmod u+x controller.py
```
and permission to the launch files,
```
cd ~/<name of workspace>/src/omnibase-movement/omnibase/omnibase_gazebo/launch
chmod u+x final_hackathon_realistic.launch
chmod u+x final_hackathon.launch
```

There are two launch files:
1) final_hackathon_realistic.launch
2) final_hackathon.launch

To run the final_hackathon_realistic.launch:
```
roslaunch omnibase_gazebo final_hackathon_realistic.launch
```
To run the final_hackathon.launch:
```
roslaunch omnibase_gazebo final_hackathon.launch
```

Bag files:
1) final_hackathon_realistic.bag - bag file of a sample run of final_hackathon_realistic.launch.
2) final_hackathon.bag - bag file of a sample run of final_hackathon.launch.

Note: 
To change the bounds of the plane where points are sampled, number of points sampled, you need to change the variables in the planners (path_planner.py and path_planner_RRT.py), ```bounds_of_plane```, ```points_to_sample``` respectively

[IMPORTANT]:

1) Changing the variable ```points_to_sample``` in the program path_planner_RRT.py may break the program, because the path planner relies solely on the speed of RRT, if ```points_to_sample``` is increased such that it decreases the speed at which the planner runs, the program will not work as intended. So, keep the ```points_to_sample``` at 0 in the file path_planner_RRT.py.

2) You may change ```points_to_sample``` in path_planner.py.
