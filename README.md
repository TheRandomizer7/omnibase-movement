# omnibase-movement

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

There are two launch files.
1) final_hackathon_realistic.launch - This file runs RRT path planner which takes a lot less time to run but, creates long paths. But, if the robot is disturbed, it stops following the old path until a new path is planned. Meanwhile, the path planner plans a new path for it to follow. So, the robot can be disturbed, rotated, moved from anywhere to anywhere, as long as the goal is reachable, it will find it's path.
2) final_hackathon.launch - This file runs the RRT* path planner which takes a long time to run but, creates a smooth path for the robot to follow. However, the robot is prone to disturbances and will not create a new path or follow the old path if it's disturbed from it's position.

To run the final_hackathon_realistic.launch:
```
roslaunch omnibase_gazebo final_hackathon_realistic.launch
```
To run the final_hackathon.launch:
```
roslaunch omnibase_gazebo final_hackathon.launch
```

Code structure: 
1) obstacle_detector.py - publishes an array with the x-coordinates and y-coordinates of the centres of the cylindrical obstacles. These obstacles are fixed, so it just publishes the same array every single time. A little bit redundant since obstacles are not moving, but just to have a more usable code structure. (publishing to topic obstacle_coords)
2) path_planner.py - plans a path from the start point to the goal point using RRT* path planning algorithm by sampling 5000 points. It plans the path only once and keeps on publishing the same path. (subscribing to topics obstacle_coords and odom. publishing to topic target_path)
3) path_planner_RRT.py - plans a path from the position of the robot to the goal using RRT algorithm. It checks whether the robot is at rest, once every 2 seconds and if the robot is at rest, it plans a path. (subscribing to topics obstacle_coords, odom and cmd_vel. publishing to topic target_path)
4) controller.py - it controls the robot based on the path published by the path_planner. If it detects a sudden change in position or if the robot reaches the goal. It stops the robot completely and waits for a new path to be planned. (subscribing to topics target_path and odom. publishing to topic cmd_vel)
