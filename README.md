# omnibase-movement

Setup: 

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
