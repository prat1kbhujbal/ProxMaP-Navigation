# ProxMaP-Navigation
Risk-aware navigation using occupancy map predictions from deep learning (ProxMaP: Proximal Occupancy Map Prediction)

## Overview


## Building the project and Installating dependencies
1. Clone the repository
```bash
git clone 
```
2. Install dependencies
```bash
cd <your_ws>/
rosdep install --from-paths src --ignore-src -r -y
```bash
3. Build the project
```bash
catkin_make
```

## Running the project
1. Launch the simulation
```bash
roslaunch  turtlebot_gazebo turtlebot_world.launch
```
2. Launch the mapping and navigation nodes
```bash
roslaunch turtlebot_navigation mapping.launch
```
3. Launch the prediction update node
```bash
roslaunch proxmap_ros main.launch
```
4. Launch the goal publisher node
```bash
roslauch proxmap_ros goal_pub.launch
```
Update your goal points as per your environment in the file `goal_publisher.py` in `ProxMaP_ROS/proxmap_ros/scripts` folder.

5. Launch the Adaptive Speed node  
```bash
roslaunch proxmap_ros adaptive_speed.launch
```
This will generate a csv file with the results in the folder `results/` i,e time taken for each goal point to be reached and total time taken for all the goal points to be reached.

## Results


