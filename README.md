# rrt-global-planner
A [Rapidly Exploring Random Trees (RRT)](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) global path planner plugin for ROS.

## Demo
![](https://github.com/mech0ctopus/rrt-global-planner/raw/main/assets/rrt_tb3_rviz.gif)

## Install Dependancies and Build
```bash
cd ~/catkin_ws/src
git clone https://github.com/mech0ctopus/rrt-global-planner.git
cd .. && rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Usage
Within the [move\_base](https://wiki.ros.org/move_base) node in your launch file, set the ````base_global_planner```` parameter to ```global_planner/RRTGlobalPlanner``` and load the required parameters.
```xml
<param name="base_global_planner" value="global_planner/RRTGlobalPlanner"/>
<rosparam file="$(find rrt-global-planner)/params/rrt_global_planner.yaml" command="load" />
```

After launching the system, when you set a ```move_base/goal``` using RViz's ```2D Nav Goal``` or with an action client, the ```RRTGlobalPlanner``` will be called. The global path will be published as a topic for visualization. Optionally, a visualization of the full RRT constructed for path planning will be published.

A RViz config file is available in [rrt-global-planner/config](https://github.com/mech0ctopus/rrt-global-planner/tree/main/config). Include the following node in your launch file to use it.
```xml
<node pkg="rviz" type="rviz" name="rviz" required="true"
    args="-d $(find rrt-global-planner)/config/rrt_tb3.rviz"/>
```

## Example
An example launch file for using  ```RRTGlobalPlanner``` with the [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) is located in [rrt-global-planner/launch](https://github.com/mech0ctopus/rrt-global-planner/tree/main/launch).

## ROS API
### Published Topics
`~/move_base/RRTGlobalPlanner/plan` ([nav\_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
- The global path constructed by the planner. Used for visualization purposes.

`~/move_base/RRTGlobalPlanner/tree` ([visualization\_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))
- Visualization of full tree built during planning process.

### Subscribed Topics
None.

### Parameters
`~move_base/RRTGlobalPlanner/goal_tol` (`double`, default: 0.05)
- Cartesian goal tolerance to be achieved by the global planner.

`~move_base/RRTGlobalPlanner/K_in` (`int`, default: 4000)
- Maximum number of iterations to attempt to find a plan.

`~move_base/RRTGlobalPlanner/d` (`double`, default: 0.2)
- Distance to extend tree per iteration.

`~move_base/RRTGlobalPlanner/viz_tree` (`bool`, default: false)
- Whether to publish full tree on `~/move_base/RRTGlobalPlanner/tree` topic for visualization purposes after planning success. 

### Services
None.
```