# Single-Goal Navigation
This package includes a script to govern the waypoint-based navigation of a robot utilizing move_base in a simulated environment. Using a custom ROS topic, a single waypoint may be set as the robot's goal. After navigating to the goal the robot will return to the center of the environment.

# Multi-Goal Navigation
This package includes a script to manage a list of waypoints for a robot utilizing move_base to navigate to. Similar to the single-goal navigation, the multi-goal navigation package will send goals to the robot for it to navigate to and then have it navigate to the simulation's center. But unlike the single-goal navigation package, the multi-goal package will store multiple goals and then execute them all upon receiving a command to. If the navigation command is cancelled, the goal list will be cleared.

## Installing the Packages
Assuming the simulation you are using works with [move_base](http://wiki.ros.org/move_base), there are no additional dependencies for this package.

Download and compile the packages into your catkin workspace using:
```
cd ~/catkin_ws/src
git clone https://github.com/MatthewZecca/ResearchWork2021.git
cd ..
catkin_make -j1
```

## Implementing the Packages
To use the single-goal navigation package in an existing simulation using move_base, add this line to the bottom of your launch file:
```
<node pkg="single_goal_navigation" type="single_goal_navigation"     name="single_goal_navigation"     output="screen"/>
```

To use the multi-goal navigation package in an existing simulation using move_base, add this line instead:
```
<node pkg="multi_goal_navigation" type="multi_goal_navigation"     name="multi_goal_navigation"     output="screen"/>
```

## Using the Packages
To use the single-goal navigation package, add the 2D Nav Goal Tool to the toolbar in the RViz window that the simulation opens. Then right-click it, and under Tool Properties change the 2D Nav Goal topic to ```cb_goal```.

![RvizCropped](https://user-images.githubusercontent.com/87440798/127158090-dcb79164-e79b-40b2-871d-54c33dce0808.png)

Now whenever you use the 2D Nav Goal tool and click a point on the map, the robot will use this navigation goal script to plan its movement.


For the multi-goal navigation package, follow the steps above. Additionally, after selecting goal points on the map using the 2D Nav Goal tool, paste the following into the terminal to start navigation:
```rostopic pub /navigate std_msgs/Bool true```.
If you would like to cancel the list of points set as goals, paste this instead:
```rostopic pub /navigate std_msgs/Bool false```.
