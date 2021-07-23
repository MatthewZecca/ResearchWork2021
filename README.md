# Single-Goal Navigation
This package includes a script to govern the waypoint-based navigation of a Jackal UGV in a simulated environment. Using a custom ROS topic, a single waypoint may be set as the Jackal's goal. After navigating to the goal the Jackal will return to the center of the environment.

# Multi-Goal Navigation
This package includes a script to manage a list of waypoints for a Jackal UGV to navigate to. Similar to the single-goal navigation, the multi-goal navigation package will send goals to the Jackal for it to navigate to and then have it navigate to the simulation's center. But unlike the single-goal navigation package, the multi-goal package will store multiple goals and then execute them all upon receiving a command to. If the navigation command is cancelled, the goal list will be cleared.

## Installing the Packages
Download and compile the packages into your catkin workspace using:
```
cd ~/catkin_ws/src
git clone https://github.com/MatthewZecca/ResearchWork2021.git
cd ..
catkin_make -j1
```

## Implementing the Packages
To use the single-goal navigation package in an existing Jackal simulation, add this line to the bottom of your launch file:
```
<node pkg="single_goal_navigation" type="single_goal_navigation"     name="single_goal_navigation"     output="screen"/>
```

To use the multi-goal navigation package in an existing Jackal simulation, add this line instead:
```
<node pkg="multi_goal_navigation" type="multi_goal_navigation"     name="multi_goal_navigation"     output="screen"/>
```

## Using the Packages
To use the packages...
*screenshot showing rviz*
