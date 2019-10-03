# Custom planners using ROS Navigation Stack
This repository contains a custom Global and Local path planner.
The algorithms implemented are
- Astar for global planner
- DWA for local planner

Both planners have been developed following the specificied interfaces
- [Turotial](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)
- [BaseGlobalPlanner interface](http://docs.ros.org/melodic/api/nav_core/html/classnav__core_1_1BaseGlobalPlanner.html)
- [BaseLocalPlanner interface](http://docs.ros.org/melodic/api/nav_core/html/classnav__core_1_1BaseLocalPlanner.html)
- [MIT RaceCar Team 5 Documentation](https://mit-racecar.github.io/6.141-spring-2016-team-5-documentation/)

The **BaseGlobalPlanner** interface require the following functions:
- initialize
- makePlan

The **BaseLocalPlanner** interface require the following functions:
- initialize
- computeVelocityCommands
- isGoalReached
- setPlan

# Running
A demo can be run using the neo_simulation package\
`roslaunch neo_simulation mpo_500_autonomous_navigation_custom.launch`.
