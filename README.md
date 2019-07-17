# SciRoc
Maps, configurations and benchmarks for SciRoc Competition.

# Restaurant - Gazebo - XTION - QR
To test this benchmark in Gazebo you must connect the Xtion camera to your laptop.

## Dependencies
  - **Internals:**
     - bica: https://github.com/IntelligentRoboticsLabs/BICA
     - ROSPLan: https://gitlab.com/Intelligent-Robotics/ROSPlan.git
     - gb_dialog: https://github.com/IntelligentRoboticsLabs/gb_dialog
     - dialogflow_ros: https://github.com/jginesclavero/dialogflow_ros/tree/master/dialogflow_ros
     - topological_navigation: https://github.com/IntelligentRoboticsLabs/topological_navigation
     - person_navigation: https://github.com/IntelligentRoboticsLabs/person_navigation

## Launch
Remember: Load the branch ```simulator``` of gb_robots. 
```
roslaunch gb_robots sim_restaurant.launch
roslaunch openni2_launch openni2.launch
roslaunch sciroc restaurant_sim.launch

```
The QR codes are in https://github.com/IntelligentRoboticsLabs/gb_perception
