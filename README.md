# SciRoc

[![Build Status](https://travis-ci.com/IntelligentRoboticsLabs/sciroc.svg?branch=master)](https://travis-ci.com/IntelligentRoboticsLabs/sciroc)


Maps, configurations and benchmarks for SciRoc Competition.

## Restaurant
 [Rulebook](https://docs.google.com/document/d/122AS7SgOQe__Aj0V3fhoXb-766LgDYz9gZfcrGY7jMM/edit#heading=h.gjdgxs)

### Dependencies
  - **Internals:**
     - bica: https://github.com/IntelligentRoboticsLabs/BICA
     - ROSPLan: https://gitlab.com/Intelligent-Robotics/ROSPlan.git
     - gb_dialog: https://github.com/IntelligentRoboticsLabs/gb_dialog
     - dialogflow_ros: https://github.com/jginesclavero/dialogflow_ros/tree/master/dialogflow_ros
     - topological_navigation: https://github.com/IntelligentRoboticsLabs/topological_navigation
     - person_navigation: https://github.com/IntelligentRoboticsLabs/person_navigation
   - **Externals:**
     - sound_play: https://wiki.ros.org/sound_play

**NOTE: Send me a mail to get our dialogflow API Key.**

### Gazebo - XTION - QR
To test this benchmark in Gazebo **you must connect the Xtion camera to your laptop.**

#### Launch
Remember: Load the branch ```simulator``` of gb_robots.
```
roslaunch gb_robots sim_restaurant.launch
roslaunch openni2_launch openni2.launch
roslaunch sciroc restaurant_sim.launch
rosrun rviz rviz
```
The QR codes are in https://github.com/IntelligentRoboticsLabs/gb_perception

### Real robot - QR

#### Previous steps
The package gb_robots contains differents branch for differents robots. Create your own branch and upload launchers, nav_params and maps. **It's neccessary create the topological_map of your scenario, saving the waypoints in gb_robots/maps/[TEST]/topological_map.yaml**.  

**Check the topics and config files in the benchmark launcher**, and if all it's correct, continues.

#### Launch
First of all, **launch robot driver, RGBD camera and navigation stack.**
```
roslaunch sciroc restaurant_lab.launch

```
#### Expected Robot Behavior
1. The robot must say that's it's ready and wait for a "Start the task" voice command.
2. The robot will navigates to every table to check it's status.  
2.1. When the robot reach a table, by default, wait for a QR with the table status - **PDDL Action RP_check_table_status.**   
2.2. A new node with this info will be created in the graph. [Fig](https://github.com/IntelligentRoboticsLabs/sciroc/blob/master/doc/1.svg)
3. It will returns to the init wp and then it will go the get a order. The robot will navigates to a **need_serving** table.
4. It will gets the order by voice, for example: "We want beer, coke and water" and this will be represented in the graph. [Fig](https://github.com/IntelligentRoboticsLabs/sciroc/blob/master/doc/2.svg)
5. The robot will goes to the barman. When the robot reaches, it will say the table and the order and wait for "The order it's ready".
6. Then the robot will check the order, by default, reading a QR and say what object is missing. [Fig](https://github.com/IntelligentRoboticsLabs/sciroc/blob/master/doc/3.svg)
7. The robot will wait for you change the order and say again "The order it's ready".
8. The robot check the order again and it will navigates to deliver the order.
9. The robot deliver the order and wait for "Everything it's ok". [Fig](https://github.com/IntelligentRoboticsLabs/sciroc/blob/master/doc/4.svg)
10. It will navigate to the wp init and wait for the new customer, guiding him to a **ready** table.
