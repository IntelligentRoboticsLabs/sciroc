/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonatan Gines jonatan.gines@urjc.es */

/* Mantainer: Jonatan Gines jonatan.gines@urjc.es */

#ifndef ELEVATOR_EXECUTOR_H
#define ELEVATOR_EXECUTOR_H

#include <ros/ros.h>
#include "bica_planning/Executor.h"
#include "./elevator_hfsm.h"
#include <bica_graph/graph_client.h>
#include <string>
#include <vector>

class ElevatorExecutor: public bica_planning::Executor, public bica::elevator_hfsm
{
public:
  ElevatorExecutor();

  bool update();
  void init_knowledge();
  void setNewGoal(std::string goal);

  void Init_code_once();
  void getShopList_code_once();
  void approachElevator_code_once();
  void approachElevator_code_iterative();
  void findProxemicPos_code_iterative();
  void findProxemicPos_code_once();
  void robotAtElevator_code_iterative();
  void robotAtElevator_code_once();
  void advertiseGoal_code_iterative();
  void advertiseGoal_code_once();
  void waitForDoor_code_once();
  void waitForDoor_code_iterative();
  void askForFloor_code_once();
  void robotAtEnd_code_iterative();
  void robotAtEnd_code_once();


  bool Init_2_getShopList();
  bool getShopList_2_approachElevator();
  bool approachElevator_2_findProxemicPos();
  bool findProxemicPos_2_robotAtElevator();
  bool robotAtElevator_2_advertiseGoal();
  bool advertiseGoal_2_waitForDoor();
  bool waitForDoor_2_askForFloor();
  bool askForFloor_2_waitForDoor();
  bool askForFloor_2_robotAtEnd();

  /*void Init_code_iterative();
  void askForFloor_code_iterative();
  void getShopList_code_iterative();
  */

private:
  ros::NodeHandle nh_;
  bica_graph::GraphClient graph_;
  std::string current_goal_, robot_id_;
  ros::Subscriber scan_sub_;
  ros::Time wait_;
};

#endif  // ELEVATOR_EXECUTOR_H
