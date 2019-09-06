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

#ifndef SRC_RESTAURANT_RESTAURANT_EXECUTOR_H
#define SRC_RESTAURANT_RESTAURANT_EXECUTOR_H

#include <ros/ros.h>
#include "bica_planning/Executor.h"
#include "./restaurant_hfsm.h"
#include <bica_graph/graph_client.h>
#include <darknet_ros_3d_msgs/BoundingBoxes3d.h>
#include <string>
#include <vector>
#include <tf2_ros/transform_listener.h>


#define DISTANCE_TH_   1.8


class RestaurantExecutor: public bica_planning::Executor, public bica::restaurant_hfsm
{
public:
  RestaurantExecutor();

  bool update();
  void init_knowledge();
  void setNewGoal(std::string goal);
  bool person_close();
  void objectsCallback(const darknet_ros_3d_msgs::BoundingBoxes3d::ConstPtr& msg);


  void Init_code_iterative();
  void deliverOrder_code_iterative();
  void fixOrder_code_iterative();
  void grettingNewCustomer_code_iterative();
  void grettingNewCustomer_code_once();
  void idle_code_iterative();
  void setOrder_code_iterative();
  void checkOrder_code_iterative();
  void getOrder_code_iterative();
  void getOrder_code_once();
  void Init_code_once();
  void checkTableStatus_code_iterative();
  void checkTableStatus_code_once();

  bool fixOrder_2_checkOrder();
  bool idle_2_grettingNewCustomer();
  bool checkOrder_2_deliverOrder();
  bool idle_2_getOrder();
  bool checkTableStatus_2_idle();
  bool checkOrder_2_fixOrder();
  bool deliverOrder_2_idle();
  bool getOrder_2_setOrder();
  bool Init_2_checkTableStatus();
  bool setOrder_2_checkOrder();
  bool grettingNewCustomer_2_idle();
private:
  ros::NodeHandle nh_;

  std::string robot_id_, current_goal_, needs_serving_table_, ready_table_;
  std::vector<std::string> splitSpaces(std::string raw_str);
  bica_graph::GraphClient graph_;
  bool order_ready_asked_, order_delivered_, new_customer_;
  ros::Subscriber object_sub_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;
  darknet_ros_3d_msgs::BoundingBoxes3d objects_msg_;
};

#endif  // SRC_RESTAURANT_RESTAURANT_EXECUTOR_H
