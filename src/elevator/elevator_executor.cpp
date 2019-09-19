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

#include "./elevator_executor.h"
#include <string>
#include <list>
#include <vector>

ElevatorExecutor::ElevatorExecutor():
nh_(),
current_goal_(),
utils_(nh_)
{
  init_knowledge();
  target_floor_ = "";
}

void ElevatorExecutor::init_knowledge()
{
  robot_id_ = "sonny";
  add_instance("robot", robot_id_);
  add_predicate("robot_at " + robot_id_ + " wp_start");
  add_predicate("robot_at_room " + robot_id_ + " main_room");
  add_instance("zone", "encounter_zone");
  add_instance("zone", "waiting_zone");

  graph_.begin_batch();
  graph_.set_tf_identity("base_footprint", robot_id_);
  graph_.add_node(robot_id_, "robot");
  graph_.add_node("elevator", "elevator");
  graph_.add_node("wp_waiting_zone", "waypoint");

  graph_.add_node("encounter_zone", "zone");
  graph_.add_node("waiting_zone", "zone");
  graph_.add_node("main_room", "room");

  graph_.set_tf_identity("main_room", "map");
  graph_.add_tf_edge("main_room", robot_id_);

  add_instance("floor", "first");
  add_instance("floor", "second");
  add_instance("floor", "third");
  add_instance("floor", "fourth");
  add_instance("floor", "fifth");
  add_instance("floor", "fifth");
  add_instance("floor", "sixth");
  add_instance("floor", "seventh");
  add_instance("floor", "eigth");
  add_instance("floor", "ninth");

  graph_.add_node("0", "floor");
  graph_.add_edge("elevator", "current_floor", "0");

  utils_.set_inital_pose(-9.69, -2.41, 1.57);
  tf2::Quaternion q;
  q.setRPY(0, 0, -1.57);
  tf2::Transform main2zone(q, tf2::Vector3(-2.88, -0.64, 0.0));
  graph_.add_edge("main_room", main2zone, "waiting_zone", true);

  graph_.flush();
}

void ElevatorExecutor::setNewGoal(std::string goal)
{
  remove_current_goal();
  current_goal_ = goal;
  add_goal(current_goal_);
  call_planner();
}

bool ElevatorExecutor::update()
{
  return ok();
}

std::string ElevatorExecutor::car2ord(int target_floor)
{
  if (target_floor == 1)
    return "first";
  else if(target_floor == 2)
    return "second";
  else if(target_floor == 3)
    return "third";
  else if(target_floor == 4)
    return "fourth";
  else if(target_floor == 5)
    return "fifth";
  else if(target_floor == 6)
    return "sixth";
  else if(target_floor == 7)
    return "seventh";
  else if(target_floor == 8)
    return "eigth";
  else if(target_floor == 9)
    return "ninth";
}

void ElevatorExecutor::Init_code_once()
{
  graph_.add_edge(robot_id_, "ask: bar_start.action", robot_id_);
}

void ElevatorExecutor::getShopList_code_once()
{
  std::vector<shop> shops = gb_datahub::getShopsList();
  for (auto shop : shops)
  {
    if (shop.goal)
    {
      ROS_INFO("shop.floor [%i]",shop.floor);
      target_floor_ = car2ord(shop.floor);
    }
  }
  if (target_floor_ == "")
    ROS_ERROR("Target_floor doesn't recovery from DH");

  graph_.add_node(target_floor_, "floor");
  graph_.add_edge("sonny", "target_floor", target_floor_);
}

bool ElevatorExecutor::Init_2_getShopList()
{
  std::vector<bica_graph::StringEdge> response_edges =
    graph_.get_string_edges_by_data("response: [[:alnum:]_]*");

  bool exists_response = false;
  for (auto edge : response_edges)
  {
    graph_.remove_edge(edge.get_source(), edge.get(), edge.get_target());
    exists_response = true;
  }

  return exists_response;
}

void ElevatorExecutor::approachElevator_code_once()
{
  graph_.add_edge(robot_id_, "robot_status: Approaching to the elevator", robot_id_);
  //graph_.add_edge(robot_id_, "say: Getting close to the elevator", robot_id_);
}
void ElevatorExecutor::approachElevator_code_iterative()
{
  setNewGoal("robot_at " + robot_id_ + " wp_pre_encounter");
  setNewGoal("social_move_pred " + robot_id_ + " wp_encounter");
  setNewGoal("robot_at " + robot_id_ + " wp_post_encounter");
}

void ElevatorExecutor::findProxemicPos_code_once()
{
  graph_.add_edge(robot_id_, "robot_status: Moving to wait the elevator", robot_id_);
  graph_.add_edge(robot_id_, "say: Moving to wait the elevator", robot_id_);
}

void ElevatorExecutor::findProxemicPos_code_iterative()
{
  setNewGoal("robot_at " + robot_id_ + " wp_waiting_zone");
}

void ElevatorExecutor::robotAtElevator_code_once()
{
  graph_.add_edge(robot_id_, "robot_status: Waiting people enter in the elevator", robot_id_);
  graph_.add_edge(robot_id_, "want_see", "waiting_zone");
  graph_.add_edge(robot_id_, "say: Waiting until people enter in the elevator", robot_id_);
  wait_ = ros::Time::now();
}

void ElevatorExecutor::robotAtElevator_code_iterative()
{
  if (ros::Time::now() > wait_ + ros::Duration(40))
    setNewGoal("robot_at " + robot_id_ + " wp_elevator_door");
}

void ElevatorExecutor::advertiseGoal_code_once()
{
  graph_.add_edge(robot_id_, "robot_status: Entering into the elevator", robot_id_);
  graph_.remove_edge("sonny", "want_see", "waiting_zone");
  graph_.add_edge(robot_id_, "say: Entering into the elevator", robot_id_);
}

void ElevatorExecutor::advertiseGoal_code_iterative()
{
  setNewGoal("robot_at " + robot_id_ + " wp_elevator");
}

void ElevatorExecutor::waitForDoor_code_once()
{

}

void ElevatorExecutor::waitForDoor_code_iterative()
{
  setNewGoal("target_floor_reached " + robot_id_ + " " + target_floor_);
}

void ElevatorExecutor::askForFloor_code_once()
{
  //graph_.add_edge(robot_id_, "say: Asking for current floor", robot_id_);
  graph_.add_edge(robot_id_, "robot_status: Asking for current floor", robot_id_);
}

void ElevatorExecutor::robotAtEnd_code_once()
{
  graph_.add_edge(robot_id_, "robot_status: Exiting the elevator. Navigating to end point", robot_id_);
  graph_.add_edge(robot_id_, "say: Please, let me exit the elevator. Navigating to end point", robot_id_);
}

void ElevatorExecutor::robotAtEnd_code_iterative()
{
  setNewGoal("robot_at " + robot_id_ + " wp_end");
}

bool ElevatorExecutor::getShopList_2_approachElevator()
{
  return true;
}

bool ElevatorExecutor::approachElevator_2_findProxemicPos()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}

bool ElevatorExecutor::findProxemicPos_2_robotAtElevator()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}

bool ElevatorExecutor::robotAtElevator_2_advertiseGoal()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}

bool ElevatorExecutor::advertiseGoal_2_waitForDoor()
{
  //graph_.add_edge(robot_id_, "say: Hi! I must go to the " + target_floor_ + " floor. Could you press the button by me?", robot_id_);
  return (!(search_predicates_regex(current_goal_)).empty());
}

bool ElevatorExecutor::waitForDoor_2_askForFloor()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}

bool ElevatorExecutor::askForFloor_2_waitForDoor()
{
  return false;
}

bool ElevatorExecutor::askForFloor_2_robotAtEnd()
{
  return true;
}
