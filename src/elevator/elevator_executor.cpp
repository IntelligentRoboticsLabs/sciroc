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

ElevatorExecutor::ElevatorExecutor(): current_goal_(), nh_(), utils_(nh_)
{
  init_knowledge();
}

void ElevatorExecutor::init_knowledge()
{
  robot_id_ = "sonny";
  add_instance("robot", robot_id_);
  add_predicate("robot_at " + robot_id_ + " wp_start");
  add_predicate("robot_at_room " + robot_id_ + " main_room");

  graph_.set_tf_identity("base_footprint", robot_id_);
  graph_.add_node(robot_id_, "robot");
  graph_.add_node("elevator", "elevator");

  add_instance("floor", "first");
  add_instance("floor", "second");
  add_instance("floor", "third");
  add_instance("floor", "fourth");
  add_instance("floor", "fifth");

  graph_.add_node("0", "floor");
  graph_.add_edge("elevator", "current_floor", "0");

  utils_.set_inital_pose(-8.65, -2.73, 1.57);

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
}

void ElevatorExecutor::Init_code_once()
{
  //graph_.add_edge(robot_id_, "ask: bar_start.action", robot_id_);
}

void ElevatorExecutor::getShopList_code_once()
{
  std::vector<shop> shops = gb_datahub::getShopsList();
  for (auto shop : shops)
  {
    if (shop.goal == true)
      target_floor_ = car2ord(shop.floor);
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
  //graph_.add_edge(robot_id_, "say: Getting close to the elevator", robot_id_);
}
void ElevatorExecutor::approachElevator_code_iterative()
{
  setNewGoal("social_move_pred " + robot_id_ + " wp_post_encounter");
}

void ElevatorExecutor::findProxemicPos_code_once()
{
  graph_.add_edge(robot_id_, "say: Moving to wait the elevator", robot_id_);
}

void ElevatorExecutor::findProxemicPos_code_iterative()
{
  setNewGoal("robot_at " + robot_id_ + " wp_waiting_zone");
}

void ElevatorExecutor::robotAtElevator_code_once()
{
  graph_.add_edge(robot_id_, "say: Getting close to the elevator door", robot_id_);
}

void ElevatorExecutor::robotAtElevator_code_iterative()
{
  setNewGoal("robot_at " + robot_id_ + " wp_elevator_door");
}

void ElevatorExecutor::advertiseGoal_code_once()
{
  graph_.add_edge(robot_id_, "say: Entring into the elevator", robot_id_);
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
}

void ElevatorExecutor::robotAtEnd_code_once()
{
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
  if (!(search_predicates_regex(current_goal_)).empty())
  {
    graph_.add_edge(robot_id_, "say: Hi! I must go to the " + target_floor_ + " floor. Could you press the button by me?", robot_id_);
    return true;
  }
  return false;
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
