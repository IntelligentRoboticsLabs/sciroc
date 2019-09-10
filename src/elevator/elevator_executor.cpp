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

ElevatorExecutor::ElevatorExecutor(): current_goal_(), nh_()
{
  init_knowledge();
}

void ElevatorExecutor::init_knowledge()
{
  robot_id_ = "leia";
  add_instance("robot", robot_id_);
  add_predicate("robot_at " + robot_id_ + " wp_start");
  add_predicate("robot_at_room " + robot_id_ + " main_room");

  graph_.set_tf_identity("base_footprint", robot_id_);
  graph_.add_node(robot_id_, "robot");
  graph_.add_node("elevator", "elevator");
  graph_.add_node("0", "floor");
  graph_.add_edge("elevator", "current_floor", "0");
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

void ElevatorExecutor::Init_code_once()
{
  //graph_.add_edge(robot_id_, "ask: bar_start.action", robot_id_);
}

void ElevatorExecutor::getShopList_code_once()
{
  //getShopList() Request to the DH
  // get the goal floor from the list

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
  graph_.add_edge(robot_id_, "say: Getting close to the elevator", robot_id_);
}
void ElevatorExecutor::approachElevator_code_iterative()
{
  setNewGoal("robot_at " + robot_id_ + " wp_waiting_zone");
}

bool ElevatorExecutor::getShopList_2_approachElevator()
{
  // if shopTarget_ is ready
  graph_.add_edge(robot_id_, "say: I have to go to the floor X. I'm ready", robot_id_);
  return true;
}

bool ElevatorExecutor::approachElevator_2_findProxemicPos()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}
