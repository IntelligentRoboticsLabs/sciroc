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

/* Author: Jonatan Gines jgines@gsyc.urjc.es */

/* Mantainer: Jonatan Gines jgines@gsyc.urjc.es */

#include "restaurant_executor.h"

RestaurantExecutor::RestaurantExecutor(): current_goal_(), nh_()
{
  init_knowledge();
  order_ready_asked = false;
  order_delivered = false;
  new_customer = false;
  total_tables_ = 6;
}

void RestaurantExecutor::init_knowledge() {
  robot_id = "leia";

  add_instance("robot", robot_id);
  add_instance("person", "barman");
  add_instance("person", "new_customer");
  add_predicate("robot_at " + robot_id + " wp_entry");
  add_predicate("robot_at_room " + robot_id + " main_room");

  add_predicate("person_at new_customer wp_entry");
  add_predicate("person_at barman wp_barman");
  add_predicate("person_at_room new_customer main_room");

  add_predicate("wp_bar_location wp_barman");
  add_predicate("wp_entry_location wp_entry");
  add_predicate("barman barman");

  add_instance("table", "mesa_1");
  add_instance("table", "mesa_2");
  add_instance("table", "mesa_3");
  add_instance("table", "mesa_4");
  add_instance("table", "mesa_5");
  add_instance("table", "mesa_6");

  add_predicate("is_wp_near_table wp_mesa_1 mesa_1");
  add_predicate("is_wp_near_table wp_mesa_2 mesa_2");
  add_predicate("is_wp_near_table wp_mesa_3 mesa_3");
  add_predicate("is_wp_near_table wp_mesa_4 mesa_4");
  add_predicate("is_wp_near_table wp_mesa_5 mesa_5");
  add_predicate("is_wp_near_table wp_mesa_6 mesa_6");

  add_predicate("is_table_at mesa_1 main_room");
  add_predicate("is_table_at mesa_2 main_room");
  add_predicate("is_table_at mesa_3 main_room");
  add_predicate("is_table_at mesa_4 main_room");
  add_predicate("is_table_at mesa_5 main_room");
  add_predicate("is_table_at mesa_6 main_room");

  graph_.add_node(robot_id, "robot");
  graph_.add_node("barman", "person");

  nh_.param<int>("/restaurant_executor_node/num_tables_to_check", num_tables_to_check_, 6);
}

bool RestaurantExecutor::update() {
  return ok();
}
std::vector<std::string> RestaurantExecutor::splitSpaces(std::string raw_str)
{
  std::vector<std::string> output;
  std::istringstream iss(raw_str);
  std::string s;
  while (getline(iss, s, ' '))
  {
    output.push_back(s);
  }
  return output;
}

void RestaurantExecutor::setNewGoal(std::string goal)
{
  remove_current_goal();
  current_goal_ = goal;
  add_goal(current_goal_);
  call_planner();
}

void RestaurantExecutor::Init_code_once()
{
  graph_.add_edge(robot_id, "ask: bar_start.action", robot_id);
}

void RestaurantExecutor::Init_code_iterative()
{
  
}

void RestaurantExecutor::checkTableStatus_code_once()
{
  table_list_ = get_instances("table");

  graph_.add_edge(robot_id, "say: I will check the table status.", robot_id);
}

void RestaurantExecutor::checkTableStatus_code_iterative()
{
  int cont = 0;
  ROS_INFO("[checkTableStatus_code_iterative]");
  for (std::vector<std::string>::iterator it = table_list_.begin(); it != table_list_.end(); ++it)
  {
    setNewGoal("table_checked " + *it);
    cont++;
    if (cont == num_tables_to_check_)
      break;
  }
}

void RestaurantExecutor::idle_code_iterative()
{
  if (order_delivered)
    setNewGoal("robot_at " + robot_id + " wp_entry");
}

void RestaurantExecutor::getOrder_code_once()
{
  graph_.add_edge(robot_id, "say: I will take the order.", robot_id);
}

void RestaurantExecutor::getOrder_code_iterative()
{
  setNewGoal(current_goal_);
}

void RestaurantExecutor::setOrder_code_iterative()
{
  setNewGoal("order_to_barman " + robot_id);
}

void RestaurantExecutor::checkOrder_code_iterative()
{
  setNewGoal("order_checked " + robot_id);
}

void RestaurantExecutor::fixOrder_code_iterative()
{
  remove_predicate("order_needs_fix " + robot_id);
  setNewGoal("order_fixed " + robot_id);
}

void RestaurantExecutor::deliverOrder_code_iterative()
{
  setNewGoal("order_delivered " + needs_serving_table_);
}

void RestaurantExecutor::grettingNewCustomer_code_once()
{
  graph_.add_edge(robot_id, "say: Hi! Welcome to the restaurant, I will guide you to a table. Follow me", robot_id);
}

void RestaurantExecutor::grettingNewCustomer_code_iterative()
{
  setNewGoal("person_guided new_customer " + ready_table_);
}

bool RestaurantExecutor::Init_2_checkTableStatus()
{
  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    if (it->get_source() == robot_id && it->get_target() == robot_id && it->get().find("response: ") != std::string::npos)
    {
      graph_.remove_edge(*it);
      return true;
    }
  }
  return false;
}

bool RestaurantExecutor::checkTableStatus_2_idle()
{
  std::regex regex_tables("(table_checked mesa_[[:alnum:]_]*)");
  std::vector<std::string> tables_checked = search_predicates_regex(regex_tables);
  if (tables_checked.size() == table_list_.size() - (total_tables_ - num_tables_to_check_) && tables_checked.size() != 0)
  {
    setNewGoal("robot_at " + robot_id + " wp_entry");
    return true;
  }
  return false;
}

bool RestaurantExecutor::idle_2_getOrder()
{
  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string source = it->get_source();
    std::string edge = it->get();
    if (std::find(table_list_.begin(), table_list_.end(), source) != table_list_.end() &&
        edge.find("needs_serving") != std::string::npos &&
        !order_delivered)
    {
      needs_serving_table_ = source;
      current_goal_ = "order_ready " + needs_serving_table_;
      return true;
    }
  }
  return false;
}

bool RestaurantExecutor::getOrder_2_setOrder()
{
  return (!(search_predicates_regex(std::regex(current_goal_))).empty());
}

bool RestaurantExecutor::setOrder_2_checkOrder()
{
  if (!(search_predicates_regex(std::regex(current_goal_))).empty())
  {
    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      if (it->get_source() == "barman" && it->get_target() == robot_id && it->get().find("response:") != std::string::npos)
      {
        graph_.remove_edge(*it);
        order_ready_asked = false;
        return true;
      }
      else if (!order_ready_asked)
      {
        order_ready_asked = true;
        graph_.add_edge(robot_id, "ask: orderReady.ask", "barman");
      }
    }
  }
  return false;
}

bool RestaurantExecutor::checkOrder_2_fixOrder()
{
  return (!(search_predicates_regex(std::regex("order_needs_fix " + robot_id))).empty());
}

bool RestaurantExecutor::checkOrder_2_deliverOrder()
{
  return (!(search_predicates_regex(std::regex(current_goal_))).empty());
}

bool RestaurantExecutor::fixOrder_2_checkOrder()
{
  if (!(search_predicates_regex(std::regex(current_goal_))).empty())
  {
    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      if (it->get_source() == "barman" && it->get_target() == robot_id && it->get().find("response:") != std::string::npos)
      {
        graph_.remove_edge(*it);
        order_ready_asked = false;
        remove_predicate("order_checked " + robot_id);
        return true;
      }
      else if (!order_ready_asked)
      {
        order_ready_asked = true;
        graph_.add_edge(robot_id, "ask: orderReady.ask", "barman");
      }
    }
  }
  return false;
}

bool RestaurantExecutor::deliverOrder_2_idle()
{
  if (!(search_predicates_regex(std::regex(current_goal_))).empty())
  {
    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      if (it->get_source() == robot_id && it->get_target() == robot_id && it->get().find("response:") != std::string::npos)
      {
        graph_.remove_edge(*it);
        order_ready_asked = false;
        order_delivered = true;
        return true;
      }
      else if (!order_ready_asked)
      {
        order_ready_asked = true;
        graph_.add_edge(robot_id, "ask: confirmOrder.ask", needs_serving_table_);
      }
    }
  }
  return false;
}

bool RestaurantExecutor::idle_2_grettingNewCustomer()
{
  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string source = it->get_source();
    std::string edge = it->get();
    if (std::find(table_list_.begin(), table_list_.end(), source) != table_list_.end() &&
        edge.find("ready") != std::string::npos &&
        order_delivered &&
        !new_customer)
    {
      new_customer = true;
      ready_table_ = source;
      return true;
    }
  }
  return false;
}

bool RestaurantExecutor::grettingNewCustomer_2_idle()
{
  if (!(search_predicates_regex(std::regex(current_goal_))).empty())
  {
    graph_.add_edge(robot_id, "say: Here's your table.", robot_id);
    return true;
  }
  return false;
}
