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

  int num_tables_to_check;
  nh_.param<int>("/restaurant_executor_node/num_tables_to_check", num_tables_to_check, 6);

  for (int i = 0; i < num_tables_to_check; i++)
  {
    std::string table = "mesa_" + std::to_string(i + 1);
    graph_.add_node(table, "table");  // node is redundantelly added by graph-kms sync issue
    graph_.add_edge(table, "needs_check", table);
  }
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
  graph_.add_edge(robot_id, "say: I will check the table status.", robot_id);
}

void RestaurantExecutor::checkTableStatus_code_iterative()
{
  std::vector<bica_graph::StringEdge> tables_to_check = graph_.get_string_edges_by_data("needs_check");
  if (!tables_to_check.empty())
    setNewGoal("table_checked " + tables_to_check[0].get_source());
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
  setNewGoal("new_customer_attended new_customer " + ready_table_);
}

bool RestaurantExecutor::Init_2_checkTableStatus()
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

bool RestaurantExecutor::checkTableStatus_2_idle()
{
  if (graph_.get_string_edges_by_data("needs_check").empty())
    setNewGoal("robot_at " + robot_id + " wp_entry");

  return graph_.get_string_edges_by_data("robot_at")[0].get_target() == "wp_entry";
}

bool RestaurantExecutor::idle_2_getOrder()
{
  std::vector<bica_graph::StringEdge> interest_edges =
    graph_.get_string_edges_by_data("status: needs_serving");

  if (!interest_edges.empty() && !order_delivered)
  {
    needs_serving_table_ = interest_edges[0].get_source();
    current_goal_ = "order_ready " + interest_edges[0].get_source();
    return true;
  }
  else
    return false;
}

bool RestaurantExecutor::getOrder_2_setOrder()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}

bool RestaurantExecutor::setOrder_2_checkOrder()
{
  if (!(search_predicates_regex(current_goal_)).empty())
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
  return (!(search_predicates_regex("order_needs_fix " + robot_id)).empty());
}

bool RestaurantExecutor::checkOrder_2_deliverOrder()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}

bool RestaurantExecutor::fixOrder_2_checkOrder()
{
  if (!(search_predicates_regex(current_goal_)).empty())
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
  if (!(search_predicates_regex(current_goal_)).empty())
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
  std::vector<bica_graph::StringEdge> edges_list = graph_.get_string_edges_by_data("status: ready");

  if (!edges_list.empty() && order_delivered && !new_customer)
  {
    new_customer = true;
    ready_table_ = edges_list[0].get_source();
    return true;
  }
  else
    return false;
}

bool RestaurantExecutor::grettingNewCustomer_2_idle()
{
  return (!(search_predicates_regex(current_goal_)).empty());
}
