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

#include <ros/ros.h>

#include <bica_planning/Executor.h>
#include <bica/Component.h>
#include <bica_graph/graph_client.h>
#include "gb_datahub/gb_datahub.h"

class MoveToFloorExecutor: public bica_planning::Executor, public bica::Component
{
public:
  MoveToFloorExecutor(): nh_("")
  {
    init_knowledge();
    executed_ = false;
  }

  void init_knowledge()
  {
    add_instance("robot", "sonny");
    add_instance("zone", "encounter_zone");
    add_instance("floor", "first");
    add_instance("floor", "second");
    add_instance("floor", "third");
    add_instance("floor", "fourth");
    add_instance("floor", "fifth");
    add_predicate("robot_at sonny wp_elevator");
    add_predicate("robot_at_room sonny main_room");
    graph_.add_node("sonny", "robot");

    std::vector<shop> shops = gb_datahub::getShopsList();
    for (auto shop : shops)
    {
      if (shop.goal = true)
        target_floor_ = car2ord(shop.floor);
    }
    graph_.add_node(target_floor_, "floor");
    graph_.add_edge("sonny", "target_floor", target_floor_);

    //auto interest_edges = graph_.get_string_edges_from_node_by_data("sonny", "target_floor");
    //if (!interest_edges.empty())
    //  target_floor_ = interest_edges[0].get_target();
  }

  std::string car2ord(int target_floor)
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

  void step()
  {
    if (!executed_)
    {
      ROS_INFO("Adding goal and planning");
      add_goal("target_floor_reached sonny " + target_floor_);
      call_planner();
      executed_ = true;
    }
    else
      ROS_INFO("Finished executing MoveToFloorExecutor");
  }

private:
  ros::NodeHandle nh_;
  bica_graph::GraphClient graph_;
  std::string target_floor_;
  bool executed_;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveToFloorExecutor");
  ros::NodeHandle n;

  ros::Rate loop_rate(1);
  MoveToFloorExecutor exec;
  exec.setRoot();
  exec.setActive(true);

  while (exec.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}