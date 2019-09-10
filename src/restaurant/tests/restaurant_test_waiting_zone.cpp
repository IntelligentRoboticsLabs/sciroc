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

class CheckWaitingZoneExecutor: public bica_planning::Executor, public bica::Component
{
public:
  CheckWaitingZoneExecutor()
  {
    init_knowledge();
    executed_ = false;
  }

  void init_knowledge()
  {
    add_instance("robot", "sonny");
    add_instance("person", "new_customer");
    add_instance("zone", "waiting_zone");
    add_predicate("robot_at sonny wp_waiting_zone");
    add_predicate("person_at new_customer wp_waiting_zone");
    add_predicate("wp_in_zone wp_waiting_zone waiting_zone");
    add_predicate("waypoint_at wp_waiting_zone main_room");


    graph_.add_node("sonny", "robot");
    graph_.add_node("wp_waiting_zone", "waypoint");  // node is redundantelly added by graph-kms sync issue
    graph_.add_node("waiting_zone", "zone");  // node is redundantelly added by graph-kms sync issue
    graph_.add_node("main_room", "room");

    graph_.set_tf_identity("main_room", "map");
    graph_.set_tf_identity("base_footprint", "sonny");

    graph_.add_tf_edge("main_room", "sonny");

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    tf2::Transform main2zone(q, tf2::Vector3(0.0, 0.0, 0.0));
    graph_.add_edge("main_room", main2zone, "waiting_zone", true);
  }

  void step()
  {
    if (!executed_)
    {
      sleep(11);
      ROS_INFO("Adding goal and planning");

      add_goal("person_detected sonny new_customer waiting_zone");
      call_planner();
      executed_ = true;
    }
    else
      ROS_INFO("Finished executing CheckWaitingZoneExecutor");
  }

private:
  ros::NodeHandle nh_;

  bica_graph::GraphClient graph_;

  bool executed_;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "Restaurant");
  ros::NodeHandle n;

  ros::Rate loop_rate(1);
  CheckWaitingZoneExecutor exec;
  exec.setRoot();
  exec.setActive(true);

  while (exec.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
