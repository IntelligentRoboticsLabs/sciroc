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

#include <sciroc/Utils.h>

class ProxemicMoveExecutor: public bica_planning::Executor, public bica::Component
{
public:
  ProxemicMoveExecutor()
  :
   nh_(""),
   utils_(nh_)
  {
    init_knowledge();
    executed_ = false;
  }

  void init_knowledge()
  {
    add_instance("robot", "sonny");
    add_instance("zone", "encounter_zone");

    add_predicate("robot_at sonny wp_start");
    add_predicate("robot_at_room sonny main_room");


    graph_.add_node("sonny", "robot");
    graph_.add_node("main_room", "room");
    graph_.add_node("wp_start", "waypoint");
    graph_.add_node("wp_waiting_zone", "waypoint");
    graph_.add_node("encounter_zone", "zone");


    graph_.set_tf_identity("base_footprint", "sonny");
    graph_.add_node("main_room", "room");  // node is redundantelly added by graph-kms sync issue
    graph_.set_tf_identity("main_room", "map");
    graph_.add_tf_edge("main_room", "sonny");

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);

    tf2::Transform main2zone(q, tf2::Vector3(1.0, 1.5, 0.0));
    graph_.add_edge("main_room", main2zone, "encounter_zone", true);


    //utils_.set_inital_pose(-8.65, -2.73, 1.57);
  }

  void step()
  {
    if (!executed_)
    {
      ROS_INFO("Adding goal and planning");

      add_goal("proxemic_moving sonny wp_waiting_zone wp_elevator");
      call_planner();
      executed_ = true;
    }
    else
      ROS_INFO("Finished executing ProxemicMoveExecutor");
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher init_pose_pub_;
  sciroc::Utils utils_;

  bica_graph::GraphClient graph_;

  bool executed_;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "Elevator");
  ros::NodeHandle n;

  ros::Rate loop_rate(1);
  ProxemicMoveExecutor exec;
  exec.setRoot();
  exec.setActive(true);

  while (exec.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
