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

#include "RP_move_to_floor.h"

#include <string>
#include <vector>

/* The implementation of RP_move_to_floor.h */

/* constructor */
RP_move_to_floor::RP_move_to_floor(ros::NodeHandle& nh) :
  nh_(nh),
  Action("move_to_floor", 3.0)
{
  robot_id_ = "sonny";
  init_timer_	=	nh_.createTimer(ros::Duration(5),&RP_move_to_floor::timeoutCB,this,true);
  scan_sub_ = nh_.subscribe("/scan", 1, &RP_move_to_floor::scanCallback, this);
  init_timer_.stop();
}

void RP_move_to_floor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  scan_ = *scan;
}

void RP_move_to_floor::activateCode()
{

  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("f"))
      target_floor_ = last_msg_.parameters[i].value;
  }
  state_ = INIT;
}

void RP_move_to_floor::deActivateCode()
{

}

void RP_move_to_floor::timeoutCB(const ros::TimerEvent&)
{
  state_ = CHECKING_DOOR;
}


void RP_move_to_floor::step()
{

  switch(state_){
    case INIT:
      ROS_INFO("[move_to_floor] INIT state");
      init_timer_.start();
      break;
    case CHECKING_DOOR:
    {
      ROS_INFO("[move_to_floor] CHECKING_DOOR state");
      for (auto range : scan_.ranges)
      {
        if (range >= 3.0)
        {
          state_ = ASK_FLOOR;
          graph_.add_edge(robot_id_, "ask: elevator_current_floor.ask", robot_id_);
        }
      }

      break;
    }
    case ASK_FLOOR:
    {
      ROS_INFO("[move_to_floor] ASK_FLOOR state");
      auto interest_edges = graph_.get_string_edges_from_node_by_data(robot_id_, "response: [[:alnum:]_]*");
      if (!interest_edges.empty())
      {
        std::string response_floor_num, response_raw = interest_edges[0].get();
        graph_.remove_edge(interest_edges[0]);
        std::string delimiter = "response: ";
        response_floor_num = response_raw.erase(0, response_raw.find(delimiter) + delimiter.length());
        if (response_floor_num == target_floor_)
          state_ = END;
        else
          state_ = CHECKING_DOOR;
      }
      break;
    }
    case END:
    {
      setSuccess();
      break;
    }
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_move_to_floor_to");
  ros::NodeHandle nh("~");
  RP_move_to_floor rpmb(nh);

  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
