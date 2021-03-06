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

#include <geometry_msgs/Twist.h>

#include <string>
#include <vector>

/* The implementation of RP_move_to_floor.h */

/* constructor */
RP_move_to_floor::RP_move_to_floor(ros::NodeHandle& nh) :
  nh_(nh),
  Action("move_to_floor", 11.0),
  obj_listener_("base_footprint")
{
  robot_id_ = "sonny";
  //init_timer_	=	nh_.createTimer(ros::Duration(5),&RP_move_to_floor::timeoutCB,this,true);
  scan_sub_ = nh_.subscribe("/scan", 1, &RP_move_to_floor::scanCallback, this);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
  //init_timer_.stop();

  darknet_ros_3d::ObjectConfiguration person_conf;

  person_conf.min_probability = MOVE_TO_FLOOR_MIN_PROBABILITY;
  person_conf.min_x = MOVE_TO_FLOOR_PERSON_MIN_X;
  person_conf.max_x = MOVE_TO_FLOOR_PERSON_MAX_X;
  person_conf.min_y = MOVE_TO_FLOOR_PERSON_MIN_Y;
  person_conf.max_y = MOVE_TO_FLOOR_PERSON_MAX_Y;
  person_conf.min_z = MOVE_TO_FLOOR_PERSON_MIN_Z;
  person_conf.max_z = MOVE_TO_FLOOR_PERSON_MAX_Z;
  person_conf.min_size_x = MOVE_TO_FLOOR_PERSON_MIN_SIZE_X;
  person_conf.min_size_y = MOVE_TO_FLOOR_PERSON_MIN_SIZE_Y;
  person_conf.min_size_z = MOVE_TO_FLOOR_PERSON_MIN_SIZE_Z;
  person_conf.max_size_x = MOVE_TO_FLOOR_PERSON_MAX_SIZE_X;
  person_conf.max_size_y = MOVE_TO_FLOOR_PERSON_MAX_SIZE_Y;
  person_conf.max_size_z = MOVE_TO_FLOOR_PERSON_MAX_SIZE_Z;
  person_conf.dynamic = true;
  person_conf.max_seconds = ros::Duration(10.0);

  obj_listener_.add_class("person", person_conf);
  last_funny_sentence_ = "";
  funny_sentences_ = {
    "I do not trust elevators, there always up to something",
    "I hope it rains, this does not seem autumn",
    "The weather is crazy",
    "When I was young I went down the stairs",
    "Did you hear about the elevator dance? It has no steps",
    "Good friends are like broken elevators. They never let you down",
    "Was not your son studing Computer Science? Please, tell him to repair my laptop"
  };
  funny_sentence_said_ = false;
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

  graph_.add_edge("sonny", "want_see", "sonny");
  obj_listener_.reset();
  obj_listener_.set_working_frame("wp_elevator");
  obj_listener_.set_active();

  state_ts_ = ros::Time::now();
}

void RP_move_to_floor::deActivateCode()
{
  graph_.remove_edge("sonny", "want_see", "sonny");
  obj_listener_.set_inactive();
}


void RP_move_to_floor::stop_robot()
{
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0.0;

  vel_pub_.publish(vel_msg);
}

double
RP_move_to_floor::get_angle_to_person()
{
  if (obj_listener_.get_objects().empty())
  {
    ROS_INFO("\tFace Person NOT FOUND  ");
    return M_PI; // max
  } else
  {
    tf2::Vector3 pos = obj_listener_.get_objects()[0].central_point;
    double person_angle = atan2(pos.y(), pos.x());

    tf2::Stamped<tf2::Transform> r2door = graph_.get_tf("wp_elevator", "sonny");
    tf2::Matrix3x3 m(r2door.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double diff = person_angle - yaw;
    ROS_INFO("Person = %lf\trobot = %lf  =====> %lf", person_angle, yaw, diff);

    while (diff > M_PI) diff = diff - 2.0 * M_PI;
    while (diff < -M_PI) diff = diff + 2.0 * M_PI;
    return diff;
  }
}

void RP_move_to_floor::face_person()
{
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;

  if (obj_listener_.get_objects().empty())
  {
    ROS_INFO("\tFace Person NOT FOUND  ==> %lf", vel_msg.angular.z);
    vel_msg.angular.z = 0.3;
  } else
  {
    double vel = get_angle_to_person();

    vel_msg.angular.z = std::max(std::min(vel, 0.3), -0.3);
    ROS_INFO("\tFace Person yaw = %lf  ==> %lf", vel, vel_msg.angular.z);
  }

  vel_pub_.publish(vel_msg);
}


void RP_move_to_floor::face_door()
{

  tf2::Stamped<tf2::Transform> r2door = graph_.get_tf("sonny", "wp_elevator");

  // wp_elevator
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;


  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;


  tf2::Matrix3x3 m(r2door.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);



  double vel = yaw;
  vel_msg.angular.z = std::max(std::min(vel, 0.3), -0.3);

  ROS_INFO("\tFace door yaw = %lf  ==> %lf", yaw, vel_msg.angular.z);


  vel_pub_.publish(vel_msg);
}


void RP_move_to_floor::step()
{

  switch(state_){
    case INIT:
      ROS_INFO("[move_to_floor] INIT state");
      state_ts_ = ros::Time::now();
      state_ = FACE_PERSON_INFORM;
      break;
    case FACE_PERSON_INFORM:
      ROS_INFO("[move_to_floor] FACE_PERSON_INFORM state");

      face_person();

      if ((ros::Time::now() - state_ts_ ).toSec() >= 30.0)
      {
        ROS_WARN("FACE_PERSON_INFORM timeout");
        stop_robot();
        state_ts_ = ros::Time::now();
        state_ = INFORM_FLOOR;
        graph_.add_edge(robot_id_, "say: Hi! I must go to the " + target_floor_ + " floor. Could you press the button by me?", robot_id_);
      }

      if (!obj_listener_.get_objects().empty())
      {
        double pos_angle = get_angle_to_person();

        if (fabs(pos_angle) < 0.2)
        {
          stop_robot();
          state_ts_ = ros::Time::now();
          state_ = INFORM_FLOOR;
          graph_.add_edge(robot_id_, "say: Hi! I must go to the " + target_floor_ + " floor. Could you press the button by me?", robot_id_);
        }
      }
      break;
    case INFORM_FLOOR:
      ROS_INFO("[move_to_floor] INFORM_FLOOR state");
      if ((ros::Time::now() - state_ts_ ).toSec() > 5.0)  // wait 5 secs for speaking
      {
        state_ts_ = ros::Time::now();
        state_ = FACE_DOOR;
        graph_.add_edge(robot_id_, "say: I'm facing the door", robot_id_);
      }
      break;

    case FACE_DOOR:
      ROS_INFO("[move_to_floor] FACE_DOOR state");

      {
        face_door();
        tf2::Stamped<tf2::Transform> r2door = graph_.get_tf("sonny", "wp_elevator");
        tf2::Matrix3x3 m(r2door.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (fabs(yaw) < 0.2 && (ros::Time::now() - state_ts_ ).toSec() >= 15.0)
        {
          stop_robot();
          state_ts_ = ros::Time::now();
          state_ = CHECK_DOOR;
          funny_sentence_said_ = false;
          graph_.add_edge(robot_id_, "say: Checking if the door is opened", robot_id_);
        }
      }
      break;
    case CHECK_DOOR:
      ROS_INFO("[move_to_floor] CHECK_DOOR state");
    {
      for (const auto& range : scan_.ranges)
      {
        if (range >= 3.0)
        {
          state_ts_ = ros::Time::now();
          state_ = FACE_PERSON_ASK;
          graph_.add_edge(robot_id_, "say: Looking for a person", robot_id_);
        }else{
          if (!funny_sentence_said_){
            //Say funny speech
            funny_sentence_said_ = true;
            std::string funny_sentence = "say: " + generateFunnySentence();
            graph_.add_edge(robot_id_, funny_sentence, robot_id_);
          }
        }
      }
      break;
    }
    case FACE_PERSON_ASK:
      ROS_INFO("[move_to_floor] FACE_PERSON_ASK state");

      face_person();
      if ((ros::Time::now() - state_ts_ ).toSec() >= 30.0)
      {
        stop_robot();
        graph_.add_edge(robot_id_, "ask: elevator_current_floor.ask", robot_id_);
        state_ts_ = ros::Time::now();
        state_ = ASK_FLOOR;
      }

      if (!obj_listener_.get_objects().empty())
      {
        double pos_angle = get_angle_to_person();

        if (fabs(pos_angle) < 0.2)
        {
          stop_robot();
          graph_.add_edge(robot_id_, "ask: elevator_current_floor.ask", robot_id_);
          state_ts_ = ros::Time::now();
          state_ = ASK_FLOOR;
        }
      }
      break;

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

        state_ts_ = ros::Time::now();
        if (response_floor_num == target_floor_)
          state_ = END;
        else
        {
          state_ = FACE_DOOR;
          graph_.add_edge(robot_id_, "say: I'm facing the door", robot_id_);
        }
      }

      if ((ros::Time::now() - state_ts_ ).toSec() >= 45.0)
        state_ = END;

      break;
    }
    case END:
    {
      ROS_INFO("[move_to_floor] END state");
      graph_.remove_edge("sonny", "want_see", "sonny");
      obj_listener_.set_inactive();
      setSuccess();
      break;
    }
  }
}

std::string RP_move_to_floor::generateFunnySentence()
{
  std::srand(time(0));
  int r = static_cast<int>(std::rand() * funny_sentences_.size());
  if (r < funny_sentences_.size())
    return funny_sentences_[r];
  else
    return "";
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
