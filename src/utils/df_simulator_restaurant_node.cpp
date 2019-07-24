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

/* Author: Jonatan Gines jginesclavero@gmail.com */

/* Mantainer: Jonatan Gines jginesclavero@gmail.com */

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <dialogflow_ros_msgs/DialogflowResult.h>
#include <dialogflow_ros_msgs/DialogflowParameter.h>
#include <std_msgs/Int64.h>

class DFSimulator
{
public:
  DFSimulator(): nh_("~")
  {
    dialog_sub_ = nh_.subscribe("/dialogflow_simulator/step", 1, &DFSimulator::dfStepCallback, this);
    df_pub_ = nh_.advertise<dialogflow_ros_msgs::DialogflowResult>("/dialogflow_client/results", 1);
    orders_.push_back("coke");
    orders_.push_back("beer");
    orders_.push_back("water");
  }


  void dfStepCallback(const std_msgs::Int64::ConstPtr& msg)
  {
    dialogflow_ros_msgs::DialogflowResult df_msg;
    dialogflow_ros_msgs::DialogflowParameter parameter;
    std::vector<dialogflow_ros_msgs::DialogflowParameter> parameter_vector;
    std::vector<std::string> values;

    switch (msg->data)
    {
      case 1:
        df_msg.intent = "bar_start.action";
        break;
      case 2:
        df_msg.intent = "bar_order.ask";
        parameter.param_name = "order";
        for (auto it = orders_.begin(); it != orders_.end(); ++it)
        {
          values.push_back(*it);
        }
        parameter.value = values;
        parameter_vector.push_back(parameter);
        df_msg.parameters = parameter_vector;
        break;
      case 3:
        df_msg.intent = "orderReady.ask";
        break;
      case 4:
        df_msg.intent = "confirmOrder.ask";
        break;
    }
    df_pub_.publish(df_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher df_pub_;
  ros::Subscriber dialog_sub_;
  std::vector<std::string> orders_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "df_simulator_node");
  DFSimulator df_simulator;
  ros::spin();
  return 0;
}
