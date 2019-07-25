#include <ros/ros.h>

#include "rosplan_action_interface/RPActionInterface.h"
#include <bica_planning/Action.h>
#include <bica_planning/KMSClient.h>
#include <bica_graph/graph_client.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#ifndef SRC_ACTIONS_RP_CHECK_ORDER_H
#define SRC_ACTIONS_RP_CHECK_ORDER_H

class RP_check_order: public bica_planning::Action
{
public:
    /* constructor */
    explicit RP_check_order(const ros::NodeHandle& nh);

protected:
    /* listen to and process action_dispatch topic */
    void qrCallback(const std_msgs::String::ConstPtr& qr);
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    ros::Subscriber qr_sub_;
    bica_graph::GraphClient graph_;
    std::string robot_id;
    std::vector<std::string> order, order_in_robot;
};

#endif  // SRC_ACTIONS_RP_CHECK_ORDER_H
