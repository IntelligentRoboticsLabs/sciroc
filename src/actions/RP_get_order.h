#include <ros/ros.h>

#include "rosplan_action_interface/RPActionInterface.h"
#include <bica_planning/Action.h>
#include <bica_planning/KMSClient.h>
#include <bica_graph/graph_client.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"
#include "gb_datahub/gb_datahub.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/algorithm/string/replace.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#ifndef SRC_ACTIONS_RP_GET_ORDER_H
#define SRC_ACTIONS_RP_GET_ORDER_H

/**
* This file defines the ***** class.
* ***** is used to connect ROSPlan ***************.
* PDDL "*****" actions become "*****" actions.
*/

class RP_get_order: public bica_planning::Action
{
public:
    /* constructor */
    explicit RP_get_order(const ros::NodeHandle& nh);

protected:
    /* listen to and process action_dispatch topic */
    void activateCode();
    void deActivateCode();
    void step();

private:
    ros::NodeHandle nh_;
    bica_graph::GraphClient graph_;
    std::string wp_id_, robot_id_, table_id_;
    menu obtained_menu_;
    bool menu_delivered;
    std::vector<std::string> splitSpaces(std::string raw_str);
    bool checkInputOrder(const std::vector<std::string> &food);
};

#endif  // SRC_ACTIONS_RP_GET_ORDER_H
