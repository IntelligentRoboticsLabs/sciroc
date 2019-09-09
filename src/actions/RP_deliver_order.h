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

#ifndef SRC_ACTIONS_RP_DELIVER_ORDER_H
#define SRC_ACTIONS_RP_DELIVER_ORDER_H

/**
* This file defines the ***** class.
* ***** is used to connect ROSPlan ***************.
* PDDL "*****" actions become "*****" actions.
*/

class RP_deliver_order: public bica_planning::Action
{
public:
    /* constructor */
    explicit RP_deliver_order(const ros::NodeHandle& nh);

protected:
    /* listen to and process action_dispatch topic */
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    bica_graph::GraphClient graph_;
    std::string robot_id, table_id;
};

#endif  // SRC_ACTIONS_RP_DELIVER_ORDER_H
