#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <bica_planning/Action.h>
#include <bica_planning/KMSClient.h>
#include <bica_graph/graph_client.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"

#ifndef KCL_get_order
#define KCL_get_order

/**
* This file defines the ***** class.
* ***** is used to connect ROSPlan ***************.
* PDDL "*****" actions become "*****" actions.
*/

class RP_get_order: public bica_planning::Action
{
public:
    /* constructor */
    RP_get_order(ros::NodeHandle &nh);

protected:
    /* listen to and process action_dispatch topic */
    void activateCode();
    void deActivateCode();
    void step();

private:
    ros::NodeHandle nh_;
    bica_graph::GraphClient graph_;
    std::string wp_id_, robot_id_, table_id_;

    std::vector<std::string> splitSpaces(std::string raw_str);

};

#endif
