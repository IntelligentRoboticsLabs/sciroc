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
    void qrCallback(const std_msgs::String::ConstPtr& qr);
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    ros::Subscriber qr_sub_;
    bica_graph::GraphClient graph_;
    std::string wp_id, robot_id;
};

#endif
