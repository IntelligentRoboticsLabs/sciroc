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

#ifndef KCL_set_order
#define KCL_set_order

/**
* This file defines the ***** class.
* ***** is used to connect ROSPlan ***************.
* PDDL "*****" actions become "*****" actions.
*/

class RP_set_order: public bica_planning::Action
{
public:
    /* constructor */
    RP_set_order(ros::NodeHandle &nh);
protected:
    /* listen to and process action_dispatch topic */
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    bica_graph::GraphClient graph_;
    std::string robot_id, person_id, table_id;
};

#endif
