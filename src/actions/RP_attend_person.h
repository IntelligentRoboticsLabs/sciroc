#include <ros/ros.h>

#include "rosplan_action_interface/RPActionInterface.h"
#include <bica_planning/Action.h>
#include <bica_planning/KMSClient.h>
#include <bica_graph/graph_client.h>
#include <std_msgs/String.h>
#include "gb_datahub/gb_datahub.h"


#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#ifndef SRC_ACTIONS_RP_ATTEND_PERSON_H
#define SRC_ACTIONS_RP_ATTEND_PERSON_H

/**
* This file defines the ***** class.
* ***** is used to connect ROSPlan ***************.
* PDDL "*****" actions become "*****" actions.
*/

class RP_attend_person: public bica_planning::Action
{
public:
    explicit RP_attend_person(const ros::NodeHandle& nh);

protected:
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    bica_graph::GraphClient graph_;
    std::string robot_id, table_id;
};

#endif  // SRC_ACTIONS_RP_ATTEND_PERSON_H
