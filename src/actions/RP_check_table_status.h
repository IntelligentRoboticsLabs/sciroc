#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <bica_planning/Action.h>
#include <bica_planning/KMSClient.h>
#include <std_msgs/String.h>

#ifndef KCL_check_table_status
#define KCL_check_table_status

/**
* This file defines the ***** class.
* ***** is used to connect ROSPlan ***************.
* PDDL "*****" actions become "*****" actions.
*/

class RP_check_table_status: public bica_planning::Action
{
public:
    /* constructor */
    RP_check_table_status(ros::NodeHandle &nh);

protected:
    /* listen to and process action_dispatch topic */
    void qrCallback(const std_msgs::String::ConstPtr& qr);
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    ros::Subscriber qr_sub_;
};

#endif
