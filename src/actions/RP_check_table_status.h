#include <ros/ros.h>


#include "rosplan_action_interface/RPActionInterface.h"
#include <bica_planning/Action.h>
#include <bica_planning/KMSClient.h>
#include <bica_graph/graph_client.h>

#include <darknet_ros_3d/Darknet3DListener.h>
#include <std_msgs/String.h>

#include "yaml-cpp/yaml.h"

#include <vector>
#include <iostream>
#include <fstream>

#ifndef KCL_check_table_status
#define KCL_check_table_status

/**
* This file defines the ***** class.
* ***** is used to connect ROSPlan ***************.
* PDDL "*****" actions become "*****" actions.
*/

#define CHECKING_TIME   15.0

#define MIN_PROBABILITY   0.4

#define PERSON_MIN_X   -1.0
#define PERSON_MAX_X   1.0
#define PERSON_MIN_Y   -1.5
#define PERSON_MAX_Y   1.5
#define PERSON_MIN_Z   0.0
#define PERSON_MAX_Z   2.0
#define PERSON_MIN_SIZE_X   0.3
#define PERSON_MIN_SIZE_Y   0.3
#define PERSON_MIN_SIZE_Z   0.3
#define PERSON_MAX_SIZE_X   1.2
#define PERSON_MAX_SIZE_Y   1.2
#define PERSON_MAX_SIZE_Z   2.0

#define OBJECT_MIN_X   -1.0
#define OBJECT_MAX_X   1.0
#define OBJECT_MIN_Y   -0.5
#define OBJECT_MAX_Y   0.5
#define OBJECT_MIN_Z   0.7
#define OBJECT_MAX_Z   1.2
#define OBJECT_MIN_SIZE_X   0.05
#define OBJECT_MIN_SIZE_Y   0.05
#define OBJECT_MIN_SIZE_Z   0.05
#define OBJECT_MAX_SIZE_X   0.5
#define OBJECT_MAX_SIZE_Y   0.5
#define OBJECT_MAX_SIZE_Z   0.5


class RP_check_table_status: public bica_planning::Action
{
public:
    /* constructor */
    RP_check_table_status(const ros::NodeHandle& nh);

protected:
    /* listen to and process action_dispatch topic */
    void qrCallback(const std_msgs::String::ConstPtr& qr);

    void activateCode();
    void deActivateCode();

    void step();
private:
    ros::NodeHandle nh_;
    bica_graph::GraphClient graph_;

    darknet_ros_3d::Darknet3DListener obj_listener_;

    ros::Subscriber qr_sub_;

    std::string wp_id_, robot_id_, table_id_, table_status_, num_customers_;

    ros::Time start_check_;
};

#endif
