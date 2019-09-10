#include <ros/ros.h>


#include "rosplan_action_interface/RPActionInterface.h"
#include <bica_planning/Action.h>
#include <bica_planning/KMSClient.h>
#include <bica_graph/graph_client.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"

#include <darknet_ros_3d/Darknet3DListener.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#ifndef SRC_ACTIONS_RP_CHECK_WAITING_PERSON_H
#define SRC_ACTIONS_RP_CHECK_WAITING_PERSON_H


#define CHECK_WAITING_PERSON_CHECKING_TIME   10.0
#define CHECK_WAITING_PERSON_MIN_PROBABILITY   0.4


#define CHECK_WAITING_PERSON_OBJECT_MIN_X   -1.5
#define CHECK_WAITING_PERSON_OBJECT_MAX_X   1.5
#define CHECK_WAITING_PERSON_OBJECT_MIN_Y   -1.5
#define CHECK_WAITING_PERSON_OBJECT_MAX_Y   1.5
#define CHECK_WAITING_PERSON_OBJECT_MIN_Z   0.0
#define CHECK_WAITING_PERSON_OBJECT_MAX_Z   2.0
#define CHECK_WAITING_PERSON_OBJECT_MIN_SIZE_X   0.15
#define CHECK_WAITING_PERSON_OBJECT_MIN_SIZE_Y   0.15
#define CHECK_WAITING_PERSON_OBJECT_MIN_SIZE_Z   0.45
#define CHECK_WAITING_PERSON_OBJECT_MAX_SIZE_X   1.0
#define CHECK_WAITING_PERSON_OBJECT_MAX_SIZE_Y   1.0
#define CHECK_WAITING_PERSON_OBJECT_MAX_SIZE_Z   2.5

class RP_check_waiting_person: public bica_planning::Action
{
public:
    /* constructor */
    explicit RP_check_waiting_person(const ros::NodeHandle& nh);

    void step();

protected:
    /* listen to and process action_dispatch topic */
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    bica_graph::GraphClient graph_;
    darknet_ros_3d::Darknet3DListener obj_listener_;

    std::string robot_id_, person_id_, wp_id_, zone_id_;

    ros::Time start_check_;

    void qrCallback(const std_msgs::String::ConstPtr& qr);
};

#endif  // SRC_ACTIONS_RP_CHECK_WAITING_PERSON_H
