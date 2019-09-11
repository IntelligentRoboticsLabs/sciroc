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

#ifndef SRC_ACTIONS_RP_CHECK_ORDER_H
#define SRC_ACTIONS_RP_CHECK_ORDER_H


#define CHECK_ORDER_CHECKING_TIME   20.0
#define CHECK_ORDER_MIN_PROBABILITY   0.4


#define CHECK_ORDER_OBJECT_MIN_X   -1.0
#define CHECK_ORDER_OBJECT_MAX_X   1.0
#define CHECK_ORDER_OBJECT_MIN_Y   -0.5
#define CHECK_ORDER_OBJECT_MAX_Y   0.5
#define CHECK_ORDER_OBJECT_MIN_Z   0.7
#define CHECK_ORDER_OBJECT_MAX_Z   1.2
#define CHECK_ORDER_OBJECT_MIN_SIZE_X   0.05
#define CHECK_ORDER_OBJECT_MIN_SIZE_Y   0.05
#define CHECK_ORDER_OBJECT_MIN_SIZE_Z   0.05
#define CHECK_ORDER_OBJECT_MAX_SIZE_X   0.5
#define CHECK_ORDER_OBJECT_MAX_SIZE_Y   0.5
#define CHECK_ORDER_OBJECT_MAX_SIZE_Z   0.5

class RP_check_order: public bica_planning::Action
{
public:
    /* constructor */
    explicit RP_check_order(const ros::NodeHandle& nh);

    void step();

protected:
    /* listen to and process action_dispatch topic */
    void qrCallback(const std_msgs::String::ConstPtr& qr);
    void activateCode();
    void deActivateCode();
private:
    ros::NodeHandle nh_;
    ros::Subscriber qr_sub_;
    bica_graph::GraphClient graph_;
    std::string robot_id_;
    std::vector<std::string> order, order_in_robot;

    darknet_ros_3d::Darknet3DListener obj_listener_;

    ros::Time start_check_;
};

#endif  // SRC_ACTIONS_RP_CHECK_ORDER_H
