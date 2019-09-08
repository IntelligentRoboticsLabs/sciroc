#include "RP_check_order.h"

/* The implementation of RP_check_order.h */


#include <algorithm>
#include <string>
#include <list>
#include <vector>

/* constructor */
RP_check_order::RP_check_order(const ros::NodeHandle& nh)
: nh_(nh),
  Action("check_order"),
  robot_id(),
  obj_listener_(std::list<std::string>{"cup"}, "barra")
{
}

void RP_check_order::activateCode()
{
  qr_sub_ = nh_.subscribe("/barcode", 1, &RP_check_order::qrCallback, this);

  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id = last_msg_.parameters[i].value;
  }

  start_check_ = ros::Time::now();

  obj_listener_.reset();
  obj_listener_.set_working_frame("barra");
  obj_listener_.set_active();
}

void RP_check_order::deActivateCode()
{
  obj_listener_.set_inactive();
}


void
RP_check_order::step()
{
  if (!isActive())
    return;

  if ((ros::Time::now() - start_check_).toSec() >= CHECK_ORDER_CHECKING_TIME)
  {

    obj_listener_.filter_objects("cup", CHECK_ORDER_MIN_PROBABILITY,
      CHECK_ORDER_OBJECT_MIN_X, CHECK_ORDER_OBJECT_MAX_X, CHECK_ORDER_OBJECT_MIN_Y,
      CHECK_ORDER_OBJECT_MAX_Z, CHECK_ORDER_OBJECT_MIN_Z, CHECK_ORDER_OBJECT_MAX_Z,
      CHECK_ORDER_OBJECT_MIN_SIZE_X, CHECK_ORDER_OBJECT_MAX_SIZE_X, CHECK_ORDER_OBJECT_MIN_SIZE_Y,
      CHECK_ORDER_OBJECT_MAX_SIZE_Y, CHECK_ORDER_OBJECT_MIN_SIZE_Z, CHECK_ORDER_OBJECT_MAX_SIZE_Z);


    for (const auto& object : obj_listener_.get_objects())
    {

    }

    obj_listener_.set_inactive();
    setSuccess();
  }
}

void RP_check_order::qrCallback(const std_msgs::String::ConstPtr& qr)
{
  if (isActive())
  {
    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      if (edge.find("has") != std::string::npos && it->get_source() == robot_id)
        graph_.remove_edge(*it);
    }

    YAML::Node root = YAML::Load(qr->data);
    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
    {
      graph_.add_node(it->as<std::string>(), "order");
      graph_.add_edge(robot_id, "has", it->as<std::string>());
    }

    edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      if (edge.find("wants") != std::string::npos)
        order.push_back(it->get_target());
      else if (edge.find("has") != std::string::npos)
        order_in_robot.push_back(it->get_target());
    }
    std::sort(order.begin(), order.end());
    std::sort(order_in_robot.begin(), order_in_robot.end());
    if (order == order_in_robot)
    {
      for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
      {
        std::string edge = it->get();
        if (edge == "not needs" && graph_.get_node(it->get_target()).get_type() == "order")
        {
          graph_.remove_edge(*it);
          graph_.remove_node(graph_.get_node(it->get_target()).get_id());
        }
        else if (edge == "needs" && graph_.get_node(it->get_target()).get_type() == "order")
          graph_.remove_edge(*it);
      }
      graph_.add_edge(robot_id, "say: Thank you! I will deliver the order.", "barman");
    }
    else
    {
      for (std::vector<std::string>::iterator it = order.begin(); it != order.end(); ++it)
      {
        if (std::find(order_in_robot.begin(), order_in_robot.end(), *it) == order_in_robot.end())
          graph_.add_edge(robot_id, "needs", *it);
      }

      for (std::vector<std::string>::iterator it = order_in_robot.begin(); it != order_in_robot.end(); ++it)
      {
        if (std::find(order.begin(), order.end(), *it) == order.end())
          graph_.add_edge(robot_id, "not needs", *it);
      }
      add_predicate("order_needs_fix leia");
    }
    setSuccess();
    order.clear();
    order_in_robot.clear();
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_check_order");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_check_order rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
