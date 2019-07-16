#include "RP_check_order.h"

/* The implementation of RP_check_order.h */

/* constructor */
RP_check_order::RP_check_order(ros::NodeHandle &nh)
: nh_(nh),
  Action("check_order"),
  robot_id()
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
}

void RP_check_order::deActivateCode(){}

void RP_check_order::qrCallback(const std_msgs::String::ConstPtr& qr)
{
  if (isActive())
  {
    YAML::Node root = YAML::Load(qr->data);
    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
    {
      graph_.add_node(it->as<std::string>(), "order");
      graph_.add_edge(robot_id, "has", it->as<std::string>());
    }

    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      if (edge.find("wants") != std::string::npos)
        order.push_back(it->get_target());
      else if(edge.find("has") != std::string::npos)
        order_in_robot.push_back(it->get_target());
    }

    if (order == order_in_robot)
      setSuccess();
    else
    {
      for (std::vector<std::string>::iterator it = order.begin(); it != order.end(); ++it)
      {
        if (std::find(order_in_robot.begin(), order_in_robot.end(), *it) == order_in_robot.end())
          graph_.add_edge(robot_id, "needs", *it);
      }
      add_predicate("order_needs_fix leia");
      setFail();
    }

    setSuccess();
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_check_order");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_check_order rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
    rpmb.runActionInterface();

    return 0;
}
