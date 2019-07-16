#include "RP_get_order.h"

/* The implementation of RP_get_order.h */

/* constructor */
RP_get_order::RP_get_order(ros::NodeHandle &nh)
: nh_(nh),
  Action("get_order"),
  wp_id()
{

}

void RP_get_order::activateCode()
{
  qr_sub_ = nh_.subscribe("/barcode", 1, &RP_get_order::qrCallback, this);

  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("wp"))
      wp_id = last_msg_.parameters[i].value;
    else if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id = last_msg_.parameters[i].value;
  }
  graph_.add_edge(robot_id, "say: Hi, What do you want to drink?", robot_id);
}

void RP_get_order::deActivateCode(){}

void RP_get_order::qrCallback(const std_msgs::String::ConstPtr& qr)
{
  if (isActive())
  {
    YAML::Node root = YAML::Load(qr->data);
    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
    {
      graph_.add_node(it->as<std::string>(), "order");
      graph_.add_edge(wp_id, "wants", it->as<std::string>());
    }
    graph_.add_edge(robot_id, "say: Your order it's comming!", robot_id);
    setSuccess();
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_get_order");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_get_order rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
    rpmb.runActionInterface();

    return 0;
}
