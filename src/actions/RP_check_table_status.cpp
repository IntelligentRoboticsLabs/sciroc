#include "RP_check_table_status.h"

/* The implementation of RP_check_table_status.h */

/* constructor */
RP_check_table_status::RP_check_table_status(ros::NodeHandle &nh)
: nh_(nh),
  Action("check_table_status"),
  wp_id()
{

}

void RP_check_table_status::activateCode()
{
  qr_sub_ = nh_.subscribe("/barcode", 1, &RP_check_table_status::qrCallback, this);

  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("wp"))
      wp_id = last_msg_.parameters[i].value;
    else if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id = last_msg_.parameters[i].value;
  }
  graph_.add_edge(robot_id, "say: Checking table status", robot_id);
}

void RP_check_table_status::deActivateCode(){}

void RP_check_table_status::qrCallback(const std_msgs::String::ConstPtr& qr)
{
  if (isActive())
  {
    graph_.add_node(wp_id, "table");
    graph_.add_edge("world","waypoint", wp_id);
    YAML::Node root = YAML::Load(qr->data);
    //YAML::Node root = YAML::Load("{status: needs_serving, num_customers: 2}");
    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
    {
      graph_.add_edge(wp_id,it->first.as<std::string>() + ": " + it->second.as<std::string>(), wp_id);
      if (it->first.as<std::string>() == "status")
        table_status_ = it->second.as<std::string>();
      else if (it->first.as<std::string>() == "num_customers")
        num_customers_ = it->second.as<std::string>();
    }

    graph_.add_edge(
      robot_id,
      "say: The table status is " + table_status_ + " and have " + num_customers_ + " customers",
      robot_id);
    setSuccess();
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_check_table_status");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_check_table_status rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
    rpmb.runActionInterface();

    return 0;
}
