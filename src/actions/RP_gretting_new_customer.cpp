#include "RP_gretting_new_customer.h"

/* The implementation of RP_gretting_new_customer.h */

/* constructor */
RP_gretting_new_customer::RP_gretting_new_customer(const ros::NodeHandle& nh)
: nh_(nh),
  Action("gretting_new_customer"),
  robot_id()
{
}

void RP_gretting_new_customer::activateCode()
{
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s",
      last_msg_.parameters[i].key.c_str(),
      last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id = last_msg_.parameters[i].value;
    else if (0 == last_msg_.parameters[i].key.compare("t"))
      table_id = last_msg_.parameters[i].value;
  }
  
  graph_.add_edge(robot_id, "say: Here's your table.", robot_id);

  graph_.remove_edge(table_id, "status: ready", table_id);
  graph_.add_edge(table_id, "status: needs_serving", table_id);
  graph_.add_edge(table_id, "num_customers: 1", table_id);

  setSuccess();
}

void RP_gretting_new_customer::deActivateCode()
{
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_gretting_new_customer");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_gretting_new_customer rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
