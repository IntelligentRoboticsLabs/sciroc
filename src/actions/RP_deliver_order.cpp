#include "RP_deliver_order.h"

/* The implementation of RP_deliver_order.h */

/* constructor */
RP_deliver_order::RP_deliver_order(ros::NodeHandle &nh)
: nh_(nh),
  Action("deliver_order"),
  robot_id()
{

}

void RP_deliver_order::activateCode()
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

  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string edge = it->get();
    if (edge.find("has") != std::string::npos && it->get_source() == robot_id)
    {
      graph_.remove_edge(*it);
    }
    else if (edge.find("wants") != std::string::npos)
    {
      graph_.add_edge(it->get_source(), "has", it->get_target());
      graph_.remove_edge(*it);
    }
    object_needed = it->get_target();
  }

  graph_.remove_edge(table_id, "status: needs_serving", table_id);
  graph_.add_edge(table_id, "status: already_served", table_id);
  setSuccess();
}

void RP_deliver_order::deActivateCode(){}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_deliver_order");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_deliver_order rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
    rpmb.runActionInterface();

    return 0;
}
