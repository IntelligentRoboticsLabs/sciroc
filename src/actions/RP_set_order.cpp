#include "RP_set_order.h"

/* The implementation of RP_set_order.h */

/* constructor */
RP_set_order::RP_set_order(ros::NodeHandle &nh): nh_(nh), Action("set_order")
{

}

void RP_set_order::activateCode()
{
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id = last_msg_.parameters[i].value;
    else if(0 == last_msg_.parameters[i].key.compare("p"))
      person_id = last_msg_.parameters[i].value;
  }
  std::vector<std::string> order;
  std::string order_str;
  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string edge = it->get();
    if (edge.find("wants") != std::string::npos)
    {
      table_id = it->get_source();
      order.push_back(it->get_target());
    }
  }
  std::string table_id_raw = table_id;
  std::string delimiter = "wp_mesa_";
  table_id_raw.erase(0, table_id_raw.find(delimiter) + delimiter.length());
  order_str = "The table " + table_id_raw + " wants: ";
  for (std::vector<std::string>::iterator it = order.begin(); it != order.end(); ++it)
  {
    order_str += *it + ", ";
  }

  graph_.add_edge(robot_id, "say: " + order_str, person_id);
  setSuccess();
}

void RP_set_order::deActivateCode(){}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_set_order");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_set_order rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
    rpmb.runActionInterface();

    return 0;
}