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
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("wp"))
      wp_id = last_msg_.parameters[i].value;
    else if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id = last_msg_.parameters[i].value;
  }
  graph_.add_edge(robot_id, "ask: bar_order.ask", robot_id);

}

void RP_get_order::deActivateCode(){}

std::vector<std::string> RP_get_order::splitSpaces(std::string raw_str)
{
  std::vector<std::string> output;
  std::istringstream iss(raw_str);
  std::string s;
  while (getline(iss, s, ' '))
  {
    output.push_back(s);
  }
  return output;
}

void RP_get_order::step()
{
  if (!isActive())
    return;

  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string edge = it->get();
    if (edge.find("response") != std::string::npos)
    {
      graph_.remove_edge(*it);
      std::string response_raw = edge;
      std::string delimiter = "response: ";
      response_raw.erase(0, response_raw.find(delimiter) + delimiter.length());
      std::vector<std::string> food = splitSpaces(response_raw);
      for (auto it_food = food.begin(); it_food != food.end(); ++it_food)
      {
        graph_.add_node(*it_food, "order");
        graph_.add_edge(wp_id, "wants", *it_food);
      }
      setSuccess();
    }
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
