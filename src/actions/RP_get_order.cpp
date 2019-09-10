#include "RP_get_order.h"

/* The implementation of RP_get_order.h */

/* Esta acción se encarga de pedir a los comensales de una mesa lo que quieren tomar

Estado inicial:
- El robot está en wp_mesa_N

Efecto de la acción
- Se crea arco "aks" para preguntar la comanda
- Se crean nodos con los elementos pedidos
  - la instancia es "mesa_N.clase.N"
  - el tipo es "clase"
- Se crean arcos "wants" desde el robot a los elementos pedidos
*/


#include <string>
#include <list>
#include <vector>

/* constructor */
RP_get_order::RP_get_order(const ros::NodeHandle& nh)
: nh_(nh),
  Action("get_order"),
  wp_id_()
{
}

void RP_get_order::activateCode()
{
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("wp"))
      wp_id_ = last_msg_.parameters[i].value;
    if (0 == last_msg_.parameters[i].key.compare("t"))
      table_id_ = last_msg_.parameters[i].value;
    else if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id_ = last_msg_.parameters[i].value;
  }
  // Obtain menu
  menu obtained_menu = gb_datahub::getMenu();
  std::string obtained_menu_;

  for(int i = 0; i < obtained_menu.products.size(); i++){
  	obtained_menu_ = obtained_menu.products[i].label + obtained_menu_;
  }

  graph_.add_edge(
    robot_id_,
    "say: Hello sir, Today we have " + obtained_menu_ + ". What do you want to take?",
    robot_id_);
  graph_.add_edge(robot_id_, "ask: bar_order.ask", robot_id_);
}

void RP_get_order::deActivateCode()
{
}

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
      order obtained_order;
      int counter = 0;
      for (auto it_food = food.begin(); it_food != food.end(); ++it_food)
      {
        std::string instance_id = table_id_ + "." + *it_food + "." + std::to_string(counter++);
        obtained_order.products[counter] = *it_food;
        graph_.add_node(instance_id, *it_food);
        graph_.add_edge(table_id_, "wants", instance_id);
      }
      ROS_INFO("Response received: %d",gb_datahub::postOrder(obtained_order));
      setSuccess();
    }
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_get_order");
    ros::NodeHandle nh("~");
    // create PDDL action subscriber
    RP_get_order rpmb(nh);
    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
