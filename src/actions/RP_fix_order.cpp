#include "RP_fix_order.h"

/* The implementation of RP_fix_order.h */

/* Esta acción se encarga de comunicar al barman posibles errores en los elementos entregados

Estado inicial:
- El robot está en wp_bar_location, y ya ha ejecutado "check_order"
- El robot puede tener arcos "has", "needs" y "not needs"

Efecto de la acción
- Se crea arco "say" para informar al barman de posibles errores en la comanda
*/


#include <string>
#include <list>

/* constructor */
RP_fix_order::RP_fix_order(const ros::NodeHandle& nh)
: nh_(nh),
  Action("fix_order"),
  robot_id()
{
  wrong_object = "";
}

void RP_fix_order::activateCode()
{
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s",
      last_msg_.parameters[i].key.c_str(),
      last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id = last_msg_.parameters[i].value;
  }

  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string edge = it->get();
    if (edge == "needs")
      object_needed = it->get_target();
    else if (edge == "not needs")
      wrong_object = it->get_target();
  }

  if (wrong_object == "")
    graph_.add_edge(
      robot_id,
      "say: Excuse me sir, I need a " + object_needed + ". Could you bring me it?",
      "barman");
  else
    graph_.add_edge(
      robot_id,
      "say: Excuse me sir, I need a " + object_needed + " and I don't need a " + \
        wrong_object + ". Could you replace it?",
      "barman");

  wrong_object = "";
  setSuccess();
}

void RP_fix_order::deActivateCode()
{
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_fix_order");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_fix_order rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
