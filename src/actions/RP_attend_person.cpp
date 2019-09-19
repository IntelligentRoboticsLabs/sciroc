#include "RP_attend_person.h"

/* The implementation of RP_attend_person.h */

/* Esta acción se encarga de anunciar que un cliente ha sido ubicado en una mesa

Estado inicial:
- El robot ha detectado a una persona
- La persona ha sido guiada
- El robot está al lado de la mesa en la que dejar a la persona

Efecto de la acción
- La mesa con estado (autoarco) 'ready' pasa a estado (autoarco) 'needs_serving'
*/

RP_attend_person::RP_attend_person(const ros::NodeHandle& nh)
: nh_(nh),
  Action("attend_person"),
  robot_id()
{
}

void RP_attend_person::activateCode()
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

  graph_.begin_batch();
  graph_.add_edge(robot_id, "say: Here's your table.", robot_id);
  graph_.remove_edge(table_id, "status: ready", table_id);
  graph_.add_edge(table_id, "status: needs_serving", table_id);
  graph_.add_edge(table_id, "num_customers: 1", table_id);
  graph_.flush();

  //Datahub integration
  std::vector<table> table_datahub = gb_datahub::getTable(table_id);
  table_datahub[0].status = "needs_serving";
  table_datahub[0].customers = 1;

  setSuccess();
}

void RP_attend_person::deActivateCode()
{
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_attend_person");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_attend_person rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
