#include "RP_check_waiting_person.h"

/* The implementation of RP_check_waiting_person.cpp */

/* Esta acción se encarga de chequear si hay personas esperando en la entrada

Estado inicial:
- El robot está en wp_waiting, que está en la zona waiting
- Hay una persona en wp_entry, que está en la zona entry

Efecto de la acción
- Persona detectada en la waiting zone
*/

#include <algorithm>
#include <string>
#include <list>
#include <vector>

/* constructor */
RP_check_waiting_person::RP_check_waiting_person(const ros::NodeHandle& nh)
: nh_(nh),
  Action("check_waiting_person"),
  wp_id_(),
  obj_listener_("base_footprint")
{
  darknet_ros_3d::ObjectConfiguration person_conf;

  person_conf.min_probability = CHECK_WAITING_PERSON_MIN_PROBABILITY;
  person_conf.min_x = CHECK_WAITING_PERSON_MIN_X;
  person_conf.max_x = CHECK_WAITING_PERSON_MAX_X;
  person_conf.min_y = CHECK_WAITING_PERSON_MIN_Y;
  person_conf.max_y = CHECK_WAITING_PERSON_MAX_Y;
  person_conf.min_z = CHECK_WAITING_PERSON_MIN_Z;
  person_conf.max_z = CHECK_WAITING_PERSON_MAX_Z;
  person_conf.min_size_x = CHECK_WAITING_PERSON_MIN_SIZE_X;
  person_conf.min_size_y = CHECK_WAITING_PERSON_MIN_SIZE_X;
  person_conf.min_size_z = CHECK_WAITING_PERSON_MIN_SIZE_Z;
  person_conf.max_size_x = CHECK_WAITING_PERSON_MAX_SIZE_X;
  person_conf.max_size_y = CHECK_WAITING_PERSON_MAX_SIZE_Y;
  person_conf.max_size_z = CHECK_WAITING_PERSON_MAX_SIZE_Z;
  person_conf.dynamic = false;

  obj_listener_.add_class("person", person_conf);
}

void RP_check_waiting_person::activateCode()
{
  ROS_INFO("=============> Starting action");

  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id_ = last_msg_.parameters[i].value;
    if (0 == last_msg_.parameters[i].key.compare("p"))
      person_id_ = last_msg_.parameters[i].value;
    if (0 == last_msg_.parameters[i].key.compare("wp"))
      wp_id_ = last_msg_.parameters[i].value;
    if (0 == last_msg_.parameters[i].key.compare("z"))
      zone_id_ = last_msg_.parameters[i].value;
  }

  graph_.add_edge(robot_id_, "say: Checking if there are people waiting", robot_id_);
  graph_.add_edge("sonny", "want_see", zone_id_);

  start_check_ = ros::Time::now();

  obj_listener_.reset();
  obj_listener_.set_working_frame(zone_id_);
  obj_listener_.set_active();
}

void RP_check_waiting_person::deActivateCode()
{
  graph_.remove_edge("sonny", "want_see", zone_id_);
  obj_listener_.set_inactive();
}


void RP_check_waiting_person::step()
{
  if (!isActive())
    return;

  ROS_INFO("=============> Step RP_check_waiting_person");

  if ((ros::Time::now() - start_check_).toSec() >= CHECK_WAITING_PERSON_CHECKING_TIME)
  {
    int num_people_waiting = 0;
    for (const auto& object : obj_listener_.get_objects())
    {
      if (object.class_id == "person")
        num_people_waiting++;
    }


    // Crear nodo persona
    // Arco hacia la persona 'waiting'

    graph_.remove_edge("sonny", "want_see", zone_id_);
    obj_listener_.set_inactive();
    if (num_people_waiting > 0)
    {
      graph_.add_edge(robot_id_, "say: I have found " + std::to_string(num_people_waiting) +
        " people in the waiting zone. Please, follow me.", robot_id_);
      setSuccess();
    }
    else
    {
      graph_.add_edge(robot_id_, "say: There is nobody in the waiting zone.", robot_id_);
      setFail();
    }
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_check_waiting_person");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_check_waiting_person rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
