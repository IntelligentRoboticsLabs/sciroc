#include "RP_check_table_status.h"

/* The implementation of RP_check_table_status.h */

/* Esta acción se encarga de chequear el estado de la mesa y el número de clientes

Estado inicial:
- El robot está en wp_table_N
- La mesa tiene un auto-arco "needs_check"

Efecto de la acción
- Crea un auto-arco "status: STATUS" desde la mesa, donde STATUS es {ready, needs_serving, needs_cleaning
  already_served}
- Crea un auto-arco "num_customers: N" desde la mesa, donde N es el número de personas
*/

#include <string>

/* constructor */
RP_check_table_status::RP_check_table_status(const ros::NodeHandle& nh)
: nh_(nh),
  Action("check_table_status"),
  wp_id_(),
  obj_listener_("base_footprint")
{
  qr_sub_ = nh_.subscribe("/barcode", 1, &RP_check_table_status::qrCallback, this);

  darknet_ros_3d::ObjectConfiguration obj_conf;

  obj_conf.min_probability = CHECK_TABLE_MIN_PROBABILITY;
  obj_conf.min_x = CHECK_TABLE_OBJECT_MIN_X;
  obj_conf.max_x = CHECK_TABLE_OBJECT_MAX_X;
  obj_conf.min_y = CHECK_TABLE_OBJECT_MIN_Y;
  obj_conf.max_y = CHECK_TABLE_OBJECT_MAX_Y;
  obj_conf.min_z = CHECK_TABLE_OBJECT_MIN_Z;
  obj_conf.max_z = CHECK_TABLE_OBJECT_MAX_Z;
  obj_conf.min_size_x = CHECK_TABLE_OBJECT_MIN_SIZE_X;
  obj_conf.min_size_y = CHECK_TABLE_OBJECT_MIN_SIZE_Y;
  obj_conf.min_size_z = CHECK_TABLE_OBJECT_MIN_SIZE_Z;
  obj_conf.max_size_x = CHECK_TABLE_OBJECT_MAX_SIZE_X;
  obj_conf.max_size_y = CHECK_TABLE_OBJECT_MAX_SIZE_Y;
  obj_conf.max_size_z = CHECK_TABLE_OBJECT_MAX_SIZE_Z;
  obj_conf.dynamic = false;

  obj_listener_.add_class("botte", obj_conf);
  obj_listener_.add_class("book", obj_conf);
  obj_listener_.add_class("tie", obj_conf);
  obj_listener_.add_class("cup", obj_conf);

  darknet_ros_3d::ObjectConfiguration person_conf;

  person_conf.min_probability = CHECK_TABLE_MIN_PROBABILITY;
  person_conf.min_x = CHECK_TABLE_PERSON_MIN_X;
  person_conf.max_x = CHECK_TABLE_PERSON_MAX_X;
  person_conf.min_y = CHECK_TABLE_PERSON_MIN_Y;
  person_conf.max_y = CHECK_TABLE_PERSON_MAX_Y;
  person_conf.min_z = CHECK_TABLE_PERSON_MIN_Z;
  person_conf.max_z = CHECK_TABLE_PERSON_MAX_Z;
  person_conf.min_size_x = CHECK_TABLE_PERSON_MIN_SIZE_X;
  person_conf.min_size_y = CHECK_TABLE_PERSON_MIN_SIZE_Y;
  person_conf.min_size_z = CHECK_TABLE_PERSON_MIN_SIZE_Z;
  person_conf.max_size_x = CHECK_TABLE_PERSON_MAX_SIZE_X;
  person_conf.max_size_y = CHECK_TABLE_PERSON_MAX_SIZE_Y;
  person_conf.max_size_z = CHECK_TABLE_PERSON_MAX_SIZE_Z;
  person_conf.dynamic = false;

  obj_listener_.add_class("person", person_conf);
}

void RP_check_table_status::activateCode()
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
  graph_.add_edge(robot_id_, "say: Checking table status", robot_id_);

  graph_.add_edge("sonny", "want_see", table_id_);

  start_check_ = ros::Time::now();

  obj_listener_.reset();
  obj_listener_.set_working_frame(table_id_);
  obj_listener_.set_active();
}

void
RP_check_table_status::deActivateCode()
{
  graph_.remove_edge("sonny", "want_see", table_id_);
  obj_listener_.set_inactive();
}

void
RP_check_table_status::step()
{
  if (!isActive())
    return;

  obj_listener_.print();

  if ((ros::Time::now() - start_check_).toSec() >= CHECK_TABLE_CHECKING_TIME)
  {
    int count = 0;
    bool object_in_table = false;
    int num_customers = 0;
    std::string table_status;

    for (const auto& object : obj_listener_.get_objects())
    {
      /* Jonatan: Esto no es necesario
      std::string instance_id = table_id_ + "." + object.class_id + "." + std::to_string(count++);
      graph_.add_node(instance_id, object.class_id);
      graph_.add_edge(instance_id, "is_in", table_id_);
      */

      /*tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      tf2::Transform table2obj(q, object.central_point);
      graph_.add_edge(table_id_, table2obj, instance_id, true);
      */
      if (object.class_id != "person")
        object_in_table = true;
      else
        num_customers++;
    }

    if (!object_in_table && num_customers == 0)
      table_status = "ready";
    else if(object_in_table && num_customers == 0)
      table_status = "needs_cleaning";
    else if(!object_in_table && num_customers > 0)
      table_status = "needs_serving";
    else if(object_in_table && num_customers > 0)
      table_status = "already_served";

    graph_.add_edge(table_id_, "status: " + table_status , table_id_);
    graph_.add_edge(table_id_, "num_customers: " + std::to_string(num_customers) , table_id_);

    //Datahub intregration
    table table_datahub;
    table_datahub.id = table_id_;
    table_datahub.type = "Table";
    table_datahub.customers = num_customers;
    table_datahub.status = table_status;
    ROS_INFO("Response received: %d",gb_datahub::postTable(table_datahub));


    graph_.add_edge(
      robot_id_,
      "say: The table status is " + table_status + " and have " + std::to_string(num_customers) + " customers",
      robot_id_);

    graph_.remove_edge(table_id_, "needs_check", table_id_);
    graph_.remove_edge("sonny", "want_see", table_id_);

    obj_listener_.set_inactive();
    setSuccess();
  }
}

void
RP_check_table_status::qrCallback(const std_msgs::String::ConstPtr& qr)
{
  if (isActive())
  {
    YAML::Node root = YAML::Load(qr->data);

    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
    {
      graph_.add_edge(table_id_, it->first.as<std::string>() + ": " + it->second.as<std::string>(), table_id_);
      if (it->first.as<std::string>() == "status")
        table_status_ = it->second.as<std::string>();
      else if (it->first.as<std::string>() == "num_customers")
        num_customers_ = it->second.as<std::string>();
    }

    graph_.add_edge(
      robot_id_,
      "say: The table status is " + table_status_ + " and have " + num_customers_ + " customers",
      robot_id_);

    graph_.remove_edge(table_id_, "needs_check", table_id_);
    setSuccess();
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_check_table_status");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_check_table_status rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
