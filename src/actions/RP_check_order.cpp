#include "RP_check_order.h"

/* The implementation of RP_check_order.h */


/* Esta acción se encarga de chequear si lo que nos has dado el barman es correcto

Estado inicial:
- El robot está en wp_bar_location
- Una de las mesas tiene arcos "wants" con su comanda a nodos del tipo de lo que pide

Efecto de la acción
- Crea arcos "has" desde el robot a los elementos percibidos
- Crea arcos "not needs" desde el robot a los elementos percibidos que no concuerdan con la comanda
- Crea arcos "needs" desde el robot a los elementos de la mesa que no han sido percibidos
*/

#include <algorithm>
#include <string>
#include <list>
#include <vector>

/* constructor */
RP_check_order::RP_check_order(const ros::NodeHandle& nh)
: nh_(nh),
  Action("check_order"),
  robot_id_(),
  obj_listener_("bar")
{
  darknet_ros_3d::ObjectConfiguration obj_conf;

  obj_conf.min_probability = CHECK_ORDER_MIN_PROBABILITY;
  obj_conf.min_x = CHECK_ORDER_OBJECT_MIN_X;
  obj_conf.max_x = CHECK_ORDER_OBJECT_MAX_X;
  obj_conf.min_y = CHECK_ORDER_OBJECT_MIN_Y;
  obj_conf.max_y = CHECK_ORDER_OBJECT_MAX_Y;
  obj_conf.min_z = CHECK_ORDER_OBJECT_MIN_Z;
  obj_conf.max_z = CHECK_ORDER_OBJECT_MAX_Z;
  obj_conf.min_size_x = CHECK_ORDER_OBJECT_MIN_SIZE_X;
  obj_conf.min_size_y = CHECK_ORDER_OBJECT_MIN_SIZE_Y;
  obj_conf.min_size_z = CHECK_ORDER_OBJECT_MIN_SIZE_Z;
  obj_conf.max_size_x = CHECK_ORDER_OBJECT_MAX_SIZE_X;
  obj_conf.max_size_y = CHECK_ORDER_OBJECT_MAX_SIZE_Y;
  obj_conf.max_size_z = CHECK_ORDER_OBJECT_MAX_SIZE_Z;
  obj_conf.dynamic = false;

  obj_listener_.add_class("italian_biscotti", obj_conf);
  obj_listener_.add_class("smoothie", obj_conf);
  obj_listener_.add_class("mango_juice", obj_conf);
  obj_listener_.add_class("crisps", obj_conf);
  obj_listener_.add_class("water", obj_conf);
  obj_listener_.add_class("sandwich", obj_conf);
  obj_listener_.add_class("toastie", obj_conf);
  obj_listener_.add_class("veggie_pot", obj_conf);
  obj_listener_.add_class("wrap", obj_conf);
  obj_listener_.add_class("espresso", obj_conf);
  obj_listener_.add_class("cappuccino", obj_conf);
  obj_listener_.add_class("americano", obj_conf);
}

void RP_check_order::activateCode()
{
  qr_sub_ = nh_.subscribe("/barcode", 1, &RP_check_order::qrCallback, this);

  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("r"))
      robot_id_ = last_msg_.parameters[i].value;
  }

  graph_.add_edge(robot_id_, "say: Checking order", robot_id_);

  graph_.add_edge(robot_id_, "want_see", "bar");

  start_check_ = ros::Time::now();

  obj_listener_.reset();
  obj_listener_.set_working_frame("bar");
  obj_listener_.set_active();

  order.clear();
  order_in_robot.clear();
}

void RP_check_order::deActivateCode()
{
  graph_.remove_edge(robot_id_, "want_see", "bar");
  obj_listener_.set_inactive();
}


void
RP_check_order::step()
{
  if (!isActive())
    return;

  obj_listener_.print();

  if ((ros::Time::now() - start_check_).toSec() >= CHECK_ORDER_CHECKING_TIME)
  {
    // Removing has if they exist
    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      if (edge.find("has") != std::string::npos &&
        (it->get_source() == robot_id_ || it->get_source() == "bar"))
        graph_.remove_edge(*it);
    }

    // Adding "has" edges
    int count = 0;
    for (const auto& object : obj_listener_.get_objects())
    {
      std::string instance_id = "bar." + object.class_id + "." + std::to_string(count++);
      ROS_WARN("Object detected: %s", instance_id.c_str());
      graph_.add_node(instance_id, object.class_id);
      graph_.add_edge("bar", "has", instance_id);
    }

    // Comparing has with wants
    edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      if (edge.find("wants") != std::string::npos)
        order.push_back(graph_.get_node(it->get_target()).get_type());
      else if (edge.find("has") != std::string::npos)
        order_in_robot.push_back(graph_.get_node(it->get_target()).get_type());
    }

    ROS_INFO("WANTS (%zu)", order.size());
    for (const std::string& obj : order)
    {
      ROS_INFO("\t[%s]", obj.c_str());
    }
    ROS_INFO("HAS (%zu)", order_in_robot.size());
    for (const std::string& obj : order_in_robot)
    {
      ROS_INFO("\t[%s]", obj.c_str());
    }


    std::sort(order.begin(), order.end());
    std::sort(order_in_robot.begin(), order_in_robot.end());

    // If lists are equals, clean before finishing
    if (order == order_in_robot)
    {
      for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
      {
        std::string edge = it->get();
        if (edge == "not needs")
        {
          graph_.remove_edge(*it);
          graph_.remove_node(graph_.get_node(it->get_target()).get_id());
        }
        else if (edge == "needs")
          graph_.remove_edge(*it);
      }
      graph_.add_edge(robot_id_,"say: Thank you! I will deliver the order.", "barman");
    }
    else //Need fix
    {
      int counter = 0;
      for (std::vector<std::string>::iterator it = order.begin(); it != order.end(); ++it)
      {
        if (std::find(order_in_robot.begin(), order_in_robot.end(), *it) == order_in_robot.end())
        {
          std::string instance_id = robot_id_ + "." + *it + "." + std::to_string(counter++);
          graph_.add_node(instance_id, *it);
          graph_.add_edge(robot_id_,"needs", instance_id);
        }
      }

      for (std::vector<std::string>::iterator it = order_in_robot.begin(); it != order_in_robot.end(); ++it)
      {
        if (std::find(order.begin(), order.end(), *it) == order.end())
        {
          std::string instance_id = robot_id_ + "." + *it + "." + std::to_string(counter++);
          graph_.add_node(instance_id, *it);
          graph_.add_edge(robot_id_,"not needs", instance_id);
        }
      }
      add_predicate("order_needs_fix " + robot_id_);
    }

    obj_listener_.set_inactive();
    setSuccess();
  }
}

// HAY QUE PROBARLO!!

void RP_check_order::qrCallback(const std_msgs::String::ConstPtr& qr)
{
  if (isActive())
  {
    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      // Remove robot-has edges
      if (edge.find("has") != std::string::npos && it->get_source() == robot_id_)
        graph_.remove_edge(*it);
    }

    YAML::Node root = YAML::Load(qr->data);
    int count = 0;
    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
    {
      std::string instance_id = "bar." + it->as<std::string>() + "." + std::to_string(count++);
      ROS_WARN("Object detected: %s", it->as<std::string>().c_str());
      graph_.add_node(instance_id, it->as<std::string>());
      graph_.add_edge("bar", "has", instance_id);
    }

    edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      if (edge.find("wants") != std::string::npos)
        order.push_back(graph_.get_node(it->get_target()).get_type());
      else if (edge.find("has") != std::string::npos)
        order_in_robot.push_back(graph_.get_node(it->get_target()).get_type());
    }
    std::sort(order.begin(), order.end());
    std::sort(order_in_robot.begin(), order_in_robot.end());

    if (order == order_in_robot)
    {
      for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
      {
        std::string edge = it->get();
        if (edge == "not needs")
        {
          graph_.remove_edge(*it);
          graph_.remove_node(graph_.get_node(it->get_target()).get_id());
        }
        else if (edge == "needs")
          graph_.remove_edge(*it);
      }
      graph_.add_edge(robot_id_,"say: Thank you! I will deliver the order.", "barman");
    }
    else
    {
      int counter = 0;
      for (std::vector<std::string>::iterator it = order.begin(); it != order.end(); ++it)
      {
        if (std::find(order_in_robot.begin(), order_in_robot.end(), *it) == order_in_robot.end())
        {
          std::string instance_id = robot_id_ + "." + *it + "." + std::to_string(counter++);
          graph_.add_node(instance_id, *it);
          graph_.add_edge(robot_id_,"needs", instance_id);
        }
      }

      for (std::vector<std::string>::iterator it = order_in_robot.begin(); it != order_in_robot.end(); ++it)
      {
        if (std::find(order.begin(), order.end(), *it) == order.end())
        {
          std::string instance_id = robot_id_ + "." + *it + "." + std::to_string(counter++);
          graph_.add_node(instance_id, *it);
          graph_.add_edge(robot_id_,"not needs", instance_id);
        }
      }
      add_predicate("order_needs_fix " + robot_id_);
    }
    obj_listener_.set_inactive();
    setSuccess();
    order.clear();
    order_in_robot.clear();
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_check_order");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_check_order rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
      &KCL_rosplan::RPActionInterface::dispatchCallback,
      dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));

    rpmb.runActionInterface();

    return 0;
}
