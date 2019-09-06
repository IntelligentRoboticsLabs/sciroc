#include "RP_check_table_status.h"

/* The implementation of RP_check_table_status.h */

#include <string>

/* constructor */
RP_check_table_status::RP_check_table_status(const ros::NodeHandle& nh)
: nh_(nh),
  tf_listener_(tfBuffer_),
  Action("check_table_status"),
  wp_id_()
{
  object_sub_ = nh_.subscribe("/darknet_ros_3d/bounding_boxes", 10, &RP_check_table_status::objectsCallback, this);
  qr_sub_ = nh_.subscribe("/barcode", 1, &RP_check_table_status::qrCallback, this);
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

  graph_.add_edge("leia", "want_see", table_id_);

  start_check_ = ros::Time::now();
  objects_.clear();
}

void
RP_check_table_status::deActivateCode()
{
  graph_.remove_edge("leia", "want_see", table_id_);
}

bool
RP_check_table_status::is_already_detected(const CheckedObject& object)
{
  for (const auto& test_obj : objects_)
  {
    if ((test_obj.class_id == object.class_id) &&
        (fabs(test_obj.central_point.x() - object.central_point.x()) < (test_obj.size_x / 2.0 + object.size_x / 2.0)) &&
        (fabs(test_obj.central_point.y() - object.central_point.y()) < (test_obj.size_y / 2.0 + object.size_y / 2.0)) &&
        (fabs(test_obj.central_point.z() - object.central_point.z()) < (test_obj.size_z / 2.0 + object.size_z / 2.0)))
    {
      return true;
    }
  }
  return false;
}

bool
RP_check_table_status::checkPoint(const CheckedObject& object)
{
  if (object.class_id == "person")
  {
    float dist2table = sqrt(object.central_point.x() * object.central_point.x() +
      object.central_point.y() * object.central_point.y());  // We only use x and y for dist

    if (  !is_already_detected(object) &&
         (object.probability > MIN_PROBABILITY) &&
         (dist2table < PERSON_MAX_DIST) &&
         (object.size_x > PERSON_MIN_SIZE_X && object.size_x < PERSON_MAX_SIZE_X) &&
         (object.size_y > PERSON_MIN_SIZE_Y && object.size_y < PERSON_MAX_SIZE_Y) &&
         (object.size_z > PERSON_MIN_SIZE_Z && object.size_z < PERSON_MAX_SIZE_Z))
      return true;
  }

  if (object.class_id == "cup")
  {
    if ( !is_already_detected(object) &&
         (object.probability > MIN_PROBABILITY) &&
         (object.central_point.x() > OBJECT_MIN_X && object.central_point.x() < OBJECT_MAX_X) &&
         (object.central_point.y() > OBJECT_MIN_Y && object.central_point.y() < OBJECT_MAX_Y) &&
         (object.central_point.z() > OBJECT_MIN_Z && object.central_point.z() < OBJECT_MAX_Z) &&
         (object.size_y > OBJECT_MIN_SIZE_Y && object.size_y < OBJECT_MAX_SIZE_Y) &&
         (object.size_z > OBJECT_MIN_SIZE_Z && object.size_z < OBJECT_MAX_SIZE_Z) &&
         (object.size_x > OBJECT_MIN_SIZE_X && object.size_x < OBJECT_MAX_SIZE_X) &&
         (object.size_y > OBJECT_MIN_SIZE_Y && object.size_y < OBJECT_MAX_SIZE_Y) &&
         (object.size_z > OBJECT_MIN_SIZE_Z && object.size_z < OBJECT_MAX_SIZE_Z))
      return true;
  }

  return false;
}

void
RP_check_table_status::objectsCallback(const darknet_ros_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{
  if (!isActive())
    return;

  for (auto bb : msg->bounding_boxes)
  {
    geometry_msgs::TransformStamped any2table_msg;
    tf2::Transform any2table;

    std::string error;
    if (tfBuffer_.canTransform(msg->header.frame_id, table_id_,
      ros::Time(0), ros::Duration(0.1), &error))
      any2table_msg = tfBuffer_.lookupTransform(msg->header.frame_id, table_id_, ros::Time(0));
    else
    {
      ROS_ERROR("Can't transform %s", error.c_str());
      return;
    }

    tf2::Stamped<tf2::Transform> aux;
    tf2::convert(any2table_msg, aux);

    any2table = aux;

    CheckedObject point;

    point.central_point.setX((bb.xmax + bb.xmin)/2.0);
    point.central_point.setY((bb.ymax + bb.ymin)/2.0);
    point.central_point.setZ((bb.zmax + bb.zmin)/2.0);

    point.central_point = any2table.inverse() * point.central_point;

    point.size_x = bb.xmax - bb.xmin;
    point.size_y = bb.ymax - bb.ymin;
    point.size_z = bb.zmax - bb.zmin;

    point.class_id = bb.Class;
    point.probability = bb.probability;

    if (checkPoint(point))
      objects_.push_back(point);
  }
}


void
RP_check_table_status::step()
{
  if (!isActive())
    return;

  if ((ros::Time::now() - start_check_).toSec() >= CHECKING_TIME)
  {
    int count = 0;
    bool object_in_table = false;
    int num_customers = 0;
    std::string table_status;
    for (const auto& object : objects_)
    {
      std::string instance_id = table_id_ + "." + object.class_id + "." + std::to_string(count++);
      graph_.add_node(instance_id, object.class_id);
      graph_.add_edge(instance_id, "is_in", table_id_);

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      tf2::Transform table2obj(q, object.central_point);
      graph_.add_edge(table_id_, table2obj, instance_id, true);

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

    graph_.add_edge(
      robot_id_,
      "say: The table status is " + table_status + " and have " + std::to_string(num_customers) + " customers",
      robot_id_);

    graph_.remove_edge(table_id_, "needs_check", table_id_);
    graph_.remove_edge("leia", "want_see", table_id_);
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
