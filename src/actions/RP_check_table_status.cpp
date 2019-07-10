#include "RP_check_table_status.h"

/* The implementation of RP_check_table_status.h */

/* constructor */
RP_check_table_status::RP_check_table_status(ros::NodeHandle &nh)
: nh_(nh),
  Action("check_table_status")
{

}

void RP_check_table_status::activateCode()
{
  qr_sub_ = nh_.subscribe("/barcode", 1, &RP_check_table_status::qrCallback, this);
}

void RP_check_table_status::deActivateCode()
{
  qr_sub_.shutdown();
}

void RP_check_table_status::qrCallback(const std_msgs::String::ConstPtr& qr)
{
  ROS_INFO("Qr readed -- %s", qr->data.c_str());
  setSuccess();
}


//void RP_check_table_status::setLocRobot()
//{
//  remove_predicate("robot_at leia wp_station");
//  add_predicate("robot_at leia wp_home");
//}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_check_table_status");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    RP_check_table_status rpmb(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
    rpmb.runActionInterface();

    return 0;
}
