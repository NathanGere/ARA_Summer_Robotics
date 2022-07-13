#include <ros/ros.h>
#include <matt_seek_tracker_n_user/seeker.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "seeker_client_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<matt_seek_tracker_n_user::seeker>("seeker");

    int human = 5;
    int tracker = 4;
    ROS_INFO("Looking for tracker and human model!\n");

    if (human == 5 && tracker == 4) {
        ROS_INFO ("Found human and tracker!\n");
        return true;
    } else if (human == 5 && tracker != 4) {
        ROS_ERROR ("Found human but not tracker!\n");
        return false;
    } else if (human != 5 && tracker == 4) {
        ROS_ERROR ("Found tracker but no human!\n");
        return false;
    } else {
        ROS_ERROR ("Did not find human or tracker!\n");
        return false;
    }
    ros::spin();

    return 0;
}