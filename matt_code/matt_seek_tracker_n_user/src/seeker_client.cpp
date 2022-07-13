#include <ros/ros.h>
#include <matt_seek_tracker_n_user/seeker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "seeker_client");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<matt_seek_tracker_n_user::seeker>("add_two_ints")
}