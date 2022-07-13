#include <ros/ros.h>
#include <matt_seek_tracker_n_user/seeker.h>

bool function_seeker(matt_seek_tracker_n_user::seeker::Request &req, matt_seek_tracker_n_user::seeker::Response &res) {

    res.human = req.human;
    res.tracker = req.tracker;

    if (res.human && res.tracker) {
        ROS_INFO("Delivered!\n");
    } else if (!res.human && res.tracker) {
        ROS_INFO("Giving up on tracker, seek human model!\n");
    } else if (res.human && !res.tracker) {
        ROS_ERROR ("ERROR! Seek tracker!");
        ROS_INFO("Giving up on tracker, seek human model!\n");
    } else {
        ROS_ERROR ("ERROR! redo the search!\n");
    }
 
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "seeker_server_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("seeker", function_seeker);
    ros::spin();

    return 0;
}