#include <ros/ros.h>
#include <matt_seek_tracker_n_user/seeker.h>

bool function_seeker(matt_seek_tracker_n_user::seeker::Request  &req, matt_seek_tracker_n_user::seeker::Response &res) {

    res.human = req.human;
    res.tracker = req.tracker;
 
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "seeker_server_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("seeker", function_seeker);
    ros::spin();

    return 0;
}