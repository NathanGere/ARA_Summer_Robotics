#include <ros/ros.h>
#include <tf/transform_listener.h>

void transformPrinter(tf::TransformListener &listener, std::string identifier, std::string side);
std::string closest_wall_frame;
std::string laser_frame;
std::string topic_name;
std::string closest_wall_frame_right;
std::string closest_wall_frame_center;
std::string closest_wall_frame_left;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_tf_listener_node");
    ros::NodeHandle nh("~");

    nh.param<std::string>("laser_frame", laser_frame, "/base_link");
    nh.param<std::string>("topic_name", topic_name, "/scan");
    nh.param<std::string>("closest_wall_frame", closest_wall_frame, "/nearest_wall");
    nh.param<std::string>("closest_wall_frame_right", closest_wall_frame_right, "/nearest_wall_right");
    nh.param<std::string>("closest_wall_frame_center", closest_wall_frame_center, "/nearest_wall_center");
    nh.param<std::string>("closest_wall_frame_right", closest_wall_frame_left, "/nearest_wall_left");

    tf::TransformListener listener;

    ros::Rate rate(10.0); //setting loop rate for the while
    while(nh.ok())
    {
        transformPrinter(listener, closest_wall_frame, "anywhere"); //gives tag for which transform to read and gives a tag for printing
        transformPrinter(listener, closest_wall_frame_right, "on the right");
        transformPrinter(listener, closest_wall_frame_center, "towards the center");
        transformPrinter(listener, closest_wall_frame_left, "one the left");
        rate.sleep();
    }
    return 0;
}
void transformPrinter(tf::TransformListener &listener, std::string identifier, std::string side)
{
    tf::StampedTransform transform;

        try 
        {
            listener.lookupTransform(laser_frame, identifier, ros::Time(0), transform);
        } 
        catch(tf::TransformException ex) 
        {
            ROS_ERROR("%s %s", identifier.c_str(), ex.what());
            ros::Duration(1.0).sleep();
        }
    float xVal = transform.getOrigin().x();
    float yVal = transform.getOrigin().y();
    ROS_INFO("The nearest wall %s is located at (%f, %f)", side.c_str(), xVal, yVal);
}