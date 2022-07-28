#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//params
std::string laser_scan;
std::string cmd_vel_output;

void type_of_wall_identifier(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //variables and arrays
    int i = 0;
    int indices_of_walls[2000];
    float distances_of_walls[2000];
    int num_of_walls = 0;
    int wall_starting_positions[100];
    int wall_ending_positions[100];
    
    int wall_type[2000];
    
    //will loop to check all indices of the laser scan
    while (i < size)
    {   
        if(scan->ranges[i] < 5) //will store values of walls within 5 meters
        {
            indices_of_walls[i] = i;
            distances_of_walls[i] = scan->ranges[i];
        }
        else //will give empty values to walls not within 5 meters
        {
            indices_of_walls[i] = 0;
            distances_of_walls[i] = 0;
        }
        i++;
    }
    i = 0;
    int wb = 0;
    int wa = 0;
    while (i < size)
    {
        if(indices_of_walls[i] != 0) //if there is a wall at an index
        {   
            if(i == 0) //if first index
            {   
                num_of_walls++;
                wall_starting_positions[num_of_walls] = i;
                if(indices_of_walls[i+1] = 0)
                {
                    wall_ending_positions[num_of_walls] = i;
                }
                ROS_INFO("\nThere is a wall at index 0");
                
            }
            else if(i == 1999) //if last index
            {
                wall_ending_positions[num_of_walls] = i;
                if(indices_of_walls[i-1] == 0)
                {
                    wall_starting_positions[num_of_walls] = i;
                    num_of_walls++;
                }
                ROS_INFO("\nThere is a wall at index 1999");
            }
            else //if any middle indexes
            {
                if(indices_of_walls[i-1] == 0) //check if wall is continuous before
                {
                    num_of_walls++;
                    wall_starting_positions[num_of_walls] = i;
                }
                else if(indices_of_walls[i-1] < indices_of_walls[i])
                {
                    wb = 1;
                }
                if(indices_of_walls[i-1] != 0 && distances_of_walls[i-1] + 0.2 < distances_of_walls[i])
                {   
                    num_of_walls++;
                    wall_ending_positions[num_of_walls-1] = i-1; 
                    wall_starting_positions[num_of_walls] = i;
                } 
                if(indices_of_walls[i-1] != 0 && distances_of_walls[i-1] - 0.2 > distances_of_walls[i]) 
                {   
                    num_of_walls++;
                    wall_ending_positions[num_of_walls-1] = i-1; 
                    wall_starting_positions[num_of_walls] = i;
                }
                if(indices_of_walls[i+1] == 0) //check if wall is continuous after
                {
                    wall_ending_positions[num_of_walls] = i;
                }
                else if(indices_of_walls[i+1] < indices_of_walls[i])
                {
                    wa = 2;
                }
            }

            int wall_calc = wb + wa;
            float slope_b = 0.0;
            float slope_ba = 0.0;
            float slope_a = 0.0;

            //by calculating slopes before and after a point on a wall, we can determine if it is a curved or straight wall
            if(indices_of_walls[i+1] != 0)
            {
                slope_a = (distances_of_walls[i] + distances_of_walls[i+1])/2;
            }
            if(indices_of_walls[i-1] != 0)
            {
                slope_b = (distances_of_walls[i] + distances_of_walls[i-1])/2;
            }
            if((indices_of_walls[i-1] != 0) && (indices_of_walls[i+1] != 0))
            {
                slope_ba = (distances_of_walls[i+1] + distances_of_walls[i-1])/2;
            }

            switch(wall_calc)
            {   
                //both wall before and wall after are farther away then current wall
                case 0: //must be an outside corner or the closest part of a curve
                    wall_type[i] = 2; //outside corner
                    break;
                //wall before is less
                case 1: //must be a straight wall or curve
                    if((slope_a < slope_b + 0.05) && (slope_a > slope_b - 0.05)) 
                    {
                        wall_type[i] = 3; //straight
                    }
                    else 
                    {
                        wall_type[i] = 4; //curved
                    }
                    break;
                //wall after is less
                case 2: //must be a straight wall or curve
                    if((slope_a < slope_b + 0.05) && (slope_a > slope_b - 0.05)) 
                    {
                        wall_type[i] = 3;
                    }
                    else 
                    {
                        wall_type[i] = 4;
                    }
                    break;
                //both wall before and after are closer than current wall
                case 3: 
                    wall_type[i] = 5; //inside corner
                    break;
            }
        }
        else
        {
            wall_type[i] = 1; //no wall
        }
        
        if(wall_type[i] == 1)
        {
            ROS_INFO("\nThere is no wall at scan->ranges[%d]", i);
        }
        else if(wall_type[i] == 2)
        {
            ROS_INFO("\nThere is an outside corner at scan->ranges[%d]", i);
        }
        else if(wall_type[i] == 3)
        {
            ROS_INFO("\nThere is a straight wall at scan->ranges[%d]", i);
        }
        else if(wall_type[i] == 4)
        {
            ROS_INFO("\nThere is a curved wall at scan->ranges[%d]", i);
        }
        else if(wall_type[i] == 5)
        {
            ROS_INFO("\nThere is an inside corner at scan->ranges[%d]", i);
        }
        else
        {
            ROS_INFO("\nThe wall type at scan->ranges[%d] is unknown.", i);
        }
        i++;
    }

    ROS_INFO("\nThe number of walls detected is: %d", num_of_walls);

    int printer = num_of_walls;
    while(printer > 0)
    {
        ROS_INFO("\nOne wall is located between scan->ranges[%d] and scan->ranges[%d]", wall_starting_positions[printer], wall_ending_positions[printer]);
        printer = printer - 1;
    }   
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "type_of_wall_identifier_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, type_of_wall_identifier);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
