#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//global variable for tracking if the robot needs to explore
bool crashed;
int index_mem;
float distance_mem;

//params
std::string laser_scan;
std::string cmd_vel_output;

void crash_recovery(const sensor_msgs::LaserScan::ConstPtr &scan)
{   
    //number of indices in laser scan array
    int size = scan->ranges.size();

    //loop iteration variable
    int i = 0;
    //tracker for nearby walls
    crashed = false;

    //will remeber locations of crashes to make best possible recovery assuming robot is still upright
    int indices_of_collisions[size];

    //The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance
    while (i < size)
    {
        if(scan->ranges[i] < scan->ranges[index_mem])
        {
            distance_mem = scan->ranges[i];
            index_mem = i;
        }
        if((i > 0 && i < 48) && scan->ranges[i] < 0.25)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 47 && i < 98) && scan->ranges[i] < 0.3)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 47 && i < 583) && scan->ranges[i] < 0.3)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 582 && i < 913) && scan->ranges[i] < 0.24)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 912 && i < 1803) && scan->ranges[i] < 0.2)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 1802 && i < 2000) && scan->ranges[i] < 0.25)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else
        {
            indices_of_collisions[i] = 3000;
        }
        i++;
    }
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //will give back a value of 1 if it is back
    if(crashed)
    {   
        ROS_INFO("Recovering crash . . . ");
        
        //sides of crash
        int A = 0;
        int B = 0;
        int C = 0;
        int D = 0;
        int E = 0;
        int F = 0;
        int G = 0;
        int H = 0;

        int c = 0;
        while(c < size)
        {
            if((1890 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1999) || (0 <= indices_of_collisions[c] && indices_of_collisions[c] <= 40)) A = 1;
            if((40 <= indices_of_collisions[c] && indices_of_collisions[c] <= 300)) B = 2;
            if((300 <= indices_of_collisions[c] && indices_of_collisions[c] <= 565)) C = 4;
            if((565 <= indices_of_collisions[c] && indices_of_collisions[c] <= 820)) D = 8;
            if((820 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1085)) E = 16;
            if((1085 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1361)) F = 32;
            if((1361 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1680)) G = 64;
            if((1680 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1890)) H = 128;
            c++;
        }

        int switch_setup = A + B + C + D + E + F + G + H;
        switch(switch_setup)
        {
            case 0: //no collisions
                ROS_INFO("case 0: No collisions detected");
                break;
            case 1:
                ROS_INFO("case 1: Collision back upper left");
                motorizer.angular.z = -0.5;
                motorizer.linear.x = 0.5;
                break;
            case 2:
                ROS_INFO("case 2: Collision back lower left");
                motorizer.angular.z = -0.5;
                motorizer.linear.x = 0.5;
                break;
            case 3:
                ROS_INFO("case 3: Collisions back left");
                motorizer.angular.z = -0.5;
                motorizer.linear.x = 0.5;
                break;
            case 4:
                ROS_INFO("case 4: Collision back lower right");
                motorizer.angular.z = 0.5;
                motorizer.linear.x = 0.5;
                break;
            case 5:
                ROS_INFO("case 2: Collision back upper left and back lower right");
                motorizer.angular.z = -0.5;
                motorizer.linear.x = 0.5;
                break;
            case 6:

                break;
            case 7:

                break;
            case 8:

                break;
            case 9:

                break;
            case 10:

                break;
            case 11:

                break;
            case 12:

                break;
            case 13:

                break;
            case 14:

                break;
            case 15:

                break;
            case 16:

                break;
            case 17:

                break;
            case 18:

                break;
            case 19:

                break;
            case 20:

                break;
            case 21:

                break;
            case 22:

                break;
            case 23:

                break;
            case 24:

                break;
            case 25:

                break;
            case 26:

                break;
            case 27:

                break;
            case 28:

                break;
            case 29:

                break;
            case 30:

                break;
            case 31:

                break;
            case 32:

                break;
            case 33:

                break;
            case 34:

                break;
            case 35:

                break;
            case 36:

                break;
            case 37:

                break;
            case 38:

                break;
            case 39:

                break;
            case 40:

                break;
            case 41:

                break;
            case 42:

                break;
            case 43:

                break;
            case 44:

                break;
            case 45:

                break;
            case 46:

                break;
            case 47:

                break;
            case 48:

                break;
            case 49:

                break;
            case 50:

                break;
            case 51:

                break;
            case 52:

                break;
            case 53:

                break;
            case 54:

                break;
            case 55:

                break;
            case 56:

                break;
            case 57:

                break;
            case 58:

                break;
            case 59:

                break;
            case 60:

                break;
            case 61:

                break;
            case 62:

                break;
            case 63:

                break;
            case 64:

                break;
            case 65:

                break;
            case 66:

                break;
            case 67:

                break;
            case 68:

                break;
            case 69:

                break;
            case 70:

                break;
            case 71:

                break;
            case 72:

                break;
            case 73:

                break;
            case 74:

                break;
            case 75:

                break;
            case 76:

                break;
            case 77:

                break;
            case 78:

                break;
            case 79:

                break;
            case 80:

                break;
            case 81:

                break;
            case 82:

                break;
            case 83:

                break;
            case 84:

                break;
            case 85:

                break;
            case 86:

                break;
            case 87:

                break;
            case 88:

                break;
            case 89:

                break;
            case 90:

                break;
            case 91:

                break;
            case 92:

                break;
            case 93:

                break;
            case 94:

                break;
            case 95:

                break;
            case 96:

                break;
            case 97:

                break;
            case 98:

                break;
            case 99:

                break;
            case 100:

                break;
            case 101:

                break;
            case 102:

                break;
            case 103:

                break;
            case 104:

                break;
            case 105:

                break;
            case 106:

                break;
            case 107:

                break;
            case 108:

                break;
            case 109:

                break;
            case 110:

                break;
            case 111:

                break;
            case 112:

                break;
            case 113:

                break;
            case 114:

                break;
            case 115:

                break;
            case 116:

                break;
            case 117:

                break;
            case 118:

                break;
            case 119:

                break;
            case 120:

                break;
            case 121:

                break;
            case 122:

                break;
            case 123:

                break;
            case 124:

                break;
            case 125:

                break;
            case 126:

                break;
            case 127:

                break;
            case 128:

                break;
            case 129:

                break;
            case 130:

                break;
            case 131:

                break;
            case 132:

                break;
            case 133:

                break;
            case 134:

                break;
            case 135:

                break;
            case 136:

                break;
            case 137:

                break;
            case 138:

                break;
            case 139:

                break;
            case 140:

                break;
            case 141:

                break;
            case 142:

                break;
            case 143:

                break;
            case 144:

                break;
            case 145:

                break;
            case 146:

                break;
            case 147:

                break;
            case 148:

                break;
            case 149:

                break;
            case 150:

                break;
            case 151:

                break;
            case 152:

                break;
            case 153:

                break;
            case 154:

                break;
            case 155:

                break;
            case 156:

                break;
            case 157:

                break;
            case 158:

                break;
            case 159:

                break;
            case 160:

                break;
            case 161:

                break;
            case 162:

                break;
            case 163:

                break;
            case 164:

                break;
            case 165:

                break;
            case 166:

                break;
            case 167:

                break;
            case 168:

                break;
            case 169:

                break;
            case 170:

                break;
            case 171:

                break;
            case 172:

                break;
            case 173:

                break;
            case 174:

                break;
            case 175:

                break;
            case 176:

                break;
            case 177:

                break;
            case 178:

                break;
            case 179:

                break;
            case 180:

                break;
            case 181:

                break;
            case 182:

                break;
            case 183:

                break;
            case 184:

                break;
            case 185:

                break;
            case 186:

                break;
            case 187:

                break;
            case 188:

                break;
            case 189:

                break;
            case 190:

                break;
            case 191:

                break;
            case 192:

                break;
            case 193:

                break;
            case 194:

                break;
            case 195:

                break;
            case 196:

                break;
            case 197:

                break;
            case 198:

                break;
            case 199:

                break;
            case 200:

                break;
            case 201:

                break;
            case 202:

                break;
            case 203:

                break;
            case 204:

                break;
            case 205:

                break;
            case 206:

                break;
            case 207:

                break;
            case 208:

                break;
            case 209:

                break;
            case 210:

                break;
            case 211:

                break;
            case 212:

                break;
            case 213:

                break;
            case 214:

                break;
            case 215:

                break;
            case 216:

                break;
            case 217:

                break;
            case 218:

                break;
            case 219:

                break;
            case 220:

                break;
            case 221:

                break;
            case 222:

                break;
            case 223:

                break;
            case 224:

                break;
            case 225:

                break;
            case 226:

                break;
            case 227:

                break;
            case 228:

                break;
            case 229:

                break;
            case 230:

                break;
            case 231:

                break;
            case 232:

                break;
            case 233:

                break;
            case 234:

                break;
            case 235:

                break;
            case 236:

                break;
            case 237:

                break;
            case 238:

                break;
            case 239:

                break;
            case 240:

                break;
            case 241:

                break;
            case 242:

                break;
            case 243:

                break;
            case 244:

                break;
            case 245:

                break;
            case 246:

                break;
            case 247:

                break;
            case 248:

                break;
            case 249:

                break;
            case 250:

                break;
            case 251:

                break;
            case 252:

                break;
            case 253:

                break;
            case 254:

                break;
            case 255:

                break;
            case 256:

                break;
        }

        //move according to conditions for a bit
        ros::Rate recovery_time(10);
        pub.publish(motorizer);
        recovery_time.sleep();

        //stopping robot before it crashes again
        motorizer.linear.x = 0;
        motorizer.angular.z = 0;
        pub.publish(motorizer);
    }
    else
    {
        ROS_INFO("There is no crash detected.");
    }
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "crash_recovery_v2_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //will assume robot is lost at start
    crashed = true;

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, crash_recovery);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}