// Ammar Nahari's modification on lidar_alarm.cpp.  3//17

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 




double theta= -0.3;// a set of 30 anlgles, bounded between -1.5 and 1.5 radians

double MIN_SAFE_DISTANCE= 2.0; // to be determened according to theta

int ping_index; // NOT real; callback will have to find this

// these values to be set within the laser callback
double ping_dist=0; // global var to hold length of a SINGLE LIDAR ping--in front

double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0; // thease values are to bo inported from the laser scanner
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;

// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) 

{
    
    angle_min_ = laser_scan.angle_min;
    angle_max_ = laser_scan.angle_max;
    angle_increment_ = laser_scan.angle_increment;
    range_min_ = laser_scan.range_min;
    range_max_ = laser_scan.range_max;


    for (theta=-0.3 ; theta <0.4 ; theta += 0.1){ 
		
	ping_index = (int) abs((theta-angle_min_)/angle_increment_);
	
	ping_dist = laser_scan.ranges[ping_index];

	

	ROS_INFO("For theta= %f Ping dist= %f and safe dist= %f ", theta, ping_dist, MIN_SAFE_DISTANCE);


	if ((ping_dist<MIN_SAFE_DISTANCE) ) {
	ROS_WARN("DANGER, WILL ROBINSON!!");
	laser_alarm_=true;
	//break;
	}
	else {
	laser_alarm_=false;
	}



    }
	
	std_msgs::Bool lidar_alarm_msg;
	lidar_alarm_msg.data = laser_alarm_;
   	lidar_alarm_publisher_.publish(lidar_alarm_msg);
   	std_msgs::Float32 lidar_dist_msg;
   	lidar_dist_msg.data = ping_dist;
   	lidar_dist_publisher_.publish(lidar_dist_msg); 
   	  
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "ps8_lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("ps8_lidar_alarm", 1);
   	lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

