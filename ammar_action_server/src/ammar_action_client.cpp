// timer_client: works together with action server called "timer_action"
// in source: example_action_server_w_fdbk.cpp
// this code could be written using classes instead (e.g. like the corresponding server)
//  see: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ammar_action_server/demoAction.h> //reference action message in this package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

bool g_goal_active = false; //some global vars for communication with callbacks
geometry_msgs::Pose g_result_output ;
geometry_msgs::Pose g_fdbk ;
double i=0;



bool g_lidar_alarm=false; // global var for lidar alarm

void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!"); 
  }
} 


// This function will be called once when the goal completes
// this is optio`nal, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const ammar_action_server::demoResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output position is (x,y) = ( %d, %d)",result->pose.position.x ,result->pose.position.y);
    g_result_output= result->pose;
    g_goal_active=false;
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const ammar_action_server::demoFeedbackConstPtr& fdbk_msg) {
    ROS_INFO("Current x and y coordinates from feedback = %d , %d" ,fdbk_msg->fdbk.position.x ,fdbk_msg->fdbk.position.y);
    g_fdbk = fdbk_msg->fdbk; //make status available to "main()"

}
// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "path_client_node"); // name this node 
        ros::NodeHandle n;
        
        ros::Subscriber alarm_subscriber = n.subscribe("my_lidar_alarm",1,alarmCallback); 

        //ros::Rate main_timer(1.0);
        // here is a "goal" object compatible with the server, as defined in ammar_action_server/action
        ammar_action_server::demoGoal goal; 
        
        geometry_msgs::Quaternion quat;

        
        // use the name of our server, which is: timer_action (named in example_action_server_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<ammar_action_server::demoAction> action_client("path_action", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;
        
        geometry_msgs::PoseStamped pose_stamped;
		geometry_msgs::Pose pose;
       			
			
        
        //initial pose
			        
			quat = convertPlanarPhi2Quaternion(1.57); // get a quaternion corresponding to this heading
			pose_stamped.pose.orientation = quat;
			pose_stamped.pose.position.y=6.0; //  desired y-coord is 3
			pose_stamped.pose.position.x=0.0; // same x
			goal.nav_path.poses.push_back(pose_stamped);


			action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
		
		while(g_goal_active){
		
		if(g_lidar_alarm) {
			ROS_INFO("cancelling goal");
			action_client.cancelGoal(); //this is how one can cancel a goal in process
          while(g_goal_active){
			  ros::Duration(0.1).sleep();}
			quat = convertPlanarPhi2Quaternion(-1.57); // get a quaternion corresponding to this heading
			pose_stamped.pose.orientation = quat;
			pose_stamped.pose.position.y=0.0; //  desired y-coord is 3
			pose_stamped.pose.position.x=0.0; // same x
			goal.nav_path.poses.push_back(pose_stamped);
			//pose_stamped.pose = pose;
			action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
			ROS_INFO("new goal sent");
			//return;
			while(!g_goal_active){
			  ros::Duration(0.1).sleep();
			  }
			  break;
		}
		ros::spinOnce();
		ros::Duration(0.1).sleep();
		
		}
			  ros::spinOnce();
			  ros::Duration(0.1).sleep();
}
        

    
      


