// example_action_server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include <ammar_action_server/demoAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;
//some tunable constants, global
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.5; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01; //
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 


class AmmarActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<ammar_action_server::demoAction> as_;
    
    // here are some message types to communicate with our client(s)
    ammar_action_server::demoGoal goal_; // goal message, received from client
    ammar_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    ammar_action_server::demoFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


public:
    AmmarActionServer(); //define the body of the constructor outside of class definition

    ~AmmarActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<ammar_action_server::demoAction>::GoalConstPtr& goal);

	// here are a few useful utility functions:


	//double sgn(double x);
	//double min_spin(double spin_angle);
	//double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	//geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

	////void do_halt();
	//void do_move(double distance);
	//void do_spin(double spin_ang);

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 

    dist = 0.0; //FALSE!!    

 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
   // the distance is the hypotenuse length between current location to desired location
   dist = sqrt(pow(goal_pose.position.x - current_pose.position.x,2)+pow(goal_pose.position.y - current_pose.position.y,2)+pow(goal_pose.position.z - current_pose.position.z,2)) ;

 }
 else {
    heading = 0.0; //FALSE!!

 }

}



//int g_count = 0;
//bool g_count_failure = false;




};

AmmarActionServer::AmmarActionServer() :
   as_(nh_, "timer_action", boost::bind(&AmmarActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ammarActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}




void AmmarActionServer::executeCB(const actionlib::SimpleActionServer<ammar_action_server::demoAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");


    //do work here: this is where your interesting code goes
    
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    int npts = goal->nav_path.poses.size();
    ROS_INFO("received path request with %d poses",npts);    
    
    
   for (int i=0;i<npts;i++) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = goal->nav_path.poses[i].pose; //get next pose from vector of poses
        
        //WRITE THIS FNC: compute desired heading and travel distance based on current and desired poses
        get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
        ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired,
           pose_desired.position.x,pose_desired.position.y); 
        ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        ROS_INFO("travel distance = %f",travel_distance);         
        
            
        yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        do_spin(spin_angle); // carry out this incremental action
        // we will just assume that this action was successful--really should have sensor feedback here
        g_current_pose.orientation = pose_desired.orientation; // assumes got to desired orientation precisely
        
        //FIX THE NEXT LINE, BASED ON get_yaw_and_dist()

        do_move(travel_distance);  // move forward by distance retrieved from get_yaw_and_dist function
		g_current_pose.position.x = pose_desired.position.x;  // update x-coordinate
		g_current_pose.position.y = pose_desired.position.y;  // update y-coordinate
		
		if (as_.isPreemptRequested()){	
          ROS_WARN("goal cancelled!");
          result_.nav_path = g_current_pose;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          do_halt();
          return; // done with callback
 		}
    }
    
      //return 0;

        
 	   //if here, then goal is still valid; provide some feedback
 	  feedback_.fdbk = g_current_pose; // populate feedback message with current countdown value
 	   as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
      // countdown_val_--; //decrement the timer countdown
      // timer.sleep(); //wait 1 sec between loop iterations of this timer
    
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.nav_path = g_current_pose; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ammar_action_server"); // name this node 
	ros::NodeHandle n;

    ROS_INFO("instantiating the ammar_action_server: ");

    AmmarActionServer as_object; // create an instance of the class "AmmarActionServer"
    
    do_inits(n); //pass in a node handle so this function can set up publisher with it

    
    ROS_INFO("Ready to accept paths");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

