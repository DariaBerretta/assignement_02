#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "robot_control/user_interaction.h"
#include <iostream>
#include <math.h>
using namespace std;

// Number of element in the ranges vector
const int N = 720;
//Coefficent for the linear velocity
const float lin_coeff = 0.35;
//Coefficent fot the angular velocity
const float ang_coeff = 1.0;
//Value of the angular velocity at time (t-1)
float prev_v_ang = 0.0;
//Value of the user coefficent to increment/decrement the speed of the robot
float user_coeff = 1.0;

// Command to run the map
//rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world&

// Define the publisher for publish the velocity on the cmd_vel topic
	ros::Publisher pub_velocity;
	

/*
 * This function rapresent the user interaction service
 * if the request is equal to 1 the user_coeff is incremented in order to
 * increase the speed of the robot
 * if the request is equal to -1 the user_coedd is decremented in order to
 * decrease the speed of the robot
 * The response assume the value of the user_coeff
 */
bool userCoeff(robot_control::user_interaction::Request&req, robot_control::user_interaction::Response&res){

	if(req.user_answer ==1){
		user_coeff = user_coeff +0.1;
	}
	else if(req.user_answer == -1){
		user_coeff = user_coeff - 0.1;
		if (user_coeff <= 0){
			user_coeff = user_coeff + 0.1;
		}
	}
	
	res.user_coeff = user_coeff;
	return true;
	
}
	
int find_max_i(float ranges[]){

	int max = 0;
	for(int i = 0; i<N; i++){
		if (ranges[i] >= ranges[max]){
			max = i;
		}
	}
	
	if(ranges[0] < ranges[N-1]){
		max = max + 90;
	}
	else{
		max = max - 90;
	}
		
	return max;
}

/*
 * This function compute the linear velocity for the robot 
 */	
float linear_velocity(float ranges[]){
	//take the distance in front of the robot 
	// -> element 360 of vector ranges
	float velocity = 0.0;
	float d = ranges[360];
	
	//compute the velocity as the moltiplication of the distance, a simple
	//linear coefficent and the user coefficent (the last one is used
	//to determine the increment or decrement of the velocity)
	velocity = d*lin_coeff*user_coeff;
	cout << "user coeff: "<< user_coeff<<" velocity: "<<velocity << endl;
	
	return velocity;
}

/*
 * This function covert the index of the vector ranges in to an
 * angle using this equation:
 * alpha = (i * pi/720) - pi/2
 */
float index_into_angle(int index){
	float alpha = (index*(M_PI/720)) - (M_PI/2);
	return alpha;
}

/*
 * This function compute the angular velocity to assign to the robot
 */
float angular_velocity(float ranges[]){
	float angular=0.0;
	int max_index = find_max_i(ranges);
	
	//convert the index in an agle
	float alpha = index_into_angle(max_index);
	
	//compute the angular velocity as
	//the sum of the previous velocity plus a coefficent, multuply by
	//the current angle
	angular = (prev_v_ang + ang_coeff) * alpha;
	
	//save of the angular velocity
	prev_v_ang = angular;
	
	return angular;
}

/*
 * Function called by the subscriber
 * It compute the linear and angular velocity and publish them into
 * the cmd_vel topic
 */
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//Copy of the vector ranges
	float ranges[N];
	for(int i = 0; i < N; i++){
		ranges[i]= msg->ranges[i];
	}
	
	//Initialization of the linear and angular velocity
	float linear_v = 0.0;
	float angular_v = 0.0;
	
	//Call to the functions which computes the two different velocity
	linear_v = linear_velocity(ranges);
	angular_v = angular_velocity(ranges);
	
	// Create a new velocity for the robot
	// and pubblication of in into the topic cmd_vel
	geometry_msgs::Twist vel;
	vel.linear.x = linear_v;
	vel.angular.z = angular_v;
	pub_velocity.publish(vel);
	
}


int main (int argc , char **argv){
	
	//Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc,argv,"robot_control_node");
	ros::NodeHandle nh;
	
	// Define the subscriber to base_scan topic
	ros::Subscriber sub_info = nh.subscribe ("/base_scan", 1, laserCallback);
	
	//Define the server for the user interaction
	ros::ServiceServer service = nh.advertiseService("/user_interaction", userCoeff);
	
	//set up the publisher to the cmd_vel topic
	pub_velocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::spin();
	return 0;
}
