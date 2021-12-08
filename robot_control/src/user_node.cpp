#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_control/user_interaction.h"
#include "std_srvs/Empty.h"
#include <iostream>
#include <math.h>  
using namespace std;

/*
 * This function interact with the user.
 * It ask him if he want to increase/decrease the speed of the robot or 
 * if he want to reset the position of it.
 * if the user press:
 * - 'w' -> means that the speed must be incremented, so the function 
 * returns 1
 * - 's' -> means that the speed must be decremented, so the function
 * returns -1
 * - With another digit the function resturns 0 that's means that the user
 * wants reset the position of the robot.
 */
int calluser(){
	char answer;
	int rtn = 0;
	
	//Message displayed
	cout<<"Do you want to increase the speed"<<endl;
	cout<<"digit 'w' to increase the speed"<<endl;
	cout<<"digit 's' to decrease the speed"<<endl;
	cout<<"digit any other letter to reset the positions of the robot"<<endl;
	cin>> answer;
	
	if(answer == 'w'){
		rtn = 1;
	}
	else if (answer == 's'){
		rtn = -1;
	}
	else{
		rtn = 0;
		
	}
	
	return rtn;
}

int main (int argc , char **argv){
	
	//Node initialization
	ros::init(argc,argv, "user_node");
	ros::NodeHandle nh;
	
	while(1){
		
		//Call to the function whitch interacts with the user
		int user_answer = calluser();
		
		if(user_answer == 0){
			//Initialization of the client to the "/reset_positions" service
			ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/reset_positions");
			std_srvs::Empty srv;
			
			//Call to the service
			client.waitForExistence();
			client.call(srv);
		}
		else{
			//Initialization of the client to the "/user_interaction" service
			ros::ServiceClient client = nh.serviceClient<robot_control::user_interaction>("/user_interaction");
			robot_control::user_interaction srv;
			
			//Assignment of the request
			srv.request.user_answer = user_answer;
			
			//Call to the service
			client.waitForExistence();
			client.call(srv);
		}
	}
	
	return 0;
}
