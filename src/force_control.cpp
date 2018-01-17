#include <ros/ros.h>

// Eigen headers
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>

// Helpers function to convert between
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>
#include <math.h>


int main(int argc, char **argv)
{
	ros::init (argc, argv, "force_controller");
	ROS_INFO("Starting program...");


		
	// Initialize the action server
	ros::NodeHandle n("");

	//ros::spin();
	ros::Rate r(10);
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	} 
		// ros::waitForShutdown();
	return 0;	
}






