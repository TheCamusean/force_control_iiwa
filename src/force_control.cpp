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

struct EigenRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
    }

    void random()
    {
        world_H_base.setIdentity();
        jointPos.setRandom();
        baseVel.setRandom();
        jointVel.setRandom();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.8;
    }

    Eigen::Matrix4d world_H_base;
    Eigen::VectorXd jointPos;
    Eigen::Matrix<double,6,1> baseVel;
    Eigen::VectorXd jointVel;
    Eigen::Vector3d gravity;
};

int main(int argc, char **argv)
{
	ros::init (argc, argv, "force_controller");
	ROS_INFO("Starting program...");


		
	// Initialize the action server
	ros::NodeHandle n("");

	//ros::spin();
	ros::Rate r(10);

	iDynTree::ModelLoader mdlLoader;
	std::string modelFile = "a string with the urdf information";

    bool ok = mdlLoader.loadModelFromFile(modelFile);

	while (ros::ok())
	{

		ros::spinOnce();
		r.sleep();
	} 
		// ros::waitForShutdown();
	return 0;	
}






