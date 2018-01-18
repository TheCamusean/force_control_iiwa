#include <ros/ros.h>

// Joint State

#include "sensor_msgs/JointState.h"
// Eigen headers
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/KinDynComputations.h>

#include <iDynTree/Model/FreeFloatingState.h>
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
struct iDynTreeRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
    }

    iDynTree::Transform world_H_base;
    iDynTree::VectorDynSize jointPos;
    iDynTree::Twist         baseVel;
    iDynTree::VectorDynSize jointVel;
    iDynTree::Vector3       gravity;
};

struct EigenRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
    }

    void random()
    {
        baseAcc.setRandom();
        jointAcc.setRandom();
    }

    Eigen::Matrix<double,6,1> baseAcc;
    Eigen::VectorXd jointAcc;
};

struct iDynTreeRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
    }

    iDynTree::Vector6 baseAcc;
    iDynTree::VectorDynSize jointAcc;
};



// GLOBAL VARIABLES
iDynTree::KinDynComputations kinDynComp;
EigenRobotState eigRobotState;
int DOFsize = 7;
iDynTreeRobotState idynRobotState;
Eigen::Vector3d com;
Eigen::Vector3d handPositionR_Abs;
Eigen::Vector3d handPositionR_Rel;
Eigen::Vector3d handOrientationR;
Eigen::Vector3d handPositionL_Abs;
Eigen::Vector3d handPositionL_Rel;
Eigen::MatrixXd eigMassMatrix;
Eigen::MatrixXd eigCOMJacobian;
iDynTree::FreeFloatingGeneralizedTorques gGF;

Eigen::MatrixXd eigJacobian;
Eigen::MatrixXf JacobTrasp;
Eigen::MatrixXf Jacob;

EigenRobotAcceleration eigRobotAcc;
iDynTreeRobotAcceleration idynRobotAcc;
Eigen::VectorXd jntTorques;
Eigen::VectorXd gravityCompensation;



void computeDynamics(sensor_msgs::JointState &robotstatus){
   // std::cout<<"entrato"<<std::endl;
    const iDynTree::Model & model = kinDynComp.model();
    eigRobotState.resize(model.getNrOfDOFs());
    eigRobotState.random(); //first I write the random state and then I overwrite the variables that I know. The others I should ask.
    //std::cout<<"Posizione giunti"<<std::endl;
    for(unsigned int i =0; i < DOFsize; i++ ) {
        eigRobotState.jointPos(i)=robotstatus.position[i];
        eigRobotState.jointVel(i)=robotstatus.velocity[i]; 
       // eigRobotState.baseVel (i)=0;
    }
    idynRobotState.resize(model.getNrOfDOFs());
    iDynTree::fromEigen(idynRobotState.world_H_base,eigRobotState.world_H_base);
    iDynTree::toEigen(idynRobotState.jointPos) = eigRobotState.jointPos;
    iDynTree::fromEigen(idynRobotState.baseVel,eigRobotState.baseVel);
    toEigen(idynRobotState.jointVel) = eigRobotState.jointVel;
    toEigen(idynRobotState.gravity)  = eigRobotState.gravity;


    kinDynComp.setRobotState(idynRobotState.world_H_base,idynRobotState.jointPos,
                             idynRobotState.baseVel,idynRobotState.jointVel,idynRobotState.gravity);
    // Once we called the setRobotState, we can call all the methods of KinDynComputations
    // For methods returning fixed size vector/matrices, the conversion to eigen types is trivial
    com = iDynTree::toEigen(kinDynComp.getCenterOfMassPosition());

    //Computation of the position and the orientation of the endEffector (hand) respect to "torso" or "RShp"<- da controllare
    iDynTree::FrameIndex arbitraryFrameIndex = model.getFrameIndex("l_hand_lower_left_link");
   // std::cout<<model.getNrOfDOFs()<<std::endl;
    Eigen::Matrix4d world_H_arbitraryFrame = iDynTree::toEigen(kinDynComp.getWorldTransform(arbitraryFrameIndex).asHomogeneousTransform());
    Eigen::Matrix4d arbitraryFrame_H_anotherArbitraryFrame = iDynTree::toEigen(kinDynComp.getRelativeTransform("torso","l_hand_lower_left_link").asHomogeneousTransform());
   // std::cout<<"Posizione della mano ASSOLUTE"<<std::endl;
    for(unsigned int i =0; i < 3; i++ ) {
    handPositionR_Abs[i]=world_H_arbitraryFrame(i,3);
    //std::cout<<handPositionR_Abs[i]<<std::endl;
    }
   // std::cout<<"Posizione della mano RELATIVE"<<std::endl;
    for(unsigned int i =0; i < 3; i++ ) {
    handPositionR_Rel[i]=arbitraryFrame_H_anotherArbitraryFrame(i,3);
  //  std::cout<<handPositionR_Rel[i]<<std::endl;
    }

    // More complex quantities (such as jacobians and matrices) need to be handled in a different way for efficency reasons
    iDynTree::FreeFloatingMassMatrix idynMassMatrix(model);
    kinDynComp.getFreeFloatingMassMatrix(idynMassMatrix);
    eigMassMatrix = iDynTree::toEigen(idynMassMatrix);// eigRobotState.jointPos=robotstatus.angles; // NOTA!!!!->// eigRobotState.jointPos=robotstatus.angles; // NOTA!!!!->da rimuovere ed aggiornare con il nuovo stato del robotda rimuover// eigRobotState.jointPos=robotstatus.angles; // NOTA!!!!->da rimuovere ed aggiornare con il nuovo stato del robote ed aggiornare con il nuovo stato del robot
    iDynTree::MatrixDynSize idynCOMJacobian(3,model.getNrOfDOFs()+6);
    kinDynComp.getCenterOfMassJacobian(idynCOMJacobian);
    eigCOMJacobian = iDynTree::toEigen(idynCOMJacobian);
   // std::cout<<"colonne COMJACOBIAN"<<eigCOMJacobian.cols()<<std::endl;

     const iDynTree::FrameIndex FI = arbitraryFrameIndex;
     // iDynTree::MatrixDynSize idynJacobian(6,model.getNrOfDOFs()+6);
    iDynTree::FrameFreeFloatingJacobian idynJacobian(model);
    bool j = kinDynComp.getFrameFreeFloatingJacobian(FI, idynJacobian);
    gGF.resize(model);
    kinDynComp.generalizedGravityForces(gGF);

   
     eigJacobian = iDynTree::toEigen(idynJacobian);
     Jacob = eigJacobian.cast <float> ();


      JacobTrasp = eigJacobian.cast <float> ();
      JacobTrasp.transposeInPlace();


    eigRobotAcc.resize(model.getNrOfDOFs());
    eigRobotAcc.random();
    //Acceleration to 0 both the joint and the base- Maybe after you can find another solution
    for(unsigned int i =0; i < DOFsize; i++ ) {
        eigRobotAcc.jointAcc(i)= 0; // it is zero in order to obtain just the compensation of gravity 
       // eigRobotAcc.baseAcc(i)=0;
    }
    idynRobotAcc.resize(model.getNrOfDOFs());
    iDynTree::toEigen(idynRobotAcc.baseAcc) = eigRobotAcc.baseAcc;
    iDynTree::toEigen(idynRobotAcc.jointAcc) = eigRobotAcc.jointAcc;

    // In input to the inverse dynamics we also have external forces (that we assume set to zero)
    iDynTree::LinkNetExternalWrenches extForces(model);
    extForces.zero(); // here you can create a plug in to pass the interaction forces???

    // The output is a set of generalized torques (joint torques + base wrenches)
    iDynTree::FreeFloatingGeneralizedTorques invDynTrqs(model);

    kinDynComp.inverseDynamics(idynRobotAcc.baseAcc,idynRobotAcc.jointAcc,extForces,invDynTrqs);

    // The output of inv dynamics can be converted easily to eigen vectors
    Eigen::Matrix<double,6,1> baseWrench = iDynTree::toEigen(invDynTrqs.baseWrench());
    jntTorques = iDynTree::toEigen(invDynTrqs.jointTorques());

}




int main(int argc, char **argv)
{
	ros::init (argc, argv, "force_controller");
	ROS_INFO("Starting program...");

	// Add iDynTree variables 

    iDynTree::VectorDynSize q, dq, ddq;
    iDynTree::SpatialAcc gravity;
    iDynTree::MatrixDynSize jac;
    iDynTree::Position pCOM;


		
	// Initialize the action server
	ros::NodeHandle n("");

	//ros::spin();
	ros::Rate r(10);

	iDynTree::ModelLoader mdlLoader;
	std::string modelFile = "a string with the urdf information";

    bool ok = mdlLoader.loadModelFromFile(modelFile);


    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        //return EXIT_FAILURE;
    }

    // Create a KinDynComputations class from the model
    ok = kinDynComp.loadRobotModel(mdlLoader.model());

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        //return EXIT_FAILURE;
    }

	while (ros::ok())
	{

		ros::spinOnce();
		r.sleep();
	} 
		// ros::waitForShutdown();
	return 0;	
}






