#include <youbot_kdl/KDLInterface.h>
#include <stdio.h>
#include <iostream>

using namespace youbot;
using namespace KDL;

namespace HardwareEncapsulation {

KDLInterface::KDLInterface() {
	youBotArm = new youbot::YouBotManipulator("youbot-manipulator");	
}

bool KDLInterface::checkJointLimits(KDL::JntArray jointpositions){

	double jointMax[] = {5.840139, 2.617989, -0.0157081, 3.42919, 5.641589};
	double jointMin[] = {0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062};
	
	for(unsigned int i=0;i<jointpositions.rows();i++){
		if(jointpositions(i) > jointMax[i] || jointpositions(i) < jointMin[i])
			return false;
	}
	return true;
}


bool KDLInterface::SetArmPosition(const KDL::JntArray &jointpositions){

	//check for joint limits and command arm position
	if(checkJointLimits(jointpositions))	{
			
		youBotArm->doJointCommutation();
		//youBotArm->calibrateManipulator();
		
		std::vector<youbot::JointAngleSetpoint> jointSetAngle;
		jointSetAngle.resize(5);
		
		for(unsigned int i=0;i<jointpositions.rows();i++){
			jointSetAngle[i].angle =  jointpositions(i) * radian;
		}	

		youBotArm->setJointData(jointSetAngle);
		SLEEP_SEC(5);
		
		
	}
	return true; //after implementation change it to true.
}


bool KDLInterface::SetArmVelocities(const KDL::JntArray &armVelocities){
	std::vector<youbot::JointVelocitySetpoint> jointSetVelocity;
	youBotArm->doJointCommutation();
	youBotArm->calibrateManipulator();
		
	for(unsigned int i=0;i<armVelocities.rows();i++){
		jointSetVelocity[i].angularVelocity=armVelocities(i)*radian_per_second;
	}
	youBotArm->setJointData(jointSetVelocity);
	SLEEP_SEC(5);
	return true;
}

bool KDLInterface::GetArmParameters(){
        JointSensedTorque torque;
	JointSensedVelocity velocity;
	JointSensedAngle angle;
	
	armTorque = KDL::JntArray(5);
	armAngle = KDL::JntArray(5);
	armVelocity = KDL::JntArray(5);


	std::cout << "Retrieving Arm Parameters" << std::endl;

	for(unsigned int i=0;i<5;i++){
		std::cout << "Arm " << (i+1) << std::endl;	
			
		youBotArm->getArmJoint(i+1).getData(torque); // reading torque
		armTorque(i)=torque.torque.value();
		std::cout << "Torque " << armTorque(i) << std::endl;

		youBotArm->getArmJoint(i+1).getData(angle); // reading angle
		armAngle(i)=angle.angle.value();
		std::cout << "Angle " << armAngle(i) << std::endl;

		youBotArm->getArmJoint(i+1).getData(velocity); // reading angular velocity
		armVelocity(i)=velocity.angularVelocity.value();
		std::cout << "Velocity " << armVelocity(i) << std::endl;
		
	}
	return true;	
}


bool KDLInterface::ikSolver(){
   KDL::Tree my_tree;
   KDL::Chain chain;	
   urdf::Model my_model;
   KDL::Vector f;

    KDL::Chain youbot_chain;	
    // creating chain using dh parameter
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.0,0.0,0.072,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.0,-1.57,0.075,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.155,0.0,0.0,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.135,0.0,0.0,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.0,1.57,0.081,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Frame::DH_Craig1989(0.09,0.0,0.0,0.0))); //tool tip
		
   // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(youbot_chain);
    ChainIkSolverVel_pinv iksolver1v(youbot_chain);//Inverse velocity solver
    ChainIkSolverPos_NR iksolver1(youbot_chain,fksolver,iksolver1v,100,1e-6);
	
    // Create joint array
    //Creation of jntarrays:
    JntArray q(youbot_chain.getNrOfJoints());
    JntArray q_init(youbot_chain.getNrOfJoints());
 
    //Set destination frame
//    Frame F_dest = Frame(Rotation::RPY(M_PI/2, 0, 0), Vector(1, 1, 0.0));
Rotation r6(0.383876,0.764533,-0.517811,0.453728,-0.644583,-0.61534,-0.80422,0.00126961,-0.59433);	
	
    Frame F_dest = Frame(r6, Vector( 0.0248988,-0.0866994,0.211844));
    int ret = iksolver1.CartToJnt(q_init, F_dest, q);

    // reads the values in the final jntarras
    for(unsigned int i=0;i<youbot_chain.getNrOfJoints();i++){
        printf ("position of %dth joint is : %lf\n",i,q(i));
    } 
    this->SetArmPosition(q);	
}

bool KDLInterface::fkSolver(){

   KDL::Tree my_tree;
   KDL::Chain chain;	
   urdf::Model my_model;
   KDL::Vector f;

   // loading urdf  
   if (!my_model.initFile("../../../src/kdl_fk/youbot.urdf")){
      std::cout << "Failed to parse urdf robot model";
      return false;
   }

   // creating kdl tree
   if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      std::cout << "Failed to construct kdl tree";
      return false;
   }

   // getting chain from tree
   my_tree.getChain("base_link","gripper_palm_link", chain);
   //chain.addSegment(Segment(Joint(Joint::RotZ)));	 
   
   KDL::Chain youbot_chain;	
    // creating chain using dh parameter
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.0,0.0,0.072,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.0,-1.57,0.075,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.155,0.0,0.0,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.135,0.0,0.0,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),Frame::DH_Craig1989(0.0,1.57,0.081,0.0)));
    youbot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Frame::DH_Craig1989(0.09,0.0,0.0,0.0))); //tool tip
		
   // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    ChainFkSolverPos_recursive youbotfksolver = ChainFkSolverPos_recursive(youbot_chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
    
    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }
 
    // Create the frame that will contain the results
    KDL::Frame cartpos; 
    KDL::Frame youbotcartpos; 
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    kinematics_status = youbotfksolver.JntToCart(jointpositions,youbotcartpos);
    if(kinematics_status>=0){
	std::cout << "Using URDF Model" << "endl" ;
        std::cout << cartpos <<std::endl;
	std::cout << "Using manually derived DH parameter" << "endl" ;
        std::cout << youbotcartpos <<std::endl;
        //printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
  
   return true;
}

KDLInterface::~KDLInterface() {
	delete youBotArm;	
	// TODO Auto-generated destructor stub
}

} /* namespace HardwareEncapsulation */

