#ifndef KDL_INTERFACE_H_
#define KDL_INTERFACE_H_

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <vector>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "youbot/DataTrace.hpp"


namespace HardwareEncapsulation {

class KDLInterface {

private:
bool checkJointLimits(KDL::JntArray);

//JointAngleSetpoint desiredJointAngle;
public:
KDLInterface();
youbot::YouBotManipulator *youBotArm;
youbot::JointAngleSetpoint desiredJointAngle;
KDL::JntArray armAngle;
KDL::JntArray armVelocity;
KDL::JntArray armTorque;
bool SetArmPosition(const KDL::JntArray &);
bool SetArmVelocities(const KDL::JntArray &);
bool GetArmParameters();
bool fkSolver();
bool ikSolver();

virtual ~KDLInterface();
};

}

#endif 
