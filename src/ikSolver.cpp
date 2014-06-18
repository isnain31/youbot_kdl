#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include "youbot_kdl/KDLInterface.h"


KDL::JntArray armpostions(5);

int main ()
{
   
	HardwareEncapsulation::KDLInterface *interface = new HardwareEncapsulation::KDLInterface();
	interface->ikSolver();	
	return 0;
}
