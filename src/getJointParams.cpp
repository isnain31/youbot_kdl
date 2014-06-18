#include <iostream>
#include "youbot_kdl/KDLInterface.h"

KDL::JntArray armVelocities(5);

int main ()
{
	HardwareEncapsulation::KDLInterface *interface = new HardwareEncapsulation::KDLInterface();
	
	//set values to joints [0 to 5]
	//setValues();
	//interface->SetArmVelocities(armVelocities);
	interface->GetArmParameters();
	return 0;
}
