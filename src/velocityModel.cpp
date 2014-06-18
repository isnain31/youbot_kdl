#include <iostream>
#include "youbot_kdl/KDLInterface.h"

KDL::JntArray armVelocities(5);

void setValues()
{
	for(unsigned int i=0;i<5;i++){
		float myinput;
		printf ("Enter the velocity for joint %i: ",i);
		scanf ("%e",&myinput);
		armVelocities(i)=(double)myinput;
    	}
	
}


int main ()
{
	HardwareEncapsulation::KDLInterface *interface = new HardwareEncapsulation::KDLInterface();
	
	//set values to joints [0 to 5]
	setValues();
	interface->SetArmVelocities(armVelocities);
	//interface->GetArmParameters();
	return 0;
}
