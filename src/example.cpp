#include <iostream>
#include "youbot_kdl/KDLInterface.h"

KDL::JntArray armpostions(5);

void setValues(const float values[])
{
	armpostions(0) =   values[0]; //0.0100692 rad and 5.84014 rad
	armpostions(1) =    values[1]; //(0.0100692 rad and 2.61799 rad)
	armpostions(2) =  values[2]; //-5.02655 rad and -0.015708 rad
	armpostions(3) =  values[3];  //0.0221239 rad and 3.4292
	armpostions(4) =  values[4];//0.110619 rad and 5.64159 rad
	
}


int main ()
{
	HardwareEncapsulation::KDLInterface *interface = new HardwareEncapsulation::KDLInterface();
	KDL::JntArray jnt = KDL::JntArray(5);
	//set values to joints [0 to 5]
	float values[] = {2.96244,1.04883,-2.43523,1.73184,2.91062};
	//float values[] = {0.34035644,2.41799,-4.0935881,2.72755,1.40347};

/*		jointSetAngle[0].angle =  jointpositions() * radian;
		jointSetAngle[1].angle = 1.04883 * radian;
		jointSetAngle[2].angle =-2.43523* radian;
		jointSetAngle[3].angle = 1.73184  * radian;
		jointSetAngle[4].angle =  2.91062 * radian; */
	setValues(values);
	interface->SetArmPosition(armpostions);
	
	return 0;
}
