#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"


using namespace Robot;

int main()
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

	int value;
    cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

    cm730.WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_L_ELBOW,          MX28::P_P_GAIN, 8, 0);

    // Upperbody manipulation

    

	while(1)
	{
		printf("\r");

		printf(" ID[%d]:", JointData::ID_R_SHOULDER_PITCH);
		if(cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			printf("%4d", value);
			cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
		}
		else
			printf("----");

		/*

		printf(" ID[%d]:", JointData::ID_R_SHOULDER_ROLL);
		if(cm730.ReadWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			printf("%4d", value);
			cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
		}
		else
			printf("----");

		printf(" ID[%d]:", JointData::ID_R_ELBOW);
		if(cm730.ReadWord(JointData::ID_R_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			printf("%4d", value);
			cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
		}
		else
			printf("----");

		*/

		/* LED light for head and eyes

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_LED_HEAD_L, &value, 0) == CM730::SUCCESS)
		{
			if(value == 0x7FFF)
				value = 0;
			else
				value++;

			cm730.WriteWord(CM730::P_LED_HEAD_L, value, 0);
		}

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_LED_EYE_L, &value, 0) == CM730::SUCCESS)
		{
			if(value == 0)
				value = 0x7FFF;
			else
				value--;

			cm730.WriteWord(CM730::P_LED_EYE_L, value, 0);
		}
		*/

		usleep(50000);
	}

	return 0;
}
