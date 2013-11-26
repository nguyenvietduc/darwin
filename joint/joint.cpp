#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"

#define MAX 100
#define STEP 2

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

    int value1, value2;
    cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

    cm730.WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_L_ELBOW,          MX28::P_P_GAIN, 8, 0);


    
    
    
    int delta = MAX;
//    while (delta > 0)
  //  {
      printf("\r");
      
      printf("ID[%d]:", JointData::ID_R_HIP_ROLL);
      if (cm730.ReadWord(JointData::ID_R_HIP_ROLL, MX28::P_PRESENT_POSITION_L, &value1, 0) == CM730::SUCCESS)
      {
	printf("%4d", value1);
	
      }
      else
      {
	printf("----");
      }

      printf(" ID[%d]:", JointData::ID_L_HIP_ROLL);
      if (cm730.ReadWord(JointData::ID_L_HIP_ROLL, MX28::P_PRESENT_POSITION_L, &value2, 0) == CM730::SUCCESS)
      {
	printf("%4d", value2);
	printf(" %4d", MX28::GetMirrorValue(value2));
      }
      else
      {
	printf("----");
      }
while(delta > 0) {
      //printf("\nValue: ");
      //std::cin >> delta;
      value1 = value1 + STEP;
      value2 = value2 - STEP;

       cm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L, value1, 0);
       cm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L, value2, 0);
       delta--;
      usleep(20000);
    }

while(delta < MAX*2) {
      //printf("\nValue: ");
      //std::cin >> delta;
      value1 = value1 - STEP;
      value2 = value2 + STEP;

       cm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L, value1, 0);
       cm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L, value2, 0);
       delta++;
      usleep(20000);
    }
delta = MAX;
while(delta > 0) {
      //printf("\nValue: ");
      //std::cin >> delta;
      value1 = value1 + STEP;
      value2 = value2 - STEP;

       cm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L, value1, 0);
       cm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L, value2, 0);
       delta--;
      usleep(20000);
    }


	

}
