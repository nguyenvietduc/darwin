/***********
  Desc:: 	Tank-DARwIn Upperbody Joint Angle Reader
  Author::	Duc

***********/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "LinuxDARwIn.h"

#define MAX 100
#define STEP 2
#define SIM_TIME_U 10000
#define PI 3.14159265

#define U2D_DEV_NAME0 "/dev/ttyUSB0"

// === Software limit for arms ===
#define RSP_MIN 1024
#define RSP_MAX 3072
#define RSR_MIN 1536
#define RSR_MAX 3584
#define REP_MIN 1040
#define REP_MAX 2400

#define LSP_MIN 1024
#define LSP_MAX 3072
#define LSR_MIN 512
#define LSR_MAX 2560
#define LEP_MIN 1660
#define LEP_MAX 3030

// === Software limit for pitch joint
#define RHR_MIN 1856
#define RHR_MAX 2256
#define LHR_MIN 1872
#define LHR_MAX 2272

// === Init Pos for Arms ===
#define RSP_INIT 1024
#define RSR_INIT 1536
#define REP_INIT 1040

#define LSP_INIT 3072
#define LSR_INIT 2560
#define LEP_INIT 3030

// === Init Pos for Pitch Joint ===
#define RHR_INIT 2056
#define LHR_INIT 2072

// Namespace
using namespace Robot;

// Global varibables
int darwin_joints[11];
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

// Define trajectory here
int lsp_traj[] = { 0, 1024, -200, -700 };
int lsr_traj[] = { -512, 0, -570, 0 };
int lep_traj[] = { -128, 0, 0, 0 };

// Function prototypes
void getCurrentRightArmPos(int[]);
void getCurrentLeftArmPos(int[]);
void getCurrentPitchPos(int[]);

int main()
{

  printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");
  
  //////////////////// Framework Initialize ////////////////////////////
  // LinuxCM730 linux_cm730("/dev/ttyUSB0");
  // CM730 cm730(&linux_cm730);
  if(cm730.Connect() == false)
  {
    printf("Fail to connect CM-730!\n");
    return 0;
  }
  
  /////////////////////////////////////////////////////////////////////
  
  cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);
  
  cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

  cm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_TORQUE_ENABLE, 0, 0);

  /////////////////////////////////////////////////////////////////////

  getchar();
  
  // Read current positions of upper body joints
  int arms[8] = {0,0,0,0,0,0,0,0};
  int rightArm[3] = {0,0,0};
  int leftArm[3] = {0,0,0};

  while (1) {
    printf("\r");
    getCurrentRightArmPos(arms);
    getCurrentLeftArmPos(arms + 3);
    getCurrentPitchPos(arm + 6);
    usleep(20000);
    
    for (int i = 0; i<8; i++)
      printf("ID %d - %d  - ", i, arms[i]);
  }

  getchar();

}

void getCurrentRightArmPos(int rightArm[])
{
  int value;
  if (cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    rightArm[0] = value;
  }
  
  if (cm730.ReadWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    rightArm[1] = value;
  }
  
  if (cm730.ReadWord(JointData::ID_R_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    rightArm[2] = value;
  }
}

void getCurrentLeftArmPos(int leftArm[])
{
  int value;
  if (cm730.ReadWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    leftArm[0] = value;
  }
  
  if (cm730.ReadWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    leftArm[1] = value;
  }
  
  if (cm730.ReadWord(JointData::ID_L_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    leftArm[2] = value;
  }
}

void getCurrentPitchPos(int pitch[])
{
  int value;
  if (cm730.ReadWord(JointData::ID_R_HIP_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    pitch[0] = value;
  }
  
  if (cm730.ReadWord(JointData::ID_L_HIP_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
    pitch[1] = value;
  }
}
