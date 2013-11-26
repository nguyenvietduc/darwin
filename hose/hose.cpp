/***********
  Desc:: 	DARwIn Upperbody trajectory for firehose handling
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

// === Init Pos for Arms ===
#define RSP_INIT 1024
#define RSR_INIT 1536
#define REP_INIT 1040

#define LSP_INIT 3072
#define LSR_INIT 2560
#define LEP_INIT 3030

// Namespace
using namespace Robot;

// Global varibables
int darwin_joints[11];
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

// Function prototypes
void interpolate(int[], int[], int, int, int[]);

void getCurrentRightArmPos(int[]);
void moveRightArm(int[], int[], int);
void moveRightArmPosAbsolute(int[]);
void gotoInitRightArmPos(int[], int);

void getCurrentLeftArmPos(int[]);
void moveLeftArm(int[], int[], int);
void moveLeftArmPosAbsolute(int[]);
void gotoInitLeftArmPos(int[], int);

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
  
  int delta = MAX;
  int i;
  
  // Read current positions of right arm joints
  int rightArm[3] = {0,0,0};
  getCurrentRightArmPos(rightArm);
  usleep(20000);

  // Read current positions of right arm joints
  int leftArm[3] = {0,0,0};
  getCurrentLeftArmPos(leftArm);
  usleep(20000);

  // Goto Init Position of the arm
  // FIXME DANGER: This code assume the current position to be "right"
  // "Right" means the arm is not currently behind the pitch joint's dynamixel
  gotoInitRightArmPos(rightArm, 100);
  gotoInitLeftArmPos(leftArm, 100);
  usleep(20000);
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

void gotoInitRightArmPos(int rightArm[], int duration)
{
  int delta[3] = {0,0,0};

  // Delta Angles
  delta[0] = RSP_INIT - rightArm[0];
  delta[1] = RSR_INIT - rightArm[1];
  delta[2] = REP_INIT - rightArm[2];

  moveRightArm(rightArm, delta, duration);
}

void moveRightArm(int init[], int delta[], int duration)
{
  int current_pos[3] = {0,0,0};
  int current_time = 0;

  while (current_time < duration)
  {
    interpolate(init, delta, duration, current_time, current_pos);
    moveRightArmPosAbsolute(current_pos);
    current_time++;
    usleep(SIM_TIME_U);
  }
}

void gotoInitLeftArmPos(int leftArm[], int duration)
{
  int delta[3] = {0,0,0};

  // Delta Angles
  delta[0] = LSP_INIT - leftArm[0];
  delta[1] = LSR_INIT - leftArm[1];
  delta[2] = LEP_INIT - leftArm[2];

  moveLeftArm(leftArm, delta, duration);
}

void moveLeftArm(int init[], int delta[], int duration)
{
  int current_pos[3] = {0,0,0};
  int current_time = 0;
  
  while (current_time < duration)
  {
    interpolate(init, delta, duration, current_time, current_pos);
    moveLeftArmPosAbsolute(current_pos);
    current_time++;
    usleep(SIM_TIME_U);
  }
}

void interpolate(int arm[], int delta[], int duration, int current_time, int current_pos[])
{
  double tmp;
  for (int i = 0; i < 3; i++)
  {
    tmp = 0.5 * (1.0 - cos(PI * current_time / duration)) * delta[i];
    current_pos[i] = (int) (arm[i] + tmp + 0.5);
  }
}

void moveRightArmPosAbsolute(int current_pos[])
{
  cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, current_pos[0], 0);
  cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, current_pos[1], 0);
  cm730.WriteWord(JointData::ID_R_ELBOW, MX28::P_GOAL_POSITION_L, current_pos[2], 0);
}

void moveLeftArmPosAbsolute(int current_pos[])
{
  cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, current_pos[0], 0);
  cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, current_pos[1], 0);
  cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, current_pos[2], 0);
}