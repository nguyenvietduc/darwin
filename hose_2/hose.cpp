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

// Define trajectory here
int lsp_traj[] = { 0, 1024, -200, -700 };
int lsr_traj[] = { -512, 0, -570, 0 };
int lep_traj[] = { -128, 0, 0, 0 };

// Function prototypes
void interpolate(int[], int[], int, int, int[], int);

void getCurrentRightArmPos(int[]);
void moveRightArm(int[], int[], int);
void moveRightArmPosAbsolute(int[]);
void gotoInitRightArmPos(int[], int);

void getCurrentLeftArmPos(int[]);
void moveLeftArm(int[], int[], int);
void moveLeftArmPosAbsolute(int[]);
void gotoInitLeftArmPos(int[], int);

void moveBothArms(int[], int[], int);
void moveBothArmsPosAbsolute(int[]);
void gotoInitBothArmsPos(int[], int);

void hoseHandling(void);

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
  
  int i;
  
  // Read current positions of right arm joints
  int arms[6] = {0,0,0,0,0,0};

  int rightArm[3] = {0,0,0};
  getCurrentRightArmPos(arms);
  usleep(20000);

  // Read current positions of right arm joints
  int leftArm[3] = {0,0,0};
  getCurrentLeftArmPos(arms + 3);
  usleep(20000);

  /*
  for (i = 0; i<6; i++)
    printf("ID %d - %d\n", i, arms[i]);

  int len = (sizeof(arms) / sizeof(*arms));
  printf("%d\n", len);
  */

  getchar();

  // Goto Init Position of the arm
  // FIXME DANGER: This code assume the current position to be "right"
  // "Right" means the arm is not currently behind the pitch joint's dynamixel
  // gotoInitRightArmPos(rightArm, 100);
  // gotoInitLeftArmPos(leftArm, 100);

  // Goto init position for both arms
  gotoInitBothArmsPos(arms, 100);

  hoseHandling();

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
    interpolate(init, delta, duration, current_time, current_pos, 3);
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
    interpolate(init, delta, duration, current_time, current_pos, 3);
    moveLeftArmPosAbsolute(current_pos);
    current_time++;
    usleep(SIM_TIME_U);
  }
}

void gotoInitBothArmsPos(int arms[], int duration)
{
  int delta[6] = {0,0,0,0,0,0};

  // Delta Angles
  delta[0] = RSP_INIT - arms[0];
  delta[1] = RSR_INIT - arms[1];
  delta[2] = REP_INIT - arms[2];
  delta[3] = LSP_INIT - arms[3];
  delta[4] = LSR_INIT - arms[4];
  delta[5] = LEP_INIT - arms[5];

  moveBothArms(arms, delta, duration);
}

void moveBothArms(int init[], int delta[], int duration)
{
  int current_pos[6] = {0,0,0,0,0,0};
  int current_time = 0;
  
  while (current_time < duration)
  {
    interpolate(init, delta, duration, current_time, current_pos, 6);
    moveBothArmsPosAbsolute(current_pos);
    current_time++;
    usleep(SIM_TIME_U);
  }
}

void interpolate(int arm[], int delta[], int duration, int current_time, int current_pos[], int len)
{
  double tmp;
  for (int i = 0; i < len; i++)
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

void moveBothArmsPosAbsolute(int current_pos[])
{
  cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, current_pos[0], 0);
  cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, current_pos[1], 0);
  cm730.WriteWord(JointData::ID_R_ELBOW, MX28::P_GOAL_POSITION_L, current_pos[2], 0);
  cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, current_pos[3], 0);
  cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, current_pos[4], 0);
  cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, current_pos[5], 0);
}

void hoseHandling()
{
  // Step 1
  int step = 0;
  int total = (sizeof(lsp_traj) / sizeof(*lsp_traj));

  printf("%d\n", total);
  getchar();

  int arms[6] = {0,0,0,0,0,0};

  for (step = 0; step < total; step++)
  {
    printf("Step %d\n", step + 1);
    getCurrentRightArmPos(arms);
    getCurrentLeftArmPos(arms + 3);
    int delta[] = {0,0,0, lsp_traj[step], lsr_traj[step], lep_traj[0]};
    moveBothArms(arms, delta, 100);
    getchar();
  }

  getCurrentRightArmPos(arms);
  getCurrentLeftArmPos(arms + 3);
  gotoInitBothArmsPos(arms, 100);


/*
  printf("Step 1\n");
  getchar();
  getCurrentRightArmPos(arms);
  getCurrentLeftArmPos(arms + 3);
  int delta[6] = {0,0,0,0,lsr_traj[0],0};
  moveBothArms(arms, delta, 100);

  // Step 2
  printf("Step 2\n");
  getchar();
  getCurrentRightArmPos(arms);
  getCurrentLeftArmPos(arms + 3);
  delta[4] = lsr_traj[1];
  moveBothArms(arms, delta, 100);
*/
}