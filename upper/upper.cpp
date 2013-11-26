#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"

#define MAX 100
#define STEP 2
#define U_NAP 20000

// === SOFTWARE LIMIT FOR PITCH JOINT ===
#define HIP_MIN 1848
#define HIP_MAX 2248

// === SOFTWARE LIMIT FOR ARMS ===
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

// Devices
#define U2D_DEV_NAME0	"/dev/ttyUSB0"

using namespace Robot;

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

//using namespace Robot;

// === Function prototypes ===
void MoveJoint(int, int, int);
void MoveJoint(int, int);
void init(void);
void initLeftArm(void);
void initRightArm(void);
void initHipPitch(void);

int main()
{
    printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

    //////////////////// Framework Initialize ////////////////////////////
    //linux_cm730("/dev/ttyUSB0");
    //cm730(&linux_cm730);
    if(cm730.Connect() == false)
    {
        printf("Fail to connect CM-730!\n");
        return 0;
    }
    /////////////////////////////////////////////////////////////////////

    int value1, value2;
    cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_L_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

    cm730.WriteByte(JointData::ID_R_SHOULDER_PITCH, MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_R_SHOULDER_ROLL,  MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_R_ELBOW,          MX28::P_P_GAIN, 8, 0);
/*

    while(1)
    {
      printf("\r");
      printf("ID[%d]:", JointData::ID_R_SHOULDER_PITCH);
      if (cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value1, 0) == CM730::SUCCESS)
      {
	printf("%4d", value1);
	
      }
      else
      {
	printf("----");
      }
      
      printf(" ID[%d]:", JointData::ID_R_SHOULDER_ROLL);
      if (cm730.ReadWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value1, 0) == CM730::SUCCESS)
      {
	printf("%4d", value1);
	
      }
      else
      {
	printf("----");
      }
      
      printf(" ID[%d]:", JointData::ID_R_ELBOW);
      if (cm730.ReadWord(JointData::ID_R_ELBOW, MX28::P_PRESENT_POSITION_L, &value1, 0) == CM730::SUCCESS)
      {
	printf("%4d", value1);
	
      }
      else
      {
	printf("----");
      }

      usleep(50000);
    }

*/

    init();


}

void init()
{
  initLeftArm();
  initRightArm();
  initHipPitch();
}

void initLeftArm()
{

}

void initRightArm()
{
}

void initHipPitch()
{

}

void initLeftArm()
{
  int value[] = {0, 0, 0};

  printf(" ID[%d]:", JointData::ID_R_SHOULDER_PITCH);
  if (cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value[0], 0) == CM730::SUCCESS)
  {
    printf("%4d", value);
  }
  
  printf(" ID[%d]:", JointData::ID_R_SHOULDER_PITCH);
  if (cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value[0], 0) == CM730::SUCCESS)
  {
    printf("%4d", value);
  }
  
  printf(" ID[%d]:", JointData::ID_R_SHOULDER_PITCH);
  if (cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value[0], 0) == CM730::SUCCESS)
  {
    printf("%4d", value);
  }
}

void returnToinitPost(int side)
{
  if (side = 1)
  {
  }
}

/*
void MoveJoint(int[] id, int address, int delta)
{
  int i = 1;
  if (delta < 0)
  {
    i = -1;
    delta = -delta;
  }

  while (delta > 0)
  {

    delta--;
  }
}

void MoveJoint(int id, int delta)
{
  int ids = { id };
  MoveJoint(ids, MX28::P_GOAL_POSITION_L, delta);
}*/
