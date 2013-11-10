/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 *	Bharadwaj Ramesh
 */
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include <termios.h>
#include <term.h>
#include <pthread.h>

#include "minIni.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "Action.h"
#include "Head.h"

#include "MX28.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "LinuxCM730.h"
#include "LinuxActionScript.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

// defining trajectory points

float xp[16]           = {  0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10, 11, 12, 13, 14,15};

float shoulderPR_loc[] = {  0,   0,  20,  20,  55,  55,   0,   0,  90,  90,  90,   0,   0,   0,   0,   0};
float shoulderRR_loc[] = {-40, -40, -40, -40, -40, -40, -40, -40, -40, -65, -40, -40, -40, -40, -40, -40};
float elbowR_loc[]     = { 90,  90,  90,  90,  35,  35,  90,  90, 100,   0,  90,  90,  90,  90,  90,  90};
float fingerR_loc[]    = { 30, -20, -20,  30,  30, -20, -20,  30,  30,  30,  30,  30,  30,  30,  30,  30};

float shoulderPL_loc[] = {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, -45,   0, -45,   0};
float shoulderRL_loc[] = { 40,  40 , 40,  40,  40,  40,  40,  40,  40,  40,  40,  40,   0,  40,  40,  40};
float elbowL_loc[]     = { 90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  45,  90,  45,  90};
float fingerL_loc[]    = {-20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20};


float x1=0; // for right hand trajectories

float shoulderPR=0;
float shoulderRR=0;
float elbowR=0;
float fingerR=0;

float shoulderPL=0;
float shoulderRL=0;
float elbowL=0;
float fingerL=0;

float currStep = 1;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    exit(0);
}

// the linear interpolation function
float lip(float x, float xp[],float yp[])
{
	float y = 0;
	float xp0 = 0;
	float yp0 = 0;
	float xp1 = 0;
	float yp1 = 0;
	for(int i = 0;i<=20;i++)
	{
		if(xp[i]<=x)
		{
			xp0 = xp[i];
			yp0 = yp[i];
		}
		if(xp[i]>x)
		{
			xp1 = xp[i];
			yp1 = yp[i];
			break;
		}
	}
	float m = (yp1- yp0)/(xp1-xp0);
	y = (yp0 + (m*(x-xp0)));
	return (y);
}

void GraspingR(void);
void GraspingL(void);
void StretchR(void);
void StretchL(void);
void StepUp(void);

int main(void)
{
//	int i = 0;
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

   
    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    
    MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0;
    if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if(0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
#endif
    }
    else if(27 <= firm_ver)
    {
#ifdef MX28_1024
        fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
        fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
        exit(0);
#else
        Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
#endif
    }
    else
        exit(0);

	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	MotionManager::GetInstance()->SetEnable(true);
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	Walking::GetInstance()->m_Joint.SetEnableLowerBody(true,true);

	while(Action::GetInstance()->IsRunning()) usleep(8*1000);
	// initializing all my joints to my initial position

	Action::GetInstance()->m_Joint.SetAngle(1,0);
	Action::GetInstance()->m_Joint.SetAngle(3,-40);
	Action::GetInstance()->m_Joint.SetAngle(5,90);
	Action::GetInstance()->m_Joint.SetAngle(23,30);

	//left trajectories
	Action::GetInstance()->m_Joint.SetAngle(2,0);
	Action::GetInstance()->m_Joint.SetAngle(4,40);
	Action::GetInstance()->m_Joint.SetAngle(6,90);
	Action::GetInstance()->m_Joint.SetAngle(24,-20);

	Action::GetInstance()->m_Joint.SetAngle(19,0);
	Action::GetInstance()->m_Joint.SetAngle(20,0);
	std::cout << "Press any Key to start \n";
	getchar();
	sleep(1);
	std::cout << "Step 1: opening the door \n";
	while(1)
	{
		if(currStep ==1)
		{x1 = x1+0.0005;
		 if(x1>=6.9995)
			{currStep=2;
			std::cout << "Step 2 : walking to door \n";}
		}

		else if (currStep ==2)
		{
		 Walking::GetInstance()->X_MOVE_AMPLITUDE = 10.0;
		 Walking::GetInstance()->PERIOD_TIME = 3000.0;
		 Walking::GetInstance()->Start();
 		 sleep(3);
		 Walking::GetInstance()->Stop();
		 sleep(2);
		 currStep = 3;
		std::cout << "Step 3 : Pushing with the right hand \n";
		}

		else if (currStep ==3)
		{
		x1 = x1+0.0005;
		 if(x1>=10.9995)
			{currStep=5;
			std::cout << "Step 5 : walking towards door \n";}
		}
		else if (currStep ==5)
		{
		 Walking::GetInstance()->X_MOVE_AMPLITUDE = 7.5;
		 Walking::GetInstance()->PERIOD_TIME = 3000.0;
		 Walking::GetInstance()->Start();
 		 sleep(2);
		 Walking::GetInstance()->Stop();
		 sleep(3);
		 currStep = 6;
		std::cout << "Step 6: opening with left hand \n";
		}
		else if (currStep == 6)
		{
		x1 = x1+0.0005;
		 if(x1>=12.9995)
			{currStep=8;
			std::cout << "Step 8: walking forward , turn left\n";}
		}
		else if (currStep == 8)
		{
		 Walking::GetInstance()->A_MOVE_AMPLITUDE = 10.0;
		 Walking::GetInstance()->PERIOD_TIME = 3000.0;
		 Walking::GetInstance()->Start();
 		 sleep(5);
		 Walking::GetInstance()->Stop();
		 sleep(2);


		 Walking::GetInstance()->X_MOVE_AMPLITUDE = 10.0;
		 Walking::GetInstance()->PERIOD_TIME = 3000.0;
		 Walking::GetInstance()->Start();
 		 sleep(6);
		 Walking::GetInstance()->Stop();
		 sleep(3);
		 currStep = 12;
		 x1 = 7;
		std::cout << " Step  : walking out  \n";
		}
		else if (currStep == 12)
		{
		 Walking::GetInstance()->X_MOVE_AMPLITUDE = 10.0;
		 Walking::GetInstance()->PERIOD_TIME = 3000.0;
		 Walking::GetInstance()->Start();
 		 sleep(5);
		 Walking::GetInstance()->Stop();
		 sleep(3);
		 currStep = 13;
		}
		else if (currStep == 13)
			break;
		// generating the right hand trajectories
		shoulderPR =  lip(x1,xp,shoulderPR_loc);
		shoulderRR =  lip(x1,xp,shoulderRR_loc);
		elbowR =  lip(x1,xp,elbowR_loc);
		fingerR =  lip(x1,xp,fingerR_loc);
		// generating the left hand trajectories
		shoulderPL =  lip(x1,xp,shoulderPL_loc);
		shoulderRL =  lip(x1,xp,shoulderRL_loc);
		elbowL =  lip(x1,xp,elbowL_loc);
		fingerL =  lip(x1,xp,fingerL_loc);

		usleep(1000);
//		std::cout<< "shoulderPL :" << shoulderPL <<"\n";
//		std::cout<< "shoulderRL :" << shoulderRL <<"\n";
//		std::cout<< "elbowL :" << elbowL <<"\n";
//		std::cout<< "fingerL :" << fingerL <<"\n";

		//sending the generated trajectory , right hand
		Action::GetInstance()->m_Joint.SetAngle(1,shoulderPR);
		Action::GetInstance()->m_Joint.SetAngle(3,shoulderRR);
		Action::GetInstance()->m_Joint.SetAngle(5,elbowR);
		Action::GetInstance()->m_Joint.SetAngle(23,fingerR);

		//left trajectories
		Action::GetInstance()->m_Joint.SetAngle(2,shoulderPL);
		Action::GetInstance()->m_Joint.SetAngle(4,shoulderRL);
		Action::GetInstance()->m_Joint.SetAngle(6,elbowL);
		Action::GetInstance()->m_Joint.SetAngle(24,fingerL);

	}

	return 0;
} 