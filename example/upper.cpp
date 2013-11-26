/*
 * upper.cpp
 * Author: Bharadwaj Ramesh
 * Email Id : br375@drexel.edu 
 * 
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

    // Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    // MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    // MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

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
// This is where you initialize the managers
	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	MotionManager::GetInstance()->SetEnable(true);
//	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());


// This is where the walking manager with only the lower body utilized for walking is switched on
//	Walking::GetInstance()->m_Joint.SetEnableLowerBody(true,true);

	std::cout << "Just Initialized \n";

	while(Action::GetInstance()->IsRunning()) usleep(8*1000);
// then we get to play with the upper body joints and give them angles.
	Action::GetInstance()->m_Joint.SetAngle(1,-10); // SetAngle(joint #, angle in degrees);
	Action::GetInstance()->m_Joint.SetAngle(2,10);
	Action::GetInstance()->m_Joint.SetAngle(3,-35);
	Action::GetInstance()->m_Joint.SetAngle(4,35);
	Action::GetInstance()->m_Joint.SetAngle(5,90);
	Action::GetInstance()->m_Joint.SetAngle(6,90);
	//Action::GetInstance()->m_Joint.SetAngle(21,0);
	//Action::GetInstance()->m_Joint.SetAngle(22,-180);
	Action::GetInstance()->m_Joint.SetAngle(19,0);
	Action::GetInstance()->m_Joint.SetAngle(20,0);
	std::cout << "Press Enter Key to start \n";
	getchar();
	sleep(1);

//	Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
	//Walking::GetInstance()->PERIOD_TIME = 1800.0;
//	Walking::GetInstance()->Start();
//	sleep(10);
//	Walking::GetInstance()->Stop();
//	sleep(2);
/*
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -10.0;
	Walking::GetInstance()->PERIOD_TIME = 2700.0;
	Walking::GetInstance()->Start();
	sleep(4);
	Walking::GetInstance()->Stop();
	sleep(2);

	Action::GetInstance()->m_Joint.SetAngle(17,0);
	sleep(1);
	std::cout << "Right : At 0 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(17,10);
	sleep(1);
	std::cout << "Right : At 10 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(17,20);
	sleep(1);
	std::cout << "Right : At 20 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(17,30);
	sleep(1);
	std::cout << "Right : At 30 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(17,40);
	sleep(1);
	std::cout << "Right : At 40 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(17,50);
	sleep(1);
	std::cout << "Right : At 50 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(17,60);
	sleep(1);
	std::cout << "Right : At 60 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";




	Action::GetInstance()->m_Joint.SetAngle(18,0);
	sleep(1);
	std::cout << "Left : At 0 my value is :" << Action::GetInstance()->m_Joint.GetValue(18) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(18,10);
	sleep(1);
	std::cout << "Left : At 10 my value is :" << Action::GetInstance()->m_Joint.GetValue(18) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(18,20);
	sleep(1);
	std::cout << "Left : At 20 my value is :" << Action::GetInstance()->m_Joint.GetValue(18) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(18,30);
	sleep(1);
	std::cout << "Left : At 30 my value is :" << Action::GetInstance()->m_Joint.GetValue(18) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(18,40);
	sleep(1);
	std::cout << "Left : At 40 my value is :" << Action::GetInstance()->m_Joint.GetValue(18) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(18,50);
	sleep(1);
	std::cout << "Left : At 50 my value is :" << Action::GetInstance()->m_Joint.GetValue(18) << "\n";

	sleep(1);
	Action::GetInstance()->m_Joint.SetAngle(17,60);
	sleep(1);
	std::cout << "Right : At 60 my value is :" << Action::GetInstance()->m_Joint.GetValue(17) << "\n";*/
//	Action::GetInstance()->m_Joint.SetAngle(18,10);
// Right Hand
//	Action::GetInstance()->m_Joint.SetAngle(1,0);
//	Action::GetInstance()->m_Joint.SetAngle(3,-40);
//	Action::GetInstance()->m_Joint.SetAngle(5,90);
//	Action::GetInstance()->m_Joint.SetAngle(21,0);
//	Action::GetInstance()->m_Joint.SetAngle(23,30);

// Left hand
//	Action::GetInstance()->m_Joint.SetAngle(2,0);
//	Action::GetInstance()->m_Joint.SetAngle(4,40);
//	Action::GetInstance()->m_Joint.SetAngle(6,0);
//	Action::GetInstance()->m_Joint.SetAngle(22,0);
//	Action::GetInstance()->m_Joint.SetAngle(24,-10);

/*
	Action::GetInstance()->m_Joint.SetAngle(23,-20);

	sleep(3);

	Action::GetInstance()->m_Joint.SetAngle(1,20);
	Action::GetInstance()->m_Joint.SetAngle(5,70);

	sleep(3);

	Action::GetInstance()->m_Joint.SetAngle(23,30);

	sleep(3);

	Action::GetInstance()->m_Joint.SetAngle(1,55);
	Action::GetInstance()->m_Joint.SetAngle(5,35);

	sleep(3);*/

//	Walking::GetInstance()->X_MOVE_AMPLITUDE = 10.0;
//	Walking::GetInstance()->PERIOD_TIME = 3000.0;
//	Walking::GetInstance()->Start();
//	sleep(20);
//	Walking::GetInstance()->Stop();
//	sleep(2);
//
//	Action::GetInstance()->m_Joint.SetAngle(1,45);
//	Action::GetInstance()->m_Joint.SetAngle(5,45);
//	sleep(1);
//	Action::GetInstance()->m_Joint.SetAngle(23,0);
//	sleep(2);
//	Action::GetInstance()->m_Joint.SetAngle(23,-30);
//	sleep(2);
	std::cout<< "Bye \n";
	//sleep(5);
	return 0;
}  // Main function end

