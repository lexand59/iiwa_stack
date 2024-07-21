/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:
  
  QueryDevice.c

Description:

  This example demonstrates how to retrieve information from the haptic device.

*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
#include <time.h>
# include "conio.h"
# include <string.h>
#define FALSE 0
#define TRUE 1
#endif

#include <ros/ros.h>
#include <iostream>
#include <iomanip> // Для std::fixed и std::setprecision
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <cstring>
#include <geometry_msgs/Point.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <math.h>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

/* Holds data retrieved from HDAPI. */
struct DeviceData 
{
    HDboolean m_buttonState;       /* Has the device button has been pressed. */
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
    hduVector3Dd gimbalAngles;
    hduVector3Dd jointAngles;
    HDErrorInfo m_error;
};

/* Synchronization structure. */ 
struct DeviceStateStruct
{
    HDdouble forceValues[3];
    HDdouble jointTorqueValues[3];   
    HDdouble gimbalTorqueValues[3];   
};

static DeviceData gServoDeviceData;

HDCallbackCode HDCALLBACK jointTorqueCallback(void *data);

void mainLoop(void);

void printHelp(void)
{
    static const char help[] = {"\
Press and release the stylus button to print out the current device location.\n\
Press and hold the stylus button to exit the application\n"};

    std::cout << help << std::endl;
}
 // Силовые
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData) // Гравитация
{
    DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;

    hdGetDoublev(HD_CURRENT_FORCE, pState->forceValues);
    hdGetDoublev(HD_CURRENT_JOINT_TORQUE, pState->jointTorqueValues);
    hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, pState->gimbalTorqueValues);

    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{   
    int nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());

    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    gServoDeviceData.m_buttonState = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
        
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);
    // std::cout << "(" 
    //           << std::fixed << std::setprecision(3) << gServoDeviceData.m_devicePosition[0] << " "
    //           << std::fixed << std::setprecision(3) << gServoDeviceData.m_devicePosition[1] << " "
    //           << std::fixed << std::setprecision(3) << gServoDeviceData.m_devicePosition[2] << ") ";

    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, gServoDeviceData.jointAngles);
    // std::cout << "(" 
    //           << std::fixed << std::setprecision(3) << radiansToDegrees(gServoDeviceData.jointAngles[0]) << " "
    //           << std::fixed << std::setprecision(3) << radiansToDegrees(gServoDeviceData.jointAngles[1]) << " "
    //           << std::fixed << std::setprecision(3) << radiansToDegrees(gServoDeviceData.jointAngles[2]) << ") ";

    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gServoDeviceData.gimbalAngles);
    // std::cout << "(" 
    //           << std::fixed << std::setprecision(3) << radiansToDegrees(gServoDeviceData.gimbalAngles[0]) << " "
    //           << std::fixed << std::setprecision(3) << radiansToDegrees(gServoDeviceData.gimbalAngles[1]) << " "
    //           << std::fixed << std::setprecision(3) << radiansToDegrees(gServoDeviceData.gimbalAngles[2]) << ")\n";

    gServoDeviceData.m_error = hdGetError();

    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;    
}

HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    std::memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}

int main(int argc, char* argv[])
{
    // Инициализация ROS узла
    ros::init(argc, argv, "haptic_device_publisher");
    ros::NodeHandle nh;

    // Создание publisher для публикации данных о положении и углах
    ros::Publisher position_pub = nh.advertise<geometry_msgs::Point>("device_position", 10);
    ros::Publisher angles_pub = nh.advertise<geometry_msgs::Point>("device_angles", 10);

    HDErrorInfo error;
    HDSchedulerHandle hUpdateHandle = 0;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        std::cerr << "\nPress any key to quit.\n";
        getch();
        return -1;           
    }

    // Главный цикл
    ros::Rate loop_rate(100); // Частота публикации сообщений


    std::cout << "Command Joint Torque Demo!" << std::endl;
    std::cout << "Found device model: " << hdGetString(HD_DEVICE_MODEL_TYPE) << std::endl << std::endl;

    hUpdateHandle = hdScheduleAsynchronous(
        updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    // printHelp();

    mainLoop();

    ros::spinOnce();
    loop_rate.sleep();

    hdStopScheduler();
    hdUnschedule(hUpdateHandle);

    hdDisableDevice(hHD);

    return 0;
}

void mainLoop(void)
{
    static const int kTerminateCount = 1000;
    int buttonHoldCount = 0;
    int positionCounter = 1; 
    int JointAnglesCounter = 1;
    int GimbalAnglesCounter = 1;

    DeviceData currentData;
    DeviceData prevData;

    hdScheduleSynchronous(copyDeviceDataCallback, 
        &currentData, HD_MIN_SCHEDULER_PRIORITY);

    std::memcpy(&prevData, &currentData, sizeof(DeviceData));    

    printHelp();

    while (1)
    {
        hdScheduleSynchronous(copyDeviceDataCallback,
                              &currentData,
                              HD_MIN_SCHEDULER_PRIORITY);

        // if (currentData.m_buttonState && !prevData.m_buttonState)
        // {           
        //     std::cout << "Current position " << positionCounter << ": (" 
        //               << currentData.m_devicePosition[0] << ", " 
        //               << currentData.m_devicePosition[1] << ", " 
        //               << currentData.m_devicePosition[2] << ")\n";
        //     positionCounter++; 
        //     std::cout << "Current Joint Angles " << JointAnglesCounter << ": (" 
        //               << currentData.jointAngles[0] << ", " 
        //               << currentData.jointAngles[1] << ", " 
        //               << currentData.jointAngles[2] << ")\n";
        //     JointAnglesCounter++;
        //     std::cout << "Current Gimbal Angles " << GimbalAnglesCounter << ": (" 
        //               << currentData.gimbalAngles[0] << ", " 
        //               << currentData.gimbalAngles[1] << ", " 
        //               << currentData.gimbalAngles[2] << ")\n\n";
        // }

        // Угловые значения с градусами
        if (currentData.m_buttonState && !prevData.m_buttonState)
        {           
            std::cout << "Positions " << positionCounter << ": (" 
                      << currentData.m_devicePosition[0] << ", " 
                      << currentData.m_devicePosition[1] << ", " 
                      << currentData.m_devicePosition[2] << ") ";
            positionCounter++; 
            std::cout << "Angles (" 
                      << radiansToDegrees(currentData.jointAngles[0]) << ", " 
                      << radiansToDegrees(currentData.jointAngles[1]) << ", " 
                      << radiansToDegrees(currentData.jointAngles[2]) << ", " 
                      << radiansToDegrees(currentData.gimbalAngles[0]) << ", " 
                      << radiansToDegrees(currentData.gimbalAngles[1]) << ", " 
                      << radiansToDegrees(currentData.gimbalAngles[2]) << ")\n\n";
        }
        else if (currentData.m_buttonState && prevData.m_buttonState)
        {
            buttonHoldCount++;

            if (buttonHoldCount > kTerminateCount)
            {
                break;
            }
        }
        else if (!currentData.m_buttonState && prevData.m_buttonState)
        {
            buttonHoldCount = 0;
        }
        
        if (HD_DEVICE_ERROR(currentData.m_error))
        {
            hduPrintError(stderr, &currentData.m_error, "Device error detected");

            if (hduIsSchedulerError(&currentData.m_error))
            {
                std::cerr << "\nPress any key to quit.\n";
                getch();                
                break;
            }
        }

        std::memcpy(&prevData, &currentData, sizeof(DeviceData));    
    }
}
