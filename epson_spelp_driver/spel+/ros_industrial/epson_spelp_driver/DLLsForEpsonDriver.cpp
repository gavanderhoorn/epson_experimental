// DLLsForEpsonDriver.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "DLLsForEpsonDriver.h"


// This is an example of an exported variable
DLLSFOREPSONDRIVER_API int nDLLsForEpsonDriver=0;

// This is an example of an exported function.
DLLSFOREPSONDRIVER_API int fnDLLsForEpsonDriver(void)
{
	return 42;
}
DLLSFOREPSONDRIVER_API short JointPositionSMtoInts(int sm_length,int sm_id ,int sm_comm_type,int sm_reply_type, int seq,float joint1,float joint2,float joint3,float joint4,float joint5 , float joint6, float joint7, float joint8,float joint9, float joint10,short* outputIntegers)
{
    float joint_data[10];
    joint_data[0]= joint1;
    joint_data[1]= joint2;
    joint_data[2]= joint3;
    joint_data[3]= joint4;
    joint_data[4]= joint5;
    joint_data[5]= joint6;
    joint_data[6]= joint7;
    joint_data[7]= joint8;
    joint_data[8]= joint9;
    joint_data[9]= joint10;
	
	unsigned char buffer[60];
	memcpy(buffer,&sm_length,sizeof(sm_length));
	memcpy(buffer + 4,&sm_id,sizeof(sm_id));
	memcpy(buffer + 8,&sm_comm_type,sizeof(sm_comm_type));
	memcpy(buffer + 12,&sm_reply_type,sizeof(sm_reply_type));
	memcpy(buffer + 16,&seq,sizeof(seq));
    memcpy(buffer + 20, &joint_data, sizeof joint_data);

    for (int i=0;i<sizeof(buffer);i++)
    {
		outputIntegers[i]=buffer[i];
    }

	return 1;
}
DLLSFOREPSONDRIVER_API short RobotStatusSMtoInts(int sm_length,int sm_id ,int sm_comm_type,int sm_reply_type,int* robot_status_body, short* outputIntegers)
{	
	unsigned char buffer[44];
	memcpy(buffer,&sm_length,sizeof(sm_length));
	memcpy(buffer + 4,&sm_id,sizeof(sm_id));
	memcpy(buffer + 8,&sm_comm_type,sizeof(sm_comm_type));
	memcpy(buffer + 12,&sm_reply_type,sizeof(sm_reply_type));
    memcpy(buffer + 16, robot_status_body,28);

    for (int i=0;i<sizeof(buffer);i++)
    {
		outputIntegers[i]=buffer[i];
    }

	return 1;
}
DLLSFOREPSONDRIVER_API short convertToFloatingpoint(short* inputintegers,float* joint_data,float velocity,float duration)
{
    unsigned char jointbuffer[40];
    unsigned char velocitybuffer[4];
    unsigned char durationbuffer[4];
	
	// filter the most significant bytes out of the input array
	for(int i=0;i<48;i++)
    {
        if(i<40)
        {
            jointbuffer[i] = static_cast<unsigned char>(inputintegers[i]);
        }
        if(i>=40 && i<44)
        {
            velocitybuffer[i-sizeof(jointbuffer)] = static_cast<unsigned char>(inputintegers[i]);
        }
        if (i>=44 && i<48)
        {
            durationbuffer[i-sizeof(jointbuffer)-sizeof(velocitybuffer)] = static_cast<unsigned char>(inputintegers[i]);
        }
    }
	
	//copy bufferdata to the concerned memory block
	memcpy(joint_data,&jointbuffer,sizeof(jointbuffer));
    memcpy(&velocity,&velocitybuffer,sizeof(velocitybuffer));
    memcpy(&duration,&durationbuffer,sizeof(durationbuffer));

	return 1;
}

// This is the constructor of a class that has been exported.
// see DLLsForEpsonDriver.h for the class definition
CDLLsForEpsonDriver::CDLLsForEpsonDriver()
{
	return;
}
