'Author: G.A. vd. Hoorn(TU Delft Robotics Institute)

#ifndef SIMPLE_MESSAGE_H__
#define SIMPLE_MESSAGE_H__

'Standard lengths
#define SM_LEN_PREFIX 4
#define SM_LEN_HDR (3 * 4)

'Standard Message Set IDs (from REP-I0004)
#define SM_MSG_TYPE_INVALID 0
#define SM_MSG_TYPE_JOINT_TRAJ_PT 11
#define SM_MSG_TYPE_JOINT_TRAJ 12
#define SM_MSG_TYPE_STATUS 13
#define SM_MSG_TYPE_JOINT_TRAJ_PT_FULL 14
#define SM_MSG_TYPE_JOINT_FEEDBACK 15

'Deprecated message types
#define SM_MSG_TYPE_JOINT_POSITION 10
#define SM_MSG_TYPE_JOINT 10
#define SM_MSG_TYPE_READ_INPUT 20
#define SM_MSG_TYPE_WRITE_OUTPUT 21

'Header comm type values
#define SM_COMM_TYPE_INVALID 0
#define SM_COMM_TYPE_TOPIC 1
#define SM_COMM_TYPE_SERVICE_REQUEST 2
#define SM_COMM_TYPE_SERVICE_REPLY 3

'Header Reply type values
#define SM_REPLY_TYPE_INVALID 0
#define SM_REPLY_TYPE_SUCCES 1
#define SM_REPLY_TYPE_FAILURE 2


'Robot Status - Robot Mode Values
#define SM_ROBOT_MODE_UNKNOWN -1
#define SM_ROBOT_MODE_MANUAL 1
#define SM_ROBOT_MODE_AUTO 2

'Robot Status - Tristate values
#define SM_TRI_STATE_UNKNOWN -1
#define SM_TRI_STATE_TRUE 1
#define SM_TRI_STATE_FALSE 0

#endif




