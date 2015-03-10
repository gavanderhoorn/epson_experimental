' Copyright 2015 TU Delft Robotics Institute
' 
' Licensed under the Apache License, Version 2.0 (the "License");
' you may not use this file except in compliance with the License.
' You may obtain a copy of the License at
' 
'     http://www.apache.org/licenses/LICENSE-2.0
' 
' Unless required by applicable law or agreed to in writing, software
' distributed under the License is distributed on an "AS IS" BASIS,
' WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
' See the License for the specific language governing permissions and
' limitations under the License.

' Constants used in the ROS-Industrial Simple Message protocol
' 
' Author: G.A. vd. Hoorn (TU Delft Robotics Institute)
' 

#ifndef SIMPLE_MESSAGE_H__
#define SIMPLE_MESSAGE_H__


'Standard lengths (in bytes)
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

'Header Comm Type values
#define SM_COMM_TYPE_INVALID 0
#define SM_COMM_TYPE_TOPIC 1
#define SM_COMM_TYPE_SERVICE_REQUEST 2
#define SM_COMM_TYPE_SERVICE_REPLY 3

'Header Reply Type values
#define SM_REPLY_TYPE_INVALID 0
#define SM_REPLY_TYPE_SUCCESS 1
#define SM_REPLY_TYPE_FAILURE 2



'RobotStatus - RobotMode values
#define SM_ROBOT_MODE_UNKNOWN -1
#define SM_ROBOT_MODE_MANUAL 1
#define SM_ROBOT_MODE_AUTO 2

'RobotStatus - TriState values
#define SM_TRISTATE_UNKNOWN -1
#define SM_TRISTATE_TRUE 1
#define SM_TRISTATE_ON 1
#define SM_TRISTATE_ENABLED 1
#define SM_TRISTATE_HIGH 1
#define SM_TRISTATE_FALSE 0
#define SM_TRISTATE_OFF 0
#define SM_TRISTATE_DISABLED 0
#define SM_TRISTATE_LOW 0



#endif 'SIMPLE_MESSAGE_H__
