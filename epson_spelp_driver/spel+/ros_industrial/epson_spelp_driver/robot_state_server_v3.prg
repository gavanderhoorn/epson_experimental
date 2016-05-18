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
'
' Author: G.A. vd. Hoorn (TU Delft Robotics Institute)
'	      L.Castelli (Student at UHasselt/KULeuven, ACRO)
'	      W. vd. Aelst (Student at UHasselt/KULeuven, ACRO)

Declare JointPositionSMtoInts, "DLLsForEpsonDriver.dll"(sm_len As Long, sm_id As Long, sm_comm As Long, sm_reply As Long, seq As Long, joint1 As Real, joint2 As Real, joint3 As Real, joint4 As Real, joint5 As Real, joint6 As Real, joint7 As Real, joint8 As Real, joint9 As Real, joint10 As Real, ByRef intvalues() As Integer) As Short
Declare RobotStatusSMtoInts, "DLLsForEpsonDriver.dll"(sm_len As Long, sm_id As Long, sm_comm As Long, sm_reply As Long, ByRef robot_status_body() As Long, ByRef intvalues() As Integer) As Short

#include "simple_message.inc"
#include "robotinfo.inc"

'in seconds. Smallest 0.01seconds
#define SLEEP_TIME 0.01

'hardcoded for now 
#define SOCKET_TAG 201

Function robot_state_mainv3()
	
					
		'var decls
		Integer bytesAvail, status, counter
		Boolean ShutdownRequested
 	
		'var init
		bytesAvail = 0
		status = 0
		counter = 0
		ShutdownRequested = False
		
		Print "Init done"
		
		Do While Not ShutdownRequested
			
			Print "Waiting for client .."
			
			'socket accept
			CloseNet #SOCKET_TAG
			SetNet #SOCKET_TAG, "192.168.0.2", 11002
			OpenNet #SOCKET_TAG As Server
			WaitNet #SOCKET_TAG, 50
			Wait SLEEP_TIME
			status = ChkNet(SOCKET_TAG)
			If status < 0 Then
				Print "Error with socket:", status
				Exit Function
			EndIf
			'Print "ChkNet:", status
			
			'should have a client here (?)
			Print "Connected"
			
			Do While Not ShutdownRequested
					
				'client loop, check connection
				status = ChkNet(SOCKET_TAG)
				If status < 0 Then
						Print "Error with socket:"
						Exit Function
				EndIf
				'Print "ChkNet:", status
				
				OnErr GoTo ErrorHandler
				Call SendRobotStateToROSv3()
				counter = counter + 1
				If counter = 10 Then
					counter = 0
					
					OnErr GoTo ErrorHandler
					Call SendRobotStatusSM
				EndIf
				
				'sleep a bit (100ms)
				Wait SLEEP_TIME '* 100
				
				'debug: exit after single iter
				'Exit Function
				
			'inner client handling
			Loop
			
			Print "error?"
			'Wait 1.0
			ErrorHandler:
					Quit All
		
		'outer loop
		Loop
		Print "Exit"

Fend

Function GetJointValuesV3(ByRef data() As Real)
	'assume 6 axis robot for now, single group, only revolute joints
	data(0) = DegToRad(Agl(1))
	'Print data(0)
	data(1) = DegToRad(Agl(2))
	'Print data(1)
	data(2) = DegToRad(Agl(3))
	'Print data(2)
	data(3) = DegToRad(Agl(4))
	'Print data(3)
	data(4) = DegToRad(Agl(5))
	'Print data(4)
	data(5) = DegToRad(Agl(6))
	'Print data(5)
Fend

Function SendRobotStateToROSv3()
		'Joint Position Simple message structure
		'--------------------------------------------------
		'sm_hdr(0) = message length -> 4 bytes (int32)
		'sm_hdr(1) = sm_id -> 4 bytes (Int32)
		'sm_hdr(2) = sm_comm_type -> 4 bytes (Int32)
		'sm_hdr(3) = sm_reply_type -> 4 bytes (Int32)
		'sequence nr -> 4 bytes (Int32)
		'joint_data -> 40 bytes (10 joints) (Real)
		'--------------------------------------------------
				
		Long sm_hdr(3)
		Long seq
		Real joint_data(9)
		
        sm_hdr(0) = ((UBound(sm_hdr) + 1) * 4) + ((UBound(joint_data) + 1) * 4) ' = 56 bytes
		sm_hdr(1) = SM_MSG_TYPE_JOINT_POSITION
		sm_hdr(2) = SM_COMM_TYPE_TOPIC
		sm_hdr(3) = SM_REPLY_TYPE_INVALID
		seq = 0
		
		GetJointValuesV3(ByRef joint_data())
					
		Integer intvaluesSM(255)
		Integer getIntValuesSM
			
			
	 	getIntValuesSM = JointPositionSMtoInts(sm_hdr(0), sm_hdr(1), sm_hdr(2), sm_hdr(3), seq, joint_data(0), joint_data(1), joint_data(2), joint_data(3), joint_data(4), joint_data(5), joint_data(6), joint_data(7), joint_data(8), joint_data(9), ByRef intvaluesSM())
	 	
	 	
	 	WriteBin #SOCKET_TAG, intvaluesSM(), 60
		
Fend
Function SendRobotStatusSM
		'Robot Status Simple message structure
		'--------------------------------------------------
		'robot_status_hdr(0) = message length (Int32)
		'robot_status_hdr(1) = message type/id (Int32)
		'robot_status_hdr(2) = comm type (Int32)
		'robot_status_hdr(3) = reply type (Int32)
		'robot_status_body(0) = drives powered (Int32)
		'robot_status_body(1) = e stopped (Int32)
		'robot_status_body(2) = error code (Int32)
		'robot_status_body(3) = in error(Int32)
		'robot_status_body(4) = in motion (Int32)
		'robot_status_body(5) = mode (Int32)
		'robot_status_body(6) = motion possible (Int32)
		'--------------------------------------------------
		Long robot_status_hdr(3)
		Long robot_status_body(6)
		
		'setup prefix & header
        robot_status_hdr(0) = (UBound(robot_status_hdr) * 4) + ((UBound(robot_status_body) + 1) * 4)
        robot_status_hdr(1) = SM_MSG_TYPE_STATUS
        robot_status_hdr(2) = SM_COMM_TYPE_TOPIC
        robot_status_hdr(3) = SM_REPLY_TYPE_INVALID
        
		
		'drives powered
		If Motor = On Then
			robot_status_body(0) = SM_TRI_STATE_TRUE
		Else
			robot_status_body(0) = SM_TRI_STATE_FALSE
		EndIf
		
		'e stopped
		If EStopOn = True Then
			robot_status_body(1) = SM_TRI_STATE_TRUE
		Else
			robot_status_body(1) = SM_TRI_STATE_FALSE
		EndIf
		
		'error code	
		If Ert > 0 Then
			OnErr GoTo Error2261 'error: specified task number does not exist
								 '(happens when the emergency button aborted the trajectory server task)
			robot_status_body(2) = Err(Ert)
		Else
			robot_status_body(2) = 0
		EndIf
		
		'in error
		If ErrorOn = -1 Then
			robot_status_body(3) = SM_TRI_STATE_TRUE
		Else
			robot_status_body(3) = SM_TRI_STATE_FALSE
		EndIf
		
		'in motion
		'TODO: find a way to check this part of the status msg!
		robot_status_body(4) = SM_TRI_STATE_UNKNOWN
		
		'robot mode 
		If CtrlInfo(7) = 0 Then
			'Running project(Run Window F5) = Manual mode
			robot_status_body(5) = SM_ROBOT_MODE_MANUAL
		ElseIf CtrlInfo(7) = 1 Then
			'test auto mode (shift F5) = Auto mode
			robot_status_body(5) = SM_ROBOT_MODE_AUTO
		EndIf
		
		'motion possible
		'TODO: find a way to check this part of the msg!
		robot_status_body(6) = SM_TRI_STATE_UNKNOWN
		
		'Write robot status message to ROS
		Integer intvaluesSM(43)
		Short dllFinished
		dllFinished = RobotStatusSMtoInts(robot_status_hdr(0), robot_status_hdr(1), robot_status_hdr(2), robot_status_hdr(3), ByRef robot_status_body(), ByRef intvaluesSM())
		WriteBin #SOCKET_TAG, intvaluesSM(), 44
		
		'********************Error Handler************************
		Error2261:
			robot_status_body(2) = 2261
		'*********************************************************
Fend




