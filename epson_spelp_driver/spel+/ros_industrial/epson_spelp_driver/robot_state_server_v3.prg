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

#include "simple_message.inc"
#include "robotinfo.inc"

'in seconds. Smallest 0.01seconds
#define SLEEP_TIME 0.3

'hardcoded for now 
#define SOCKET_TAG 201

Function robot_state_mainv3
		
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
			WaitNet #SOCKET_TAG, 10
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
				
				Call SendRobotStateToROSv3()
				'Print "Sending:", counter
				counter = counter + 1
				
				'sleep a bit (100ms)
				Wait SLEEP_TIME '* 100
				
				'debug: exit after single iter
				'Exit Function
				
			'inner client handling
			Loop
			
			Print "error?"
			'Wait 1.0
			
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
	 	
	 	'Integer i
	 	'For i = 0 To (sm_hdr(0) + SM_LEN_PREFIX - 1)
	 	'	WriteBin #SOCKET_TAG, intvaluesSM(i)
	 	'Next i
		
Fend
