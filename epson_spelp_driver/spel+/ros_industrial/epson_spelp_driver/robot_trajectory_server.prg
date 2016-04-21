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
'	    L.Castelli (Student at UHasselt/KULeuven, ACRO)
'	    W. vd. Aelst (Student at UHasselt/KULeuven, ACRO)

'TODO: fix the seq nr problem

Declare convertToFloatingpoint, "DLLsForEpsonDriver.dll"(ByRef inputintegers() As Integer, ByRef output_joint_data() As Real, ByRef output_velocity As Real, ByRef output_duration As Real) As Short

#include "simple_message.inc"
#include "robotinfo.inc"

'in seconds. Smallest 0.01seconds
#define SLEEP_TIME 0.3

'hardcoded for now 
#define SOCKET_TAG 202

Function robot_trajectory_main
		
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
			SetNet #SOCKET_TAG, "192.168.0.2", 11000
			OpenNet #SOCKET_TAG As Server
			WaitNet #SOCKET_TAG, 10
			Wait SLEEP_TIME * 10
			status = ChkNet(SOCKET_TAG)
			If status < 0 Then
				Print "Error with socket:", status
				Exit Function
			EndIf
			Print "ChkNet:", status
			
			Print "Connected"
			
			Do While Not ShutdownRequested
				
				'client loop, check connection
				status = ChkNet(SOCKET_TAG)
				If status < 0 Then
						Print "Error with socket:"
						Exit Function
				EndIf
				Print "ChkNet:", status
				
				Call readSimpleMessage()
				Call sendServiceReply()
				
				'sleep a bit (100ms)
				Wait SLEEP_TIME '* 100
				
				'debug: exit after single iter
				'Exit Function
				
			'inner client handling
			Loop
			
			Print "error?"
			Wait 1.0
			
		'outer loop
		Loop
		Print "Exit"
Fend
Function readSimpleMessage()
	
	'Joint trajectory pt Simple message structure
	'sm_length -> 4 bytes
	'sm_hdr(0) = sm_id -> 4 bytes
	'sm_hdr(1) = sm_comm_type -> 4 bytes
	'sm_hdr(2) = sm_reply_type -> 4 bytes
	'joint_data -> 40 bytes (10 joints)
	'velocity -> 4 bytes
	'duration -> 4 bytes
	
	Long sm_length
	Long sm_hdr(2)
	Long seq
	Real joint_data(9)
	Real velocity
	Real duration
		
	'read first 4 bytes of the simple message (simple message length)
	Integer read_sm_length(3)
	ReadBin #SOCKET_TAG, read_sm_length(), UBound(read_sm_length) + 1
	
	'Convert the 4 bytes to a Long integer which gives the SM length
	sm_length = read_sm_length(0) + read_sm_length(1) * &H100 + read_sm_length(2) * &H10000 + read_sm_length(3) * &H1000000
	Print "sm_length: ", sm_length
	
	'length of a joint_trajectory_pt message = 64 bytes
	If sm_length = 64 Then
		'read header of the simple message (including sequence nr part)
		Integer read_hdr(15)
		ReadBin #SOCKET_TAG, read_hdr(), UBound(read_hdr) + 1
		
		'get values of the simple message header (including sequence part of the sm)
		Call getHeaderValues(ByRef read_hdr(), ByRef sm_hdr(), ByRef seq)
		Print "sm_id: ", sm_hdr(0)
		Print "sm_comm_type: ", sm_hdr(1)
		Print "sm_reply_type: ", sm_hdr(2)
		Print "sequence nr: ", seq
		
		'read the remaining bytes
		Integer read_remaining_bytes(47)
		ReadBin #SOCKET_TAG, read_remaining_bytes(), UBound(read_remaining_bytes) + 1
		
		'get floatingpoint values(joint_data,velocity,duration) of the sm by
		'converting the remaining bytes using a DLL
		'only do this when we are sure we received a joint trajectory pt message
		'only do this when we are sure we didn't receive a trajectory stop message
		'trajectory stop message has a sequence nr = -4
		If sm_hdr(0) = 11 And seq >= 0 Then
			'TODO: Make a DLL and use it here
			Short getFloatingpoints
			getFloatingpoints = convertToFloatingpoint(ByRef read_remaining_bytes(), ByRef joint_data(), ByRef velocity, ByRef duration)
			Integer k
			For k = 0 To 9
				Print "joint", k, ": ", joint_data(k)
			Next k
			
		ElseIf seq = -4 Then
			Print "received a trajectory stop message, no movement of the robot is allowed"
		EndIf
		
	Else
		Print "Error, unexpected simple message length. Close socket?"
	EndIf
Fend
Function getHeaderValues(ByRef read_hdr() As Integer, ByRef sm_hdr() As Long, ByRef seq As Long)
	Integer i
	For i = 0 To UBound(sm_hdr()) + 1
		If i < 3 Then
			sm_hdr(i) = read_hdr(i * 4) + read_hdr((i * 4) + 1) * &H100 + read_hdr((i * 4) + 2) * &H10000 + read_hdr((i * 4) + 3) * &H1000000
		Else
			'when seq = -4 we get an error here (epson bug)
			'temporary fix for this problem... (we will never have a seq nr higher than 0x01000000 unless we get a negative seq nr (-4 = 0xFFFFFFFC)
			If (read_hdr((i * 4) + 3) > 0) Then
				seq = -4
			Else
				seq = read_hdr(i * 4) + read_hdr((i * 4) + 1) * &H100 + read_hdr((i * 4) + 2) * &H10000 + read_hdr((i * 4) + 3) * &H1000000
			EndIf
		EndIf
	Next i
Fend
Function sendServiceReply()
	Integer i
	Integer SMreply(59) 'array to be written to generic client
	For i = 0 To 59
		If i = 0 Then
			SMreply(i) = 56
		ElseIf i = 4 Then
			SMreply(i) = SM_MSG_TYPE_JOINT_POSITION
		ElseIf i = 8 Then
			SMreply(i) = SM_COMM_TYPE_SERVICE_REPLY
		ElseIf i = 12 Then
			SMreply(i) = SM_REPLY_TYPE_SUCCES
		Else
			SMreply(i) = 0
		EndIf
	Next i
	'write reply to ROS
	WriteBin #202, SMreply(), UBound(SMreply) + 1
Fend


