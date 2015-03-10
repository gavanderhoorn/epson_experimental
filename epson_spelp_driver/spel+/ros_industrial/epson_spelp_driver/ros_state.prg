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

' Initial attempt at industrial_robot_client compatible state server
'
' Author: G.A. vd. Hoorn (TU Delft Robotics Institute)
'
' TODO:
'  - make this a background task
'  - use native dll to do msg (de)serialisation (?) if WriteBin can't do it
'

#include "simple_message.inc"


'in seconds. Smallest 0.01seconds
#define SLEEP_TIME 1

'hardcoded for now
#define SOCKET_TAG 201


Function ros_state_main
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
		'SetNet #SOCKET_TAG, "", 11002
		OpenNet #SOCKET_TAG As Server
		WaitNet #SOCKET_TAG, 10

		status = ChkNet(SOCKET_TAG)
		If status < 0 Then
			Print "Error with socket:", status
			Exit Function
		EndIf
		Print "ChkNet:", status

		'should have a client here (?)
		Print "Connected"

		Do While Not ShutdownRequested

			'client loop, check connection
			status = ChkNet(SOCKET_TAG)
			If status < 0 Then
				Print "Error with socket:", status
				Exit Function
			EndIf
			Print "ChkNet:", status

			Call SendJointPosition(SOCKET_TAG)

			Print "Sending:", counter
			counter = counter + 1

			'sleep a bit (100 ms)
			Wait SLEEP_TIME

			'debug: exit after single iter
			Exit Function

		'inner client handling
		Loop

		Print "Error?"
		Wait 1.0

	'outer loop
	Loop

	Print "Exit"

Fend


Function SendJointPosition(sock_id As Int32)
	'pkt structure
	'
	' Prefix
	'  pkt_len          INT32
	' Header
	'  msg_type         INT32
	'  comm_type        INT32
	'  reply_type       INT32
	'Body
	'  seq              INT32
	'  joint_data[10]   REAL

	Long sm_hdr(4)
	UInt32 seq
	Real joint_data(10)
	Byte data(56)
	Integer i, status

	'setup prefix & header
	sm_hdr(0) = (UBound(sm_hdr, 1) * 4) + (UBound(joint_data, 1) * 4)
	sm_hdr(1) = SM_MSG_TYPE_JOINT_POSITION
	sm_hdr(2) = SM_COMM_TYPE_TOPIC
	sm_hdr(3) = SM_REPLY_TYPE_INVALID

	'get data
	'TODO: refactor: make this caller's responsibility
	GetJointPositions(ByRef joint_data())

	'send out over socket

	If False Then
		'approach 1: pass WriteBin Long/Real arrays directly

		'prefix & hdr
		WriteBin #sock_id, sm_hdr(), UBound(sm_hdr, 1)

		'body: status data
		WriteBin #sock_id, joint_data(), UBound(joint_data, 1)

	Else
		'approach 2: manually serialise Long/Real into LE byte array
		'TODO: make approach 1 work

		'convert
		AddToByteArray(ByRef data(), 0, sm_hdr(0))
		AddToByteArray(ByRef data(), 4, sm_hdr(1))
		AddToByteArray(ByRef data(), 8, sm_hdr(2))
		AddToByteArray(ByRef data(), 12, sm_hdr(3))

		AddToByteArray(ByRef data(), 16, seq)

		'For i = 1 To 10
		'	AddToByteArray(ByRef data(), 20 + (i * 4), joint_data(i))
		'Next i

		WriteBin #sock_id, data(), UBound(data, 1)

	EndIf


	'TODO: figure out how to flush
Fend


'serialises (little endian order) 'ival' into 'buf' byte array
Function AddToByteArray(ByRef buf() As Byte, wpos As Integer, ival As UInt32)
	buf(wpos + 0) = (ival And &HFF)
	buf(wpos + 1) = ((ival / &H100) And &HFF)
	buf(wpos + 2) = ((ival / &H10000) And &HFF)
	buf(wpos + 3) = ((ival / &H1000000) And &HFF)
Fend


Function GetJointPositions(ByRef data() As Real)
	'assume 6 axis robot for now, single group, only revolute joints
	data(0) = DegToRad(Agl(1))
	data(1) = DegToRad(Agl(2))
	data(2) = DegToRad(Agl(3))
	data(3) = DegToRad(Agl(4))
	data(4) = DegToRad(Agl(5))
	data(5) = DegToRad(Agl(6))
Fend


Function SendRobotStatus(sock_id As Int32)
	'NOTE: this doesn't work properly yet


	'pkt structure
	'
	' Prefix
	'  pkt_len          INT32
	' Header
	'  msg_type         INT32
	'  comm_type        INT32
	'  reply_type       INT32
	'Body
	'  mode             INT32
	'  e_stopped        INT32
	'  drives_powered   INT32
	'  motion_possible  INT32
	'  in_motion        INT32
	'  in_error         INT32
	'  error_code       INT32

	Int32 sm_hdr(4)
	UInt32 sm_body(7)

	'setup prefix & header
	sm_hdr(0) = (UBound(sm_hdr, 1) * 4) + (UBound(sm_body, 1) * 4)
	sm_hdr(1) = SM_MSG_TYPE_STATUS
	sm_hdr(2) = SM_COMM_TYPE_TOPIC
	sm_hdr(3) = SM_REPLY_TYPE_INVALID

	'init to default values
	sm_body(0) = SM_ROBOT_MODE_MANUAL   'robot mode
	sm_body(1) = SM_TRISTATE_FALSE      'e_stopped
	sm_body(2) = SM_TRISTATE_FALSE      'drives_powered
	sm_body(3) = SM_TRISTATE_UNKNOWN    'motion_possible
	sm_body(4) = SM_TRISTATE_UNKNOWN    'in_motion
	sm_body(5) = SM_TRISTATE_FALSE      'in_error
	sm_body(6) = 0                      'error_code

	'robot_mode
	If CtrlInfo(7) = 1 Then
		sm_body(0) = SM_ROBOT_MODE_AUTO
	EndIf

	'e_stopped
	'If (CtrlInfo(1) And &H100) Then
	If EStopOn() Then
		sm_body(1) = SM_TRISTATE_TRUE
	EndIf

	'drives_powered
	If Motor = On Then
		sm_body(2) = SM_TRISTATE_TRUE
	EndIf

	'in_error
	If ErrorOn() Then
		sm_body(5) = SM_TRISTATE_TRUE
		'get error code (0 == current task)
		sm_body(6) = Err(0)
	EndIf

	'TODO: set error_code and other fields in msg body


	'send to client
	'prefix & hdr
	WriteBin #sock_id, sm_hdr(), (UBound(sm_hdr, 1) * 4)

	'body: status data
	WriteBin #sock_id, sm_body(), (UBound(sm_body, 1) * 4)

	'TODO: figure out how to flush
Fend



