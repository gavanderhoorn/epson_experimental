Declare convertToFloatingpoint, "DLLsForEpsonDriver.dll"(ByRef inputintegers() As Integer, ByRef output_joint_data() As Real, ByRef output_velocity As Real, ByRef output_duration As Real) As Short

Function main

Xqt robot_state_mainv3, NoEmgAbort
Xqt robot_trajectory_main_v2, Normal
'Call robot_trajectory_main_smooth

Fend


