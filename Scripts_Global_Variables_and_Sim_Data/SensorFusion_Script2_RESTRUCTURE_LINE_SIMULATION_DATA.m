% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : This script rearranges the simulated sensor data such that the data can be accessed conveniently.
%             : The simulated data is generated using matlab
% ==============================================================================================================================================================
% EGO SENSOR ( Simulation Data )
EGO_Sensor_Simulated_Data.px = EGO_PX;
EGO_Sensor_Simulated_Data.py = EGO_PX;
EGO_Sensor_Simulated_Data.vx = EGO_VX;
EGO_Sensor_Simulated_Data.vy = EGO_VY;
EGO_Sensor_Simulated_Data.yaw = EGO_YAW;
EGO_Sensor_Simulated_Data.yawRate = EGO_YAWRATE;
EGO_Sensor_Simulated_Data.detTimeStamp = EGO_TimeStamp;
% ==============================================================================================================================================================
% CAMERA 1 ( Simulation Data )
CAM1_Line_Sensor_Simulated_Data.LateralOffset = LINE_LATERALOFFSET_CAM1;
CAM1_Line_Sensor_Simulated_Data.HeadingAngle = LINE_HEADINGANGLE_CAM1;
CAM1_Line_Sensor_Simulated_Data.Curvature = LINE_CURVATURE_CAM1; 
CAM1_Line_Sensor_Simulated_Data.CurvatureDerivative = LINE_CURVDERIVATIVE_CAM1; 
CAM1_Line_Sensor_Simulated_Data.CurveLength = LINE_CURVELENGTH_CAM1; 
CAM1_Line_Sensor_Simulated_Data.LineWidth = LINE_WIDTH_CAM1;
CAM1_Line_Sensor_Simulated_Data.MaximumValidX = LINE_XMAX_CAM1;
CAM1_Line_Sensor_Simulated_Data.MinimumValidX = LINE_XMIN_CAM1;
CAM1_Line_Sensor_Simulated_Data.detTimeStamp = LINE_detTimeStamp_CAM1;
% ==============================================================================================================================================================
% CAMERA 2 ( Simulation Data )
CAM2_Line_Sensor_Simulated_Data.LateralOffset = LINE_LATERALOFFSET_CAM2;
CAM2_Line_Sensor_Simulated_Data.HeadingAngle = LINE_HEADINGANGLE_CAM2;
CAM2_Line_Sensor_Simulated_Data.Curvature = LINE_CURVATURE_CAM2; 
CAM2_Line_Sensor_Simulated_Data.CurvatureDerivative = LINE_CURVDERIVATIVE_CAM2; 
CAM2_Line_Sensor_Simulated_Data.CurveLength = LINE_CURVELENGTH_CAM2; 
CAM2_Line_Sensor_Simulated_Data.LineWidth = LINE_WIDTH_CAM2;
CAM2_Line_Sensor_Simulated_Data.MaximumValidX = LINE_XMAX_CAM2;
CAM2_Line_Sensor_Simulated_Data.MinimumValidX = LINE_XMIN_CAM2;
CAM2_Line_Sensor_Simulated_Data.detTimeStamp = LINE_detTimeStamp_CAM2;
% ===============================================================> END OF SCRIPT <===================================================================