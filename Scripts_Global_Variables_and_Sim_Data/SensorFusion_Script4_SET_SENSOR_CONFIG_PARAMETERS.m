% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : Sensor Callibration Parameters Specifications   
% ==============================================================================================================================================================
% RADAR Long Range Radar (LRR) Intrinsic Parameters (Field Of View and Maximum Range)
RADAR_TYPE_LRR = uint8(1);
RADAR_LRR_Max_Range = single(200);
RADAR_LRR_Max_Azimuth = single(15);
RADAR_LRR_Max_Elevation = single(-1); %not currently set
RADAR_LRR_Max_RangeRate = single(-1); %not currently set
RADAR_LRR_Max_RangeRes = single(-1); %not currently set
RADAR_LRR_Max_AzimuthRes = single(-1); %not currently set
RADAR_LRR_Max_ElevationRes = single(-1); %not currently set
RADAR_LRR_Max_RangeRateRes = single(-1); %not currently set
RADAR_LRR_Max_RangeErr = single(-1); %not currently set
RADAR_LRR_Max_AzimuthErr = single(-1); %not currently set
RADAR_LRR_Max_ElevationErr = single(-1); %not currently set
RADAR_LRR_Max_RangeRateErr = single(-1); %not currently set
RADAR_LRR_Max_ProbOfDetection = single(0.99); 
RADAR_LRR_Max_FalseAlarmRate = single(2/45000); 
RADAR_LRR_Max_FovRangePts = single(zeros(1, 20)); %not currently set
RADAR_LRR_Max_FovAzimuthPts = single(zeros(1, 20)); %not currently set
% ==============================================================================================================================================================
% RADAR Long Range Radar (MRR) Intrinsic Parameters (Field Of View and Maximum Range)
RADAR_TYPE_MRR = uint8(2);
RADAR_MRR_Max_Range = single(100);
RADAR_MRR_Max_Azimuth = single(50);
RADAR_MRR_Max_Elevation = single(-1); %not currently set
RADAR_MRR_Max_RangeRate = single(-1); %not currently set
RADAR_MRR_Max_RangeRes = single(-1); %not currently set
RADAR_MRR_Max_AzimuthRes = single(-1); %not currently set
RADAR_MRR_Max_ElevationRes = single(-1); %not currently set
RADAR_MRR_Max_RangeRateRes = single(-1); %not currently set
RADAR_MRR_Max_RangeErr = single(-1); %not currently set
RADAR_MRR_Max_AzimuthErr = single(-1); %not currently set
RADAR_MRR_Max_ElevationErr = single(-1); %not currently set
RADAR_MRR_Max_RangeRateErr = single(-1); %not currently set
RADAR_MRR_Max_ProbOfDetection = single(0.99); 
RADAR_MRR_Max_FalseAlarmRate = single(2/45000);
RADAR_MRR_Max_FovRangePts = single(zeros(1, 20)); %not currently set
RADAR_MRR_Max_FovAzimuthPts = single(zeros(1, 20)); %not currently set
% ==============================================================================================================================================================
% Sensor Intrinsic Parameters (Field Of View and Maximum Range)
RADAR_MAX_RANGE = [RADAR_LRR_Max_Range; RADAR_MRR_Max_Range];
RADAR_MAX_AZIMUTH = [RADAR_LRR_Max_Azimuth; RADAR_MRR_Max_Azimuth];
RADAR_MAX_ELEVATION = [RADAR_LRR_Max_Elevation; RADAR_MRR_Max_Elevation];
RADAR_MAX_RANGE_RATE = [RADAR_LRR_Max_RangeRate; RADAR_MRR_Max_RangeRate];
RADAR_RANGE_RES = [RADAR_LRR_Max_RangeRes; RADAR_MRR_Max_RangeRes];
RADAR_AZIMUTH_RES = [RADAR_LRR_Max_AzimuthRes; RADAR_MRR_Max_AzimuthRes];
RADAR_ELEVATION_RES = [RADAR_LRR_Max_ElevationRes; RADAR_MRR_Max_ElevationRes];                                                              
RADAR_RANGE_RATE_RES = [RADAR_LRR_Max_RangeRateRes; RADAR_MRR_Max_RangeRateRes];
RADAR_RANGE_ERR_VAR = [RADAR_LRR_Max_RangeErr; RADAR_MRR_Max_RangeErr];
RADAR_AZIMUTH_ERR_VAR = [RADAR_LRR_Max_AzimuthErr; RADAR_MRR_Max_AzimuthErr];
RADAR_ELEVATION_ERR_VAR = [RADAR_LRR_Max_ElevationErr; RADAR_MRR_Max_ElevationErr];
RADAR_RANGE_RATE_ERR_VAR = [RADAR_LRR_Max_RangeRateErr; RADAR_MRR_Max_RangeRateErr];
RADAR_PD = [RADAR_LRR_Max_ProbOfDetection; RADAR_MRR_Max_ProbOfDetection];
RADAR_FA = [RADAR_LRR_Max_FalseAlarmRate; RADAR_MRR_Max_FalseAlarmRate];
RADAR_FOV_BOUNDARY_PTS_RANGE = [RADAR_LRR_Max_FovRangePts; RADAR_MRR_Max_FovRangePts];
RADAR_FOV_BOUNDARY_PTS_AZIMUTH = [RADAR_LRR_Max_FovAzimuthPts; RADAR_MRR_Max_FovAzimuthPts];  
% ==============================================================================================================================================================
% CAMARA Long Range (Narrow FOV : NFOV) Intrinsic Parameters (Field Of View and Maximum Range)
CAMERA_TYPE_NARROW = uint8(1);
CAM_NARROW_Max_Range = single(150);
CAM_NARROW_Max_Azimuth = single(30);
CAM_NARROW_Max_Elevation = single(-1); % not used 
CAM_NARROW_LongErrVar = single(-1); % not used 
CAM_NARROW_LatErrVar = single(-1); % not used 
CAM_NARROW_ProbOfDetection = single(0.99); % not used 
CAM_NARROW_FalseAlarmRate = single(2/45000); % not used 
CAM_NARROW_FOVRangePoints = single(zeros(1, 20)); % not used 
CAM_NARROW_FOVAzimuthPts = single(zeros(1, 20)); % not used 
% ==============================================================================================================================================================
% CAMARA Short Range (Wide FOV : WFOV) Intrinsic Parameters (Field Of View and Maximum Range)
CAMERA_TYPE_WIDE = uint8(2);
CAM_WIDE_Max_Range = single(70);
CAM_WIDE_Max_Azimuth = single(60);
CAM_WIDE_Max_Elevation = single(-1); % not used 
CAM_WIDE_LongErrVar = single(-1); % not used 
CAM_WIDE_LatErrVar = single(-1); % not used 
CAM_WIDE_ProbOfDetection = single(0.99); % not used 
CAM_WIDE_FalseAlarmRate = single(2/45000); % not used 
CAM_WIDE_FOVRangePoints = single(zeros(1, 20)); % not used 
CAM_WIDE_FOVAzimuthPts = single(zeros(1, 20)); % not used 
% ==============================================================================================================================================================
% CAMARA Intrinsic Parameters (Field Of View and Maximum Range)
CAMERA_MAX_RANGE = [CAM_NARROW_Max_Range; CAM_WIDE_Max_Range];
CAMERA_MAX_AZIMUTH = [CAM_NARROW_Max_Azimuth; CAM_WIDE_Max_Azimuth];
CAMERA_MAX_ELEVATION = [CAM_NARROW_Max_Elevation; CAM_WIDE_Max_Elevation];
CAMERA_LONG_ERR_VAR = [CAM_NARROW_LongErrVar; CAM_WIDE_LongErrVar];
CAMERA_LAT_ERR_VAR = [CAM_NARROW_LatErrVar; CAM_WIDE_LatErrVar];
CAMERA_PD = [CAM_NARROW_ProbOfDetection; CAM_WIDE_ProbOfDetection];
CAMERA_FA = [CAM_NARROW_FalseAlarmRate; CAM_WIDE_FalseAlarmRate];
CAMERA_FOV_BOUNDARY_PTS_RANGE = [CAM_NARROW_FOVRangePoints; CAM_WIDE_FOVRangePoints];
CAMERA_FOV_BOUNDARY_PTS_AZIMUTH = [CAM_NARROW_FOVAzimuthPts; CAM_WIDE_FOVAzimuthPts];                                                                  
% ==============================================================================================================================================================
% Sensor Extrinsic Parameter (X, Y, Z , Yaw mounting parameters) w.r.t ego vehicle center (in Clock Wise Direction)
RAD_X_INSTALL   = single([3.7; 3.7;  -1; -1; -1; 3.7]);
RAD_Y_INSTALL   = single([0;  -0.9; -0.9;  0;  0.9;  0.9]);
RAD_Z_INSTALL   = single([0;   0;     0;     0;  0;    0]); % not set
RAD_YAW_INSTALL = single([0;  -45;   -135; -180; 135;  45]);
RAD_ROLL_INSTALL = single([0;   0;     0;     0;  0;    0]); % not set
RAD_PITCH_INSTALL = single([0;   0;     0;     0;  0;    0]); % not set
RAD_nMeas = uint16([200; 200; 200; 200; 200; 200]);
RAD_TYPE = uint8([RADAR_TYPE_LRR, RADAR_TYPE_MRR, RADAR_TYPE_MRR, RADAR_TYPE_MRR, RADAR_TYPE_MRR, RADAR_TYPE_MRR]);
% ==============================================================================================================================================================
% Camera installs in clockwise direction (Check the order once a data is generated)
CAM_X_INSTALL   = single([1.9; 0;    2.8;     0;  1.38;  2.8;    0;  1.44]);
CAM_Y_INSTALL   = single([0;   0;   -0.9;  -0.9;  -0.9;  0.9;  0.9;   0.9]);
CAM_Z_INSTALL   = single([0;   0;      0;     0;     0;    0;    0;     0]); % not set
CAM_YAW_INSTALL = single([0;  -180;  -60;  -120;   -90;   60;  120;    90]);
CAM_ROLL_INSTALL = single([0;  0;    0;    0;    0;    0;    0;   0]); % not set
CAM_PITCH_INSTALL = single([0; 0;    0;    0;    0;    0;    0;   0]); % not set
CAM_nMeas = uint16([200; 200; 200; 200; 200; 200; 200; 200]);
CAM_TYPE = uint8([CAMERA_TYPE_NARROW, CAMERA_TYPE_NARROW, CAMERA_TYPE_WIDE, CAMERA_TYPE_WIDE, CAMERA_TYPE_WIDE, CAMERA_TYPE_WIDE, CAMERA_TYPE_WIDE, CAMERA_TYPE_WIDE]);
% ==============================================================================================================================================================
% Sensor Layout(SL) module parameters (This module is one time computation which gets actived using a "trigger")
% Sensor Activation Flags for 6 Radars and 8 Cameras (1:Sensor Active, 0:Sensor Inactive)
ACTIVATE_RAD = boolean([1; 1; 1; 1; 1; 1]);
ACTIVATE_CAM = boolean([1; 1; 1; 1; 1; 1; 1; 1]);