% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : This script creates the data structure and the necessary arrays for Sensor Fusion execution.
%               (A seperate script is written for data struct initialization so that it will be helpful in 
%               determining the memory consumption of the system)
% ==============================================================================================================================================================
nLineMeas = uint16(2);       % maximum number of sensor line measurements possible (per sensor)
nLineCameras = uint8(1);     % number of cameras installed around subject vehicle
nCameraTypes = uint8(2);     % number of types of cameras (NFOV, WFOV)
nLineTracks = 10;            % maximum number of lane line tracks
dimLineVector = int16(5);    % measurement dimension Lane Line measurements (Offset, alpha, curv, curvRate, curvLength)
dimLineInfo = 2;             % to maintain other info (meas index in CAN and sensor ID)
maxNumLineMeasCamera = uint16(nLineCameras)*nLineMeas; % maximum number of measurements (all camera sensors)
% ==============================================================================================================================================================
% Set Camera Sensor Interface (An Array of structure whose purpose is to hold the Radar measurements at current time t)
CameraLineCAN = struct;
CameraLineCAN.LineID = uint16(0);
CameraLineCAN.sensorID = uint8(0);
CameraLineCAN.detTimeStamp = single(0);
CameraLineCAN.LateralOffset = single(0);
CameraLineCAN.HeadingAngle = single(0);
CameraLineCAN.Curvature = single(0);
CameraLineCAN.CurvatureDerivative = single(0);
CameraLineCAN.CurveLength = single(0);
CameraLineCAN.LineWidth = single(0);
CameraLineCAN.MaximumValidX = single(0);
CameraLineCAN.MinimumValidX = single(0);
CAMERA_CAN_LANE_LINE_BUS =  CameraLineCAN(ones(nLineCameras, nLineMeas));
% ==============================================================================================================================================================
% Clothoid State Structure
ClothoidParam = struct;
ClothoidParam.LateralOffset = single(0);
ClothoidParam.HeadingAngle = single(0);
ClothoidParam.Curvature = single(0);
ClothoidParam.CurvatureDerivative = single(0);
ClothoidParam.ErrCOV = single(0.5*eye(4));
ClothoidParam.CurveLength = single(0);
% Polynomial State Structure
PolynomialParam = struct;
PolynomialParam.a0 = single(0);
PolynomialParam.a1 = single(0);
PolynomialParam.a2 = single(0);
PolynomialParam.a3 = single(0);
PolynomialParam.ErrCOV = single(zeros(4,4));
% Track state errors
LineParamAccuracy = struct;
LineParamAccuracy.c0Sigma = single(0);
LineParamAccuracy.c1Sigma = single(0);
LineParamAccuracy.c2Sigma = single(0);
LineParamAccuracy.c3Sigma = single(0);
LineParamAccuracy.lenSigma = single(0);
% Sensor Source of the Track (which sensors contributed to the track estimation)
SensorSource = struct;
SensorSource.CameraSource = false(1, nLineCameras);  % camera sensor sources that contributed to state update
SensorSource.CameraCatch = false; % is the state updated from the camera measurements
% Track status (is Occluded, is Out Of FOV, is in prediction mode, is lost , is stationary, is an obstacle etc)
isLineTrack = struct;
isLineTrack.New = false;           
isLineTrack.Predicted = false;
isLineTrack.OutOfFov = false;
isLineTrack.Occluded = false;
% Track Quality 
LineTrack = struct;
LineTrack.TrackedTime = single(0);
LineTrack.Confidence = single(0);
LineTrack.Quality = uint16(0);
% ==============================================================================================================================================================
% Track parameters
LaneTrackParam = struct;
LaneTrackParam.id = int16(0);
LaneTrackParam.ClothoidParam = ClothoidParam;
LaneTrackParam.PolynomialParam = PolynomialParam;
LaneTrackParam.LineParamAccuracy = LineParamAccuracy;
LaneTrackParam.SensorSource = SensorSource;
LaneTrackParam.isLineTrack = isLineTrack;
LaneTrackParam.LineTrack = LineTrack;
LaneTrackParam = LaneTrackParam(ones(1, nLineTracks));
% ==============================================================================================================================================================
% Track Data Structure
LANE_TRACK_ESTIMATES = struct;
LANE_TRACK_ESTIMATES.LaneTrackParam = LaneTrackParam;
LANE_TRACK_ESTIMATES.nValidTracks = uint16(0);
% ==============================================================================================================================================================
LANE_TRACK_BOUNDARY = LANE_TRACK_ESTIMATES;
LANE_TRACK_EGO_LANE = LANE_TRACK_ESTIMATES;
% ==============================================================================================================================================================
EGO_LANE_CENTER_TRACK_HYPOTHESIS = struct;
EGO_LANE_CENTER_TRACK_HYPOTHESIS.nValidTracks = single(0);
EGO_LANE_CENTER_TRACK_HYPOTHESIS.ClothoidParam = ClothoidParam(ones(1,nLineTracks));
% ==============================================================================================================================================================
ROAD_PROFILE = struct;
ROAD_PROFILE.EGO_LANE_CENTER_TRACK_ESTIMATION  = ClothoidParam;
ROAD_PROFILE.EGO_LANE_NEXT_NEXT_RIGHT_BOUNDARY = ClothoidParam;
ROAD_PROFILE.EGO_LANE_NEXT_RIGHT_BOUNDARY      = ClothoidParam;
ROAD_PROFILE.EGO_LANE_RIGHT_BOUNDARY           = ClothoidParam;
ROAD_PROFILE.EGO_LANE_LEFT_BOUNDARY            = ClothoidParam;
ROAD_PROFILE.EGO_LANE_NEXT_LEFT_BOUNDARY       = ClothoidParam;
ROAD_PROFILE.EGO_LANE_NEXT_NEXT_LEFT_BOUNDARY  = ClothoidParam;
% ==============================================================================================================================================================
nPtsFront = 100;
nPtsRear = 200;
n = nPtsFront + nPtsRear;
LineGeometry = struct;
LineGeometry.X = single(zeros(1,n));
LineGeometry.Y = single(zeros(1,n));
LineGeometry.arcLength = single(zeros(1,n));
LineGeometry.HeadingAngle = single(zeros(1,n));
LineGeometry.Curvature = single(zeros(1,n));
LineGeometry.CurvatureDerivative = single(zeros(1,n));
LineGeometry.validCurvLenFront = single(0);
LineGeometry.validCurvLenRear = single(0);
LineGeometry.BufferLastValidIdx = uint16(0);
ROAD_GEOMETEY = struct;
ROAD_GEOMETRY.EgoLaneCenter = LineGeometry;
ROAD_GEOMETRY.EgoLaneRightBoundary = LineGeometry;
ROAD_GEOMETRY.EgoLaneNxtRightBoundary = LineGeometry;
ROAD_GEOMETRY.EgoLaneNxtNxtRightBoundary = LineGeometry;
ROAD_GEOMETRY.EgoLaneLeftBoundary = LineGeometry;
ROAD_GEOMETRY.EgoLaneNxtLeftBoundary = LineGeometry;
ROAD_GEOMETRY.EgoLaneNxtNxtLeftBoundary = LineGeometry;
% ==============================================================================================================================================================
% Structure of Lane Line Measurement matrix :
% MeasArray : Lane Line Measurement matrix holding valid line measurements for all sensors: 
%           : size : (meas_dim, max_num_of_meas_all_sensors)
% MeasRef : Sensor measurement reference matrix holding measurement index and sensor index of the measurement CAN bus
%         : size : (2, max_num_of_meas_all_sensors)
% ValidMeasCount : maximum number of valid measurements returned by each of the sensors
%                : size : (1, number of sensors)
% ValidCumulativeMeasCount : vector of cumulative count of 'ValidMeasCount'
%                          : size : (1, number of sensors)
LANE_LINE_MEAS_MAT = struct;
LANE_LINE_MEAS_MAT.MeasArray = single(zeros(dimLineVector , uint16(nLineCameras)*nLineMeas));              
LANE_LINE_MEAS_MAT.MeasRef = uint16(zeros(dimLineInfo , uint16(nLineCameras)*nLineMeas));
LANE_LINE_MEAS_MAT.ValidMeasCount = uint16(zeros(1,nLineCameras));
LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount = uint16(zeros(1,nLineCameras));
% ==============================================================================================================================================================
% Clustering Output Structure ( Camera )
LANE_LINES_CLUSTERS = struct;                                                                    
LANE_LINES_CLUSTERS.nClusters = uint16(0);                               % number of Camera clusters
LANE_LINES_CLUSTERS.ClusterSizes = uint16(zeros(1,maxNumLineMeasCamera));    % Camera cluster sizes (number of measurements forming a cluster)
LANE_LINES_CLUSTERS.ClusterIDs = uint16(zeros(1,maxNumLineMeasCamera));      % Camera cluster Id
LANE_LINES_CLUSTERS.ClustIDAssig = uint16(zeros(1,maxNumLineMeasCamera));    % Camera measurement to Radar cluster ID assignment vector
% ==============================================================================================================================================================
% Cluster Manage Structure (Lines)  
LANE_LINES_CLST_MANAGE = struct;
LANE_LINES_CLST_MANAGE.sizeClust = uint16(0);
LANE_LINES_CLST_MANAGE.ClusterID = uint16(0);
LANE_LINES_CLST_MANAGE.clusterMember = uint16(zeros(1,maxNumLineMeasCamera));
LANE_LINES_CLST_MANAGE.measurementVisited = false(1,maxNumLineMeasCamera);
% ==============================================================================================================================================================
% Unassociated Clusters 
UNASSOCIATED_CLUSTERS_LINE = uint16(zeros(1,maxNumLineMeasCamera));      % Unassociated Line clusters to track
% ==============================================================================================================================================================
% Boolean Flag array of gated measurement indexes
GATED_LINE_INDEX = false(1,maxNumLineMeasCamera);
LINE_ASSIGNMENT_MAT = struct;
LINE_ASSIGNMENT_MAT.LogLikelihoodMat = single(zeros(1,maxNumLineMeasCamera));
LINE_ASSIGNMENT_MAT.isGatedMat = false(1,maxNumLineMeasCamera);
LINE_ASSIGNMENT_MAT.nMeas = single(0);
LINE_ASSIGNMENT_MAT = LINE_ASSIGNMENT_MAT(ones(1,nLineTracks));








