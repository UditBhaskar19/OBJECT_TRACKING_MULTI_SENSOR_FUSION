% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : This script creates the data structure and the necessary arrays for Sensor Fusion execution.
%               (A seperate script is written for data struct initialization so that it will be helpful in determining the memory consumption of the system)
% ==============================================================================================================================================================
nMeas = uint16(200); % maximum number of sensor measurements possible (per sensor)
nRadars = uint8(6);  % number of radars installed around the subject vehicle
nCameras = uint8(8); % number of cameras installed around the subject vehicle
nRadarTypes = uint8(2);  % number of types of radars (as of now two types of radars : LRR and MRR)
nCameraTypes = uint8(2); % number of types of cameras (NFOV, WFOV)
nTracks = uint16(100);   % maximum number of estimated object tracks possible
dimVector = int16(2); % measurement dimension for radar and camera measurements (px, vx, py, vy) % only range measurements (px, py) are used
dimInfo = int16(2);   % number of attributes to maintain other info (meas index in CAN and sensor ID)
maxNumMeasRadar = uint16(nRadars)*nMeas;   % maximum number of measurements (all radar sensors)
maxNumMeasCamera = uint16(nCameras)*nMeas; % maximum number of measurements (all camera sensors)
maxNumMeasUpperLimit = max(maxNumMeasRadar, maxNumMeasCamera);
nFOVpoints = uint16(20);   % number of Points sampled on Sensor Field of View boundary (on one side of the FOV symmetry)
nTrackHistoryPoints = uint16(100); % number of trajectory History points
nPointsSengment     = uint16(20);  % number of trajectory History points to fit
nSegments           = nTrackHistoryPoints/nPointsSengment;
% ==============================================================================================================================================================
% Set Ego Sensor Interface (An Array of structure whose purpose is to hold the Ego sensor measurements at current time t)
EGO_CAN_BUS = struct;
EGO_CAN_BUS.detTimeStamp = single(0);
EGO_CAN_BUS.px = single(0);
EGO_CAN_BUS.py = single(0);
EGO_CAN_BUS.vx = single(0);
EGO_CAN_BUS.vy = single(0);
EGO_CAN_BUS.yaw = single(0);
EGO_CAN_BUS.yawRate = single(0);
% ==============================================================================================================================================================
% Set Radar Sensor Interface (An Array of structure whose purpose is to hold the Radar measurements at current time t)
RadarCAN = struct;
RadarCAN.measID = uint16(0);
RadarCAN.sensorID = uint8(0);
RadarCAN.detTimeStamp = single(0);
RadarCAN.px = single(0);
RadarCAN.py = single(0);
RadarCAN.measNoise = single(zeros(dimVector, dimVector));
RadarCAN.snr = single(0);
RADAR_CAN_BUS = RadarCAN(ones(nRadars, nMeas));
% ==============================================================================================================================================================
% Set Camera Sensor Interface (An Array of structure whose purpose is to hold the Radar measurements at current time t)
CameraCAN = struct;
CameraCAN.measID = uint16(0);
CameraCAN.sensorID = uint8(0);
CameraCAN.detTimeStamp = single(0);
CameraCAN.px = single(0);
CameraCAN.py = single(0);
CameraCAN.measNoise = single(zeros(dimVector, dimVector));
CameraCAN.objClassID = uint8(0);
CAMERA_CAN_BUS =  CameraCAN(ones(nCameras, nMeas));
% ==============================================================================================================================================================
% Sensor Extrinsic Parameters Structure (Applicable for both Radar and Camera)
SensorEXTRINSICparam = struct;
SensorEXTRINSICparam.SensorID = uint8(0);
SensorEXTRINSICparam.SensorType = uint8(0);
SensorEXTRINSICparam.isActive = false;
SensorEXTRINSICparam.RotMat2D = single(zeros(2,2));
SensorEXTRINSICparam.TranslationVec = single(zeros(2,1));
SensorEXTRINSICparam.MountX = single(0);
SensorEXTRINSICparam.MountY = single(0);
SensorEXTRINSICparam.MountZ = single(0);
SensorEXTRINSICparam.MountYaw = single(0);
SensorEXTRINSICparam.MountPitch = single(0);
SensorEXTRINSICparam.MountRoll = single(0);
SensorEXTRINSICparam.nMeas = single(0);
RadarEXTRINSICparam = SensorEXTRINSICparam(ones(1, nRadars));
CameraEXTRINSICparam = SensorEXTRINSICparam(ones(1, nCameras));
% ==============================================================================================================================================================
% Radar Intrinsic Parameters Structure
RadarINTRINSICparam = struct;
RadarINTRINSICparam.RadarType = uint8(0);
RadarINTRINSICparam.MaxRange = single(0);
RadarINTRINSICparam.MaxAzimuth = single(0);
RadarINTRINSICparam.MaxElevation = uint8(0);
RadarINTRINSICparam.MaxRangeRate = single(0);
RadarINTRINSICparam.RangeResolution = single(0);
RadarINTRINSICparam.AzimuthResolution = single(0);
RadarINTRINSICparam.ElevationResolution = single(0);
RadarINTRINSICparam.RangeRateResolution = uint8(0);
RadarINTRINSICparam.RangeErrVariance = single(0);
RadarINTRINSICparam.AzimuthErrVariance = single(0);
RadarINTRINSICparam.ElevationErrVariance = uint8(0);
RadarINTRINSICparam.RangeRateErrVariance = single(0);
RadarINTRINSICparam.ProbOfDetection = single(0);
RadarINTRINSICparam.FalseAlarmRate = single(0);
RadarINTRINSICparam.FOVRangePoints = single(zeros(1, nFOVpoints));
RadarINTRINSICparam.FOVAzimuthPts = single(zeros(1, nFOVpoints));
RadarINTRINSICparam = RadarINTRINSICparam(ones(1, nRadarTypes));
% ==============================================================================================================================================================
% Camera Intrinsic Parameters Structure 
CameraINTRINSICparam = struct;
CameraINTRINSICparam.CameraType = uint8(0);
CameraINTRINSICparam.RectificationMatrix = single(0);
CameraINTRINSICparam.ProjectionMatrix = single(0);
CameraINTRINSICparam.MaxRange = single(0);
CameraINTRINSICparam.MaxAzimuth = single(0);
CameraINTRINSICparam.MaxElevation = uint8(0);
CameraINTRINSICparam.LongitudinalErrVariance = single(0);
CameraINTRINSICparam.LateralErrVariance = single(0);
CameraINTRINSICparam.ProbOfDetection = single(0);
CameraINTRINSICparam.FalseAlarmRate = single(0);
CameraINTRINSICparam.FOVRangePoints = single(zeros(1, nFOVpoints));
CameraINTRINSICparam.FOVAzimuthPts = single(zeros(1, nFOVpoints));
CameraINTRINSICparam = CameraINTRINSICparam(ones(1, nCameraTypes));
% ==============================================================================================================================================================
% Radar CALLIBRATION Parameters Structure 
RadarCALLIBRATIONparam = struct;
RadarCALLIBRATIONparam.Intrinsic = RadarINTRINSICparam;
RadarCALLIBRATIONparam.Extrinsic = RadarEXTRINSICparam;
% Radar CALLIBRATION Parameters Structure 
CameraCALLIBRATIONparam = struct;
CameraCALLIBRATIONparam.Intrinsic = CameraINTRINSICparam;
CameraCALLIBRATIONparam.Extrinsic = CameraEXTRINSICparam;
% ==============================================================================================================================================================
% State Structure (estimated kinematics)
StateParam = struct;
StateParam.px = single(0);   % longitudinal position estimate of the track
StateParam.py = single(0);   % lateral position estimate of the track
StateParam.vx = single(0);   % longitudinal velocity estimate of the track 
StateParam.vy = single(0);   % lateral velocity estimate of the track 
StateParam.ax = single(0);   % longitudinal acceleration estimate of the track
StateParam.ay = single(0);   % lateral acceleration estimate of the track
StateParam.ErrCOV = single(zeros(6,6));  % Track error covariance of the estimate
% Track state errors (errors in the estimated kinematics)
StateParamAccuracy = struct;
StateParamAccuracy.px3Sigma = single(0);     % 3 sigma error for the longitudinal position estimate of the track
StateParamAccuracy.py3Sigma = single(0);     % 3 sigma error for the lateral position estimate of the track
StateParamAccuracy.vx3Sigma = single(0);     % 3 sigma error for the longitudinal velocity estimate of the track 
StateParamAccuracy.vy3Sigma = single(0);     % 3 sigma error for the lateral velocity estimate of the track 
StateParamAccuracy.ax3Sigma = single(0);     % 3 sigma error for the longitudinal acceleration estimate of the track 
StateParamAccuracy.ay3Sigma = single(0);     % 3 sigma error for the lateral acceleration estimate of the track   
% parameters from derived from camera/Radar detections (bounding box info)
StateParamCAM = struct;
StateParamCAM.Classification = int16(0);             % classification of the object/bounding box
StateParamCAM.ClassificationScore = single(0);       % classification score of the object
StateParamCAM.BoundingBoxDim = single(zeros(3,1));   % in the order of length width and height
StateParamCAM.BoundingBoxPoint = single(zeros(3,1)); % in the order of X, Y, Z
% Sensor Source of the Track (which sensors contributed to the track estimation)
SensorSource = struct;
SensorSource.RadarSource = false(1, nRadars);    % radar sensor sources that contributed to state update 
SensorSource.CameraSource = false(1, nCameras);  % camera sensor sources that contributed to state update
SensorSource.RadarCatch = false;                 % is the state updated from radar measurements
SensorSource.CameraCatch = false;                % is the state updated from the camera measurements
SensorSource.RadarAndCameraCatch = false;        % is the state updated from both radar and camera measurements
% Track status (is Occluded, is Out Of FOV, is in prediction mode, is lost , is stationary, is an obstacle etc)
isTrack = struct;
isTrack.New = false;         % Indicates if a track is not confirmed for a newly appeared object
isTrack.Existing = false;    % Indicates if the track is confirmed for an object, and is updated with a measurement
isTrack.Lost = false;        % Indicates if the track it is not of interest anymore and needs to be deleted
isTrack.Predicted = false;   % Indicates if the track is predicted (not gated or updated with any measurements)
isTrack.Gated = false;       % Indicate  if the track is gated with any measurement 
isTrack.OutOfFov = false;    % Indicates if the track is out of FoV of all active sensors
isTrack.Occluded = false;    % Indicates if the track is occluded
isTrack.Stationary = false;  % Indicates if the object track is stationary
isTrack.SlowMoving = false;  % Indicates if the object track is moving slowly
isTrack.Moving = false;      % Indicates if the object track is moving  
isTrack.Obstacle = false;    % Indicates if the track is a part of static environment (guardrails, barriers, parked-vehicles, etc)
isTrack.Object = false;      % Indicates if the track is capable of moving independently (pedstrians, 2-wheelers, 4-wheelers etc)
% Track Quality 
Track.TrackedTime = single(-SamplingTime);   % Total tracked time in sec
Track.RegionConfidence = single(0);          % Region Confidence of the Track 
Track.Quality = uint16(0);                   % Track estimate quality
Track.PredictedCounter = uint16(0);          % number of times the track got predicted continuously 
Track.GatedCounter = uint16(0);              % number of times the track got gated within a certain time interval 
% ==============================================================================================================================================================
% Track parameters  
TrackParam = struct; 
TrackParam.id = uint16(0);                             % Track ID
TrackParam.StateEstimate = StateParam;                 % Track estimates
TrackParam.StateEstimateAccuracy = StateParamAccuracy; % Track estimate accuracy
TrackParam.BoundingBoxInfo = StateParamCAM;            % Track box info
TrackParam.SensorSource = SensorSource;                % Sensor source for the Track estimate 
TrackParam.Status = isTrack;                           % Track status parameters
TrackParam.Quality = Track;                            % Track Quality
% ==============================================================================================================================================================
% Track Data Structure
TRACK_ESTIMATES = struct;
TRACK_ESTIMATES.nValidTracks = uint16(0);                  % number of valid tracks in time t
TRACK_ESTIMATES.TrackParam = TrackParam(ones(1, nTracks)); % Track estimated/computed parameters
% ==============================================================================================================================================================
% Track Indexes for track management
TRACK_ESTIMATES.TrackIDList = uint16(1:1:nTracks);       % list of all possible track ids
TRACK_ESTIMATES.IsTrackIDused = false(1,nTracks);        % are there track ids assigned to any track 
TRACK_ESTIMATES.FirstAvailableIDindex = uint16(1);       % index to the first available track id (track ids not currently used by existing tracks) 
TRACK_ESTIMATES.LastAvailableIDindex = uint16(nTracks);  % index to the last available track id (track ids not currently used by existing tracks) 
% ==============================================================================================================================================================
% Data structure to compute trajectory history buffer
TRACK_HISTORY = struct;
TRACK_HISTORY.id = uint16(0);
TRACK_HISTORY.TrackEstRefIdx   = uint16(0);
TRACK_HISTORY.BufferStartIndex = uint16(0);                     % number of elements held in the buffer
TRACK_HISTORY.WriteIndex       = uint16(1);                     % buffer array index where the latest element has to be copied
TRACK_HISTORY.SegmentLength    = single(0);                      % trajectory segment length 
TRACK_HISTORY.Length           = single(0);                      % trajectory length 
TRACK_HISTORY.isInitialized    = false;                          % is the trajectory buffer initialized 
TRACK_HISTORY.HistoryBufferPx  = single(zeros(1,nTrackHistoryPoints));
TRACK_HISTORY.HistoryBufferPy  = single(zeros(1,nTrackHistoryPoints));
TRACK_HISTORY.HistoryBufferVx  = single(zeros(1,nTrackHistoryPoints));
TRACK_HISTORY.HistoryBufferVy  = single(zeros(1,nTrackHistoryPoints));
TRACK_HISTORY.HistoryBufferAx  = single(zeros(1,nTrackHistoryPoints));
TRACK_HISTORY.HistoryBufferAy  = single(zeros(1,nTrackHistoryPoints));
TRACK_HISTORY.SegmentedPolynomialCoefficients = single(zeros(5, nSegments));     % 4 3rd degree polynomial coefficients : curve length, c0, c1, c2, c3
TRACK_HISTORY.SegmentedClothoidCoefficients   = single(zeros(5, nSegments));     % curve length, initial offset , heading, curvature and curvature rate
TRACK_HISTORY.ClothoidCoefficients            = single(zeros(1,4+nSegments));    % merged clothoid curve coefficients : init offset, 
TRACK_HISTORY.SegmentAvailableToFit  = uint16(0);                       % indicates how many segments are available to fit 
TRACK_HISTORY = TRACK_HISTORY(ones(1,nTracks));
% ==============================================================================================================================================================
% Tracjectory History
TRAJECTORY_HISTORY = struct;
TRAJECTORY_HISTORY.nConfirmedTracks = uint16(0);
TRAJECTORY_HISTORY.TRACK_HISTORY = TRACK_HISTORY; 
% ==============================================================================================================================================================
% for Track to Track fusion
TRACK_ESTIMATES_RAD = TRACK_ESTIMATES;   % track estimates from the radar sensor
TRACK_ESTIMATES_CAM = TRACK_ESTIMATES;   % track estimates from the camera sensor
TRACK_ESTIMATES_FUS = TRACK_ESTIMATES;   % track fusion estimates from the radar and camera estimated tracks 
% ==============================================================================================================================================================
% Structure of Sensor Measurement matrix :
% MeasArray : Sensor measurement matrix holding coordinate transformed measurements for all sensor measuurements : 
%           : size : (meas_dim, max_num_of_meas_all_sensors)
% MeasCovariance : Sensor measurement matrix holding coordinate transformed meas noise covariance for all sensor measuurements : 
%                : size : (meas_dim, meas_dim, max_num_of_meas_all_sensors)
% MeasRef : Sensor measurement reference matrix holding measurement index and sensor index of the measurement CAN bus
%         : size : (2, max_num_of_meas_all_sensors)
% ValidMeasCount : maximum number of valid measurements returned by each of the sensors
%                : size : (1, number of sensors)
% ValidCumulativeMeasCount : vector of cumulative count of 'ValidMeasCount'
%                          : size : (1, number of sensors)
RADAR_MEAS_CTS = struct;
RADAR_MEAS_CTS.MeasArray = single(zeros(dimVector , uint16(nRadars)*nMeas));              
RADAR_MEAS_CTS.MeasCovariance = single(zeros(dimVector, dimVector, uint16(nRadars)*nMeas));
RADAR_MEAS_CTS.MeasRef = uint16(zeros(dimInfo , uint16(nRadars)*nMeas));
RADAR_MEAS_CTS.ValidMeasCount = uint16(zeros(1,nRadars));
RADAR_MEAS_CTS.ValidCumulativeMeasCount = uint16(zeros(1,nRadars));

CAMERA_MEAS_CTS = struct;
CAMERA_MEAS_CTS.MeasArray = single(zeros(dimVector, uint16(nCameras)*nMeas));
CAMERA_MEAS_CTS.MeasCovariance = single(zeros(dimVector, dimVector, uint16(nCameras)*nMeas));
CAMERA_MEAS_CTS.MeasRef = uint16(zeros(dimInfo , uint16(nCameras)*nMeas));
CAMERA_MEAS_CTS.ValidMeasCount = uint16(zeros(1,nCameras));
CAMERA_MEAS_CTS.ValidCumulativeMeasCount = uint16(zeros(1,nCameras));
% ==============================================================================================================================================================
% Clustering Output Structure ( Radar )
RADAR_CLUSTERS = struct;
RADAR_CLUSTERS.nClusters = uint16(0);                                                        % number of Radar clusters
RADAR_CLUSTERS.ClusterSizes = uint16(zeros(1,maxNumMeasRadar));                              % Radar cluster sizes (number of measurements forming a cluster)
RADAR_CLUSTERS.ClusterCenters = single(zeros(dimVector, maxNumMeasRadar));                   % Radar cluster centers
RADAR_CLUSTERS.ClusterCovariance = single(zeros(dimVector, dimVector, maxNumMeasRadar));     % Radar cluster center errror covariance
RADAR_CLUSTERS.ClusterIDs = uint16(zeros(1,maxNumMeasRadar));                                % Radar cluster Id
RADAR_CLUSTERS.ClustIDAssig = uint16(zeros(1,maxNumMeasRadar));                              % Radar measurement to Radar cluster ID assignment vector
% ==============================================================================================================================================================
% Clustering Output Structure ( Camera )
CAMERA_CLUSTERS = struct;                                                                    
CAMERA_CLUSTERS.nClusters = uint16(0);                                                       % number of Camera clusters
CAMERA_CLUSTERS.ClusterSizes = uint16(zeros(1,maxNumMeasCamera));                            % Camera cluster sizes (number of measurements forming a cluster)
CAMERA_CLUSTERS.ClusterCenters = single(zeros(dimVector, maxNumMeasCamera));                 % Camera cluster centers
CAMERA_CLUSTERS.ClusterCovariance = single(zeros(dimVector, dimVector, maxNumMeasRadar));    % Camera cluster center errror covariance
CAMERA_CLUSTERS.ClusterIDs = uint16(zeros(1,maxNumMeasCamera));                              % Camera cluster Id
CAMERA_CLUSTERS.ClustIDAssig = uint16(zeros(1,maxNumMeasCamera));                            % Camera measurement to Radar cluster ID assignment vector
% ==============================================================================================================================================================
% Clustering Output Structure ( Fused Cluster : Radar + Camera )
% each cluster can have either camera only measurements, radar only measurements, both camera and radar measurements
MERGED_CLUSTERS = struct;                                                                    
MERGED_CLUSTERS.nClusters = uint16(0);                                                         % Total number of measurement clusters
MERGED_CLUSTERS.ClusterCenters = single(zeros(dimVector, maxNumMeasUpperLimit));               % Merged cluster centers
MERGED_CLUSTERS.ClusterCovariance = single(zeros(dimVector, dimVector, maxNumMeasUpperLimit)); % Merged cluster center errror covariance
MERGED_CLUSTERS.ClusterIDs = uint16(zeros(1,maxNumMeasUpperLimit));                            % Merged cluster Id
MERGED_CLUSTERS.LookUpToRadClstrID = uint16(zeros(1,maxNumMeasUpperLimit));                    % Radar Cluster ID to Merged cluster ID assignment vector
MERGED_CLUSTERS.LookUpToCamClstrID = uint16(zeros(1,maxNumMeasUpperLimit));                    % Camera Cluster ID to Merged cluster ID assignment vector.
% ==============================================================================================================================================================
% Unassociated Clusters 
UNASSOCIATED_CLUSTERS_RAD = uint16(zeros(1,maxNumMeasRadar));      % Unassociated Radar clusters to track
UNASSOCIATED_CLUSTERS_CAM = uint16(zeros(1,maxNumMeasCamera));     % Unassociated Camera clusters to track
% ==============================================================================================================================================================
% Cluster Manage Structure (RADAR)
RAD_CLST = struct;
RAD_CLST.measTypeCore = false(1,maxNumMeasRadar);
RAD_CLST.clusterMember = uint16(zeros(1,maxNumMeasRadar));  
RAD_CLST.clusterMemberFlag = false(1,maxNumMeasRadar); 
RAD_CLST.measurementVisited = false(1,maxNumMeasRadar);
% ==============================================================================================================================================================
% Cluster Manage Structure (CAMERA)  
CAM_CLST = struct;
CAM_CLST.sizeClust = uint16(0);
CAM_CLST.ClusterID = uint16(0);
CAM_CLST.clusterMember = uint16(zeros(1,maxNumMeasCamera));
CAM_CLST.measurementVisited = false(1,maxNumMeasCamera);
% ==============================================================================================================================================================
% Cluster Manage Structure (MERGE)
FUS_CLST = struct;
FUS_CLST.RadarClstAdded = false(1,maxNumMeasUpperLimit); 
FUS_CLST.RadarClstrMemberList = uint16(zeros(1,maxNumMeasUpperLimit)); 
% ==============================================================================================================================================================
% MANAGE CLUSTER
MANAGE_CLUSTERS = struct;
MANAGE_CLUSTERS.RAD = RAD_CLST;
MANAGE_CLUSTERS.CAM = CAM_CLST;
MANAGE_CLUSTERS.FUSE = FUS_CLST;
% ==============================================================================================================================================================
% Boolean Flag array of gated measurement indexes
GATED_MEAS_INDEX_RAD = false(1,maxNumMeasRadar);
GATED_MEAS_INDEX_CAM = false(1,maxNumMeasCamera);
GATED_CLUSTER_INDEX_RAD = false(1,maxNumMeasRadar);
GATED_CLUSTER_INDEX_CAM = false(1,maxNumMeasCamera);
% Track To Measurement Association Matrix
INV = single(-99);
ASSOCIATION_MAT_RADAR = INV*single(zeros(nTracks, (nTracks + maxNumMeasRadar)));
ASSOCIATION_MAT_CAMERA = INV*single(zeros(nTracks, (nTracks + maxNumMeasCamera)));
ASSOCIATION_MAT_UNASSOCIATED_CLUSTER_IDs = uint16(zeros(1,maxNumMeasUpperLimit));
% Measurement To Track Assignment Matrix
ASSIGNMENT_MAT = struct;
ASSIGNMENT_MAT.AssociationMat = single(zeros(nTracks, (nTracks + nMeas)));
ASSIGNMENT_MAT.nMeas = uint16(0);
ASSIGNMENT_MAT_RADAR = ASSIGNMENT_MAT(ones(1,nRadars));
ASSIGNMENT_MAT_CAMERA = ASSIGNMENT_MAT(ones(1,nCameras));


Components = struct;
Components.x = single(zeros(4,1));
Components.P = single(zeros(4,4));
MixtureComponents = Components(ones(1,nMeas));

FusedStates = struct;
FusedStates.x = single(zeros(4,1));
FusedStates.P = single(zeros(4,4));


FUSION_INFO = struct;
FUSION_INFO.Beta = single(zeros(1,nMeas));
FUSION_INFO.BetaSum = single(0);
FUSION_INFO.MixtureComponents = MixtureComponents;
FUSION_INFO.nHypothesis = uint16(0);
FUSION_INFO.x = single(zeros(4,1));
FUSION_INFO.P = single(zeros(4,4));
FUSION_INFO_RAD = FUSION_INFO(ones(nTracks, nRadars));
FUSION_INFO_CAM = FUSION_INFO(ones(nTracks, nCameras));
% ==============================================================================================================================================================











