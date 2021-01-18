% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : Main Script for Object Tracking by sensor Fusion (Excluding Road Geometry Estimation)
% ==============================================================================================================================================================
% Run the below scripts (in this sequence)
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script1_LOAD_SIMULATION_DATA.m');
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script2_RESTRUCTURE_SIMULATION_DATA.m');
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script3_LOAD_DATA_STRUCTURE_PARAMETERS.m');
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script4_SET_SENSOR_CONFIG_PARAMETERS.m');
addpath('Library');
% ==============================================================================================================================================================
% Simulation Parameters
dT = SamplingTime; %in sec (50 millisec)
nTimeSample = SimulationTime/dT + 1;
% =======================================>  SET MODEL PARAMETERS% <=============================================================================================
sigmaQ = 1;
MOTION_MODEL_CV = MODEL.cvmodel(dT, sigmaQ);     % set the process model as constant velocity model
MEAS_MODEL_CV = MODEL.cvmeasmodelPxPy();         % set the observation model
GammaSq = 9.2103; P_G = 0.99;                       
epsPosDBSCAN = single(5);                        % radar measurement clustering euclidean distance threshold
epsPosNN = single(4);                            % camera measurement clustering euclidean distance threshold
epsPosJOIN = single(4);                          % radar and camera cluster grouping euclidean distance threshold
ANTICLOCKWISE_ROT = single(1);
CLOCKWISE_ROT = single(-1);
ROT_CONVENTION = ANTICLOCKWISE_ROT;
% =======================================>  SET SENSOR INSTALLATION PARAMETERS <================================================================================
RadarCALLIBRATIONparam.Intrinsic = SENSOR_LAYOUT.setRadarIntrinsicParam(nRadarTypes, ...
                                                          RADAR_MAX_RANGE, RADAR_MAX_AZIMUTH, RADAR_MAX_ELEVATION, RADAR_MAX_RANGE_RATE, ...
                                                          RADAR_RANGE_RES, RADAR_AZIMUTH_RES, RADAR_ELEVATION_RES, RADAR_RANGE_RATE_RES, ...
                                                          RADAR_RANGE_ERR_VAR, RADAR_AZIMUTH_ERR_VAR, RADAR_ELEVATION_ERR_VAR, RADAR_RANGE_RATE_ERR_VAR, ...
                                                          RADAR_PD, RADAR_FA, ...
                                                          RADAR_FOV_BOUNDARY_PTS_RANGE, RADAR_FOV_BOUNDARY_PTS_AZIMUTH, ...
                                                          RadarCALLIBRATIONparam.Intrinsic);

RadarCALLIBRATIONparam.Extrinsic = SENSOR_LAYOUT.setSensorExtrinsicParam(nRadars, RAD_TYPE, ACTIVATE_RAD, ...
                                                          RAD_X_INSTALL, RAD_Y_INSTALL, RAD_Z_INSTALL, ...
                                                          RAD_ROLL_INSTALL, RAD_PITCH_INSTALL, RAD_YAW_INSTALL, ...
                                                          RAD_nMeas, RadarCALLIBRATIONparam.Extrinsic, ROT_CONVENTION);
                                                                                                             
CameraCALLIBRATIONparam.Intrinsic = SENSOR_LAYOUT.setCameraIntrinsicParam(nCameraTypes, ...
                                                          CAMERA_MAX_RANGE, CAMERA_MAX_AZIMUTH, CAMERA_MAX_ELEVATION, ...
                                                          CAMERA_LONG_ERR_VAR, CAMERA_LAT_ERR_VAR, ...
                                                          CAMERA_PD, CAMERA_FA, ...
                                                          CAMERA_FOV_BOUNDARY_PTS_RANGE, CAMERA_FOV_BOUNDARY_PTS_AZIMUTH, ...
                                                          CameraCALLIBRATIONparam.Intrinsic);

CameraCALLIBRATIONparam.Extrinsic = SENSOR_LAYOUT.setSensorExtrinsicParam(nCameras, CAM_TYPE, ACTIVATE_CAM, ...
                                                          CAM_X_INSTALL, CAM_Y_INSTALL, CAM_Z_INSTALL, ...
                                                          CAM_ROLL_INSTALL, CAM_PITCH_INSTALL, CAM_YAW_INSTALL, ...
                                                          CAM_nMeas, CameraCALLIBRATIONparam.Extrinsic, ROT_CONVENTION);
% ==============================================================================================================================================================
FOVPtsEGOframeRAD = SENSOR_LAYOUT.TransformFOVtoEGOframe(RadarCALLIBRATIONparam, nRadars, nRadarTypes, 100);
FOVPtsEGOframeCAM = SENSOR_LAYOUT.TransformFOVtoEGOframe(CameraCALLIBRATIONparam, nCameras, nCameraTypes, 100);
VIZ.VisualizeSensorLayout(FOVPtsEGOframeRAD, FOVPtsEGOframeCAM, nRadars, nCameras);
ExecutionCycleTime_FUS = single(zeros(nTimeSample,1));
ExecutionCycleTime_CAM = single(zeros(nTimeSample,1));
ExecutionCycleTime_RAD = single(zeros(nTimeSample,1));
disp('Press any Key to Continue');
pause();
% ============================================================>   START THE OBJECT DETECTION    <===============================================================
for t = 1:nTimeSample
    
    
    
    % -----------------------------------------------> GET RADAR & CAMERA SENSOR DATA AS ARRAY OF STRUCTURE <---------------------------------------------------
    % 1) - A) get data in the form of structure array (RADAR)
    RADAR_CAN_BUS =  SENSOR_INTERFACE.RAD_SENSOR_INTERFACE(RAD1_Sensor_Simulated_Data, ...
                                                           RAD2_Sensor_Simulated_Data, ...
                                                           RAD3_Sensor_Simulated_Data, ...
                                                           RAD4_Sensor_Simulated_Data, ...
                                                           RAD5_Sensor_Simulated_Data, ...
                                                           RAD6_Sensor_Simulated_Data, ...
                                                           RADAR_CAN_BUS, t, nRadars, nMeas);
    % 1) - B) get data in the form of structure array (CAMERA)
    CAMERA_CAN_BUS = SENSOR_INTERFACE.CAM_SENSOR_INTERFACE(CAM1_Sensor_Simulated_Data, ...
                                                           CAM2_Sensor_Simulated_Data, ...
                                                           CAM3_Sensor_Simulated_Data, ...
                                                           CAM4_Sensor_Simulated_Data, ...
                                                           CAM5_Sensor_Simulated_Data, ...
                                                           CAM6_Sensor_Simulated_Data, ...
                                                           CAM7_Sensor_Simulated_Data, ...
                                                           CAM8_Sensor_Simulated_Data, ...
                                                           CAMERA_CAN_BUS, t, nCameras, nMeas);
    % 1) - C) get data into the CAN BUS (EGO)
    EGO_CAN_BUS = SENSOR_INTERFACE.EGO_SENSOR_INTERFACE(EGO_Sensor_Simulated_Data, EGO_CAN_BUS, t);
    % ========================================================================================================================================================== 
    % --------------------------------------------> COORDINATE TRANSFORMATION OF RADAR & CAMERA MEASUREMENTS <--------------------------------------------------
    tic
    % 2) - A) Coordinate Transformation of the Radar measurements
    RADAR_MEAS_CTS = SPATIAL_ALLIGNMENT.CTS_SENSOR_FRAME_TO_EGO_FRAME(nRadars, nMeas, RadarCALLIBRATIONparam.Extrinsic, RADAR_CAN_BUS, RADAR_MEAS_CTS);
    % 2) - B) Coordinate Transformation of the Camera measurements
    CAMERA_MEAS_CTS = SPATIAL_ALLIGNMENT.CTS_SENSOR_FRAME_TO_EGO_FRAME(nCameras, nMeas, CameraCALLIBRATIONparam.Extrinsic, CAMERA_CAN_BUS, CAMERA_MEAS_CTS);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % ===================================================> TRACK ESTIMATIONS FROM RADAR MEASUREMENTS <==========================================================
    % The following Steps are for Tracks estimation using RADAR Sensors Only (RADAR TRACKS)
    % The steps are as follows :
    % 1. Cluster the Concatenated Radar measurements (All radars) by DBSCAN Clustering Algorithm, 
    %    where each cluster corresponds to a specific object ( Traffic Object like vehicle, pedestrian etc)
    % 2. Recompute the Cluster Statistics (Mean and Covariance) WITH RESPECT TO EACH of the radars
    % 3. Perform state prediction of the object tracks from time t-1 to t (tracks are detected by th RADAR sensors only)
    % 4. Perform Gating of radar clusters from each radars with the predicted tracks
    % 5. Perform Data association , radar sensor fusion and state estimation 
    % 6. Manage Radar Tracks by a Radar Track manager
    % ==========================================================================================================================================================
    % ---------------------------------------------------------->  RADAR MEASUREMENT CLUSTERING  <--------------------------------------------------------------
    % 3) - A) Cluster Radar measurements
    RADAR_CLUSTERS  = CLUSTERING.CLUSTER_RADAR_MEASUREMENTS_DBSCAN(RADAR_MEAS_CTS, MANAGE_CLUSTERS.RAD, RADAR_CLUSTERS, epsPosDBSCAN);
    RADAR_MEAS_CLUSTER = CLUSTERING.SEGREGATE_CLUSTER(RADAR_MEAS_CTS, RADAR_CLUSTERS);
    % ==========================================================================================================================================================
    % ----------------------------------------------------->  STATE PREDICTION OF RADAR OBJECTS/TRACKS  <-------------------------------------------------------
    % 4) - A) State Prediction of the object Tracks (Step 1: Ego Compensation of the tracks )
    TRACK_ESTIMATES_RAD = STATE_PREDICTOR.EGO_COMPENSATION(TRACK_ESTIMATES_RAD, EGO_CAN_BUS, dT, 'none');
    % 4) - B) State Prediction of the object Tracks (Step 2: Constant Velocity based Prediction (CV : Linear Motion Model) )
    TRACK_ESTIMATES_RAD = STATE_PREDICTOR.LINEAR_PROCESS_MODEL(TRACK_ESTIMATES_RAD, MOTION_MODEL_CV);
    % ==========================================================================================================================================================
    % ------------------------------------------->  RADAR MEASUREMENT CLUSTER GATING & IDENTIFY UNGATED CLUSTERS <----------------------------------------------
    % 5) - A) Gating of RADAR CLUSTERS
    [ASSIGNMENT_MAT_RADAR, GATED_CLUSTER_INDEX_RAD] = GATING.GATE_MEASUREMENTS(TRACK_ESTIMATES_RAD, RADAR_MEAS_CLUSTER, MEAS_MODEL_CV, ...
                                                                               RadarCALLIBRATIONparam, GammaSq, MOTION_MODEL_CV, P_G, ...
                                                                               ASSOCIATION_MAT_RADAR, ASSIGNMENT_MAT_RADAR, GATED_CLUSTER_INDEX_RAD);
    GATED_MEAS_INDEX_RAD = GATING.FIND_GATED_MEASUREMENT_INDEX(GATED_MEAS_INDEX_RAD, RADAR_MEAS_CTS, GATED_CLUSTER_INDEX_RAD, RADAR_MEAS_CLUSTER, RADAR_CLUSTERS);
    % 5) - B) Identify Unassociated RADAR clusters
    [UNASSOCIATED_CLUSTERS_RAD, cntRadClst] = GATING.FIND_UNGATED_CLUSTERS(RADAR_MEAS_CTS.ValidCumulativeMeasCount(end), ...
                                                                           GATED_MEAS_INDEX_RAD, RADAR_CLUSTERS, UNASSOCIATED_CLUSTERS_RAD);
    % ==========================================================================================================================================================                                                               
    % ------------------------------------------------> RADAR MEASUREMENT , TRACKS  & RADAR SENSOR FUSION <-----------------------------------------------------
    % 6 - A) Data Association (Radar Measurements)
    [TRACK_ESTIMATES_RAD, FUSION_INFO_RAD] = FUSION.DATA_ASSOCIATION(TRACK_ESTIMATES_RAD, ASSIGNMENT_MAT_RADAR, RADAR_MEAS_CLUSTER, MEAS_MODEL_CV, FUSION_INFO_RAD);
    % 6 - B) Sensor Fusion    (Radar Tracks)
    [TRACK_ESTIMATES_RAD] = FUSION.HOMOGENEOUS_SENSOR_FUSION_RADARS(TRACK_ESTIMATES_RAD, FUSION_INFO_RAD);  
    % ==========================================================================================================================================================                                                               
    % -----------------------------------------------------------> LOCAL RADAR TRACK MANAGEMENT <---------------------------------------------------------------
    % 7 - A) Track Management (RADAR Sensor)
    [TRACK_ESTIMATES_RAD, ~] = TRACK_MANAGER.INIT_NEW_TRACK(RADAR_CLUSTERS, UNASSOCIATED_CLUSTERS_RAD, cntRadClst, TRACK_ESTIMATES_RAD, dT); 
    TRACK_ESTIMATES_RAD = TRACK_MANAGER.MAINTAIN_EXISTING_TRACK(TRACK_ESTIMATES_RAD, dT);
    [TRACK_ESTIMATES_RAD, ~] = TRACK_MANAGER.DELETE_LOST_TRACK(TRACK_ESTIMATES_RAD, TrackParam); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % ===================================================> TRACK ESTIMATIONS FROM CAMERA MEASUREMENTS <=========================================================
    % The following Steps are for Tracks estimation using CAMERA Sensors Only (CAMERA TRACKS)
    % The steps are as follows :
    % 1. Cluster the Concatenated Camera measurements (All cameras) by NN Clustering Algorithm, 
    %    where each cluster corresponds to a specific object ( Traffic Object like vehicle, pedestrian etc)
    % 2. Perform state prediction of the object tracks from time t-1 to t (tracks are detected by th CAMERA sensors only)
    % 3. Perform Gating of CAMERA merasurements from each CAMERA with the predicted tracks
    % 4. Perform Data association , CAMERA sensor fusion and state estimation 
    % 5. Manage CAMERA Tracks by a Camera Track manager
    % ==========================================================================================================================================================
    % ---------------------------------------------------------->  CAMERA MEASUREMENT CLUSTERING  <-------------------------------------------------------------
    % 3) - B) Cluster Camera measurements
    CAMERA_CLUSTERS = CLUSTERING.CLUSTER_CAMERA_MEASUREMENTS_NN(CAMERA_MEAS_CTS, MANAGE_CLUSTERS.CAM, CAMERA_CLUSTERS, epsPosNN);
    % ==========================================================================================================================================================
    % --------------------------------------------------------> STATE PREDICTION OF CAMERA OBJECTS/TRACKS <-----------------------------------------------------
    % 4) - A) State Prediction of the object Tracks (Step 1: Ego Compensation of the tracks )
    TRACK_ESTIMATES_CAM = STATE_PREDICTOR.EGO_COMPENSATION(TRACK_ESTIMATES_CAM, EGO_CAN_BUS, dT, 'none');
    % 4) - B) State Prediction of the object Tracks (Step 2: Constant Velocity based Prediction (CV : Linear Motion Model) )
    TRACK_ESTIMATES_CAM = STATE_PREDICTOR.LINEAR_PROCESS_MODEL(TRACK_ESTIMATES_CAM,  MOTION_MODEL_CV);
    % ==========================================================================================================================================================
    % -----------------------------------------------> CAMERA MEASUREMENT CLUSTER GATING & IDENTIFY UNGATED CLUSTERS <------------------------------------------                                                          
    % 5) - C) Gating of CAMERA MEASUREMENTS
    [ASSIGNMENT_MAT_CAMERA, GATED_MEAS_INDEX_CAM] = GATING.GATE_MEASUREMENTS(TRACK_ESTIMATES_CAM, CAMERA_MEAS_CTS, MEAS_MODEL_CV, ...
                                                                             CameraCALLIBRATIONparam, GammaSq, MOTION_MODEL_CV, P_G, ...
                                                                             ASSOCIATION_MAT_CAMERA, ASSIGNMENT_MAT_CAMERA, GATED_MEAS_INDEX_CAM);                                                          
    % 5) - D) Identify Unassociated CAMERA clusters
    [UNASSOCIATED_CLUSTERS_CAM, cntCamClst] = GATING.FIND_UNGATED_CLUSTERS(CAMERA_MEAS_CTS.ValidCumulativeMeasCount(end), GATED_MEAS_INDEX_CAM, ...
                                                                           CAMERA_CLUSTERS, UNASSOCIATED_CLUSTERS_CAM); 
    % ==========================================================================================================================================================                                                               
    % ----------------------------------------------------> CAMERA MEASUREMENT , TRACKS  & CAMERA SENSOR FUSION <-----------------------------------------------
    % 6 - C) Data Association (Camera Measurements)
    [TRACK_ESTIMATES_CAM, FUSION_INFO_CAM] = FUSION.DATA_ASSOCIATION(TRACK_ESTIMATES_CAM, ASSIGNMENT_MAT_CAMERA, CAMERA_MEAS_CTS, MEAS_MODEL_CV, FUSION_INFO_CAM);
    % 6 - D) Sensor Fusion       (Camera Tracks)
    [TRACK_ESTIMATES_CAM] = FUSION.HOMOGENEOUS_SENSOR_FUSION_CAMERAS(TRACK_ESTIMATES_CAM, FUSION_INFO_CAM);
    % ==========================================================================================================================================================                                                               
    % -----------------------------------------------------------> LOCAL CAMERA TRACK MANAGEMENT <-------------------------------------------------------------- 
    % 7 - B) Track Management (CAMERA), 
    [TRACK_ESTIMATES_CAM, ~] = TRACK_MANAGER.INIT_NEW_TRACK(CAMERA_CLUSTERS, UNASSOCIATED_CLUSTERS_CAM, cntCamClst, TRACK_ESTIMATES_CAM, dT); 
    TRACK_ESTIMATES_CAM = TRACK_MANAGER.MAINTAIN_EXISTING_TRACK(TRACK_ESTIMATES_CAM, dT);
    [TRACK_ESTIMATES_CAM, ~] = TRACK_MANAGER.DELETE_LOST_TRACK(TRACK_ESTIMATES_CAM, TrackParam);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    
    
    % ===================================================> TRACK To TRACK FUSION  FROM RADAR & CAMERA TRACKS <==================================================
    % The following Steps are for Tracks fusin using RADAR & CAMERA estimated Tracks
    % The steps are as follows :
    % 1. Init Fused Track
    % 2. Perform state prediction of the FUSED object tracks from time t-1 to t (tracks are detected by the eith CAMERA or RADAR or BOTH sensors)
    % 3. Perform Grouping of RADAR and CAMERA tracks with the predicted FUSED tracks
    % 4. Perform TRACK to TRACK FUSION  from RADAR and CAMERA estimated Tracks
    % ==========================================================================================================================================================
    % ------------------------------------------------------ > State Prediction of the object Tracks <----------------------------------------------------------
    TRACK_ESTIMATES_FUS = STATE_PREDICTOR.EGO_COMPENSATION(TRACK_ESTIMATES_FUS, EGO_CAN_BUS, dT, 'none');
    TRACK_ESTIMATES_FUS = STATE_PREDICTOR.LINEAR_PROCESS_MODEL(TRACK_ESTIMATES_FUS, MOTION_MODEL_CV);
    % ==========================================================================================================================================================
    % ----------------------------------------------> Gating local radar and camera tracks with the fused track <-----------------------------------------------
    [GATED_TRACK_INFO, UNGATED_TRACK_INFO] = GATING.GATE_FUSED_TRACK_WITH_LOCAL_TRACKS(TRACK_ESTIMATES_FUS, TRACK_ESTIMATES_RAD, TRACK_ESTIMATES_CAM);
    % ==========================================================================================================================================================
    % ----------------------------------------------------> Sensor Fusion (Heterogeneous,  RADAR + CAMERA) <----------------------------------------------------
    TRACK_ESTIMATES_FUS = FUSION.TRACK_FUSION_HETEROGENEOUS_SENSORS(TRACK_ESTIMATES_FUS, TRACK_ESTIMATES_RAD, TRACK_ESTIMATES_CAM, GATED_TRACK_INFO);
    % ==========================================================================================================================================================
    % ------------------------------------------------> Set New (FUSED) Tracks from the Clustered Local Tracks <------------------------------------------------
    [FUSED_TRACKS, nNewTracks] = TRACK_MANAGER.FORM_NEW_TRACKS_FROM_LOCAL_TRACKS(TRACK_ESTIMATES_RAD, TRACK_ESTIMATES_CAM, UNGATED_TRACK_INFO);
    % ==========================================================================================================================================================                                                               
    % ----------------------------------------------------------------> FUSED TRACK MANAGEMENT <---------------------------------------------------------------- 
    TRACK_ESTIMATES_FUS = TRACK_MANAGER.SET_NEW_TRACK_INFO(TRACK_ESTIMATES_FUS, FUSED_TRACKS, nNewTracks, dT);
    TRACK_ESTIMATES_FUS = TRACK_MANAGER.MAINTAIN_EXISTING_TRACK(TRACK_ESTIMATES_FUS, dT);
    [TRACK_ESTIMATES_FUS, ~] = TRACK_MANAGER.DELETE_LOST_TRACK(TRACK_ESTIMATES_FUS, TrackParam);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % =======================================================> TRAJECTORY HISTORY COMPUTATION <=================================================================
    TRAJECTORY_HISTORY = TRAJECTORY.INIT_TRAJECTORY_HISTORY(TRAJECTORY_HISTORY, TRACK_ESTIMATES_FUS);
    TRAJECTORY_HISTORY = TRAJECTORY.TRAJECTORY_HISTORY_COMPUTATION(TRAJECTORY_HISTORY, TRACK_ESTIMATES_FUS, EGO_CAN_BUS, dT);
    
    
    ExecutionCycleTime_FUS(t) = toc;
    VIZ.VisualizeTrackData(FUSED_TRACKS, TRACK_ESTIMATES_FUS, TRACK_ESTIMATES_RAD, TRACK_ESTIMATES_CAM, RADAR_MEAS_CLUSTER, RADAR_MEAS_CTS, CAMERA_MEAS_CTS, ...
                           nNewTracks, nRadars, nCameras, GammaSq, TRAJECTORY_HISTORY);
    disp(strcat('Time Elapsed : ', num2str(0.05*t)));
end