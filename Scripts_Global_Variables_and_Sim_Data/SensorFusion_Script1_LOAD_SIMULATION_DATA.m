% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : This script loads and rearranges the simulated sensor data. The simulated data is generated using matlab Automated Driving Toolbox
% ==============================================================================================================================================================
clearvars; clc; close all;
simData = load('..\Scenario_Simulation_Data\SCENARIO_1\SCENE_1_RAW.mat');
simData = simData.SCENARIO_1_RAW;
nTimeSamples = length(simData);
dT = 0.05; % in sec
SimulationTime = simData(end).Time;
% simData is an array of structure of size (1 x nTimeSamples) having the
% below member variables:
% 1. Time
% 2. ActorPoses
% 3. ObjectDetections.
% 4. LaneDetections.
TimeVector = single(zeros(nTimeSamples,1));
for idx = 1:nTimeSamples
    TimeVector(idx,1) = simData(idx).Time;
end
% ==============================================================================================================================================================
% Lane Detection Data 
% An Array of idx1->curvature, idx2->curvRate, idx3->curvLen, idx4->alpha, idx5->latOffset, idx6->xMin, idx7->xMax, idx8->width
nLaneLines = 2; nLaneDimension = 8; nLaneSensors = 8;
Lane_Line_Measurements.Time = single(zeros(nTimeSamples, 1));
Lane_Line_Measurements.SensorID = int16(zeros(nTimeSamples, 1));
Lane_Line_Measurements.Curvature = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.CurvRate = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.CurveLength = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.HeadingAngle = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.LateralOffset = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.XMin = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.XMax = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.Width = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements = Lane_Line_Measurements(ones(nLaneSensors, 1));
for timeIdx = 1:nTimeSamples
    for snsrIdx = 1:nLaneSensors
        Lane_Line_Measurements(snsrIdx).Time(timeIdx,1) = simData(timeIdx).LaneDetections(snsrIdx).Time;
        Lane_Line_Measurements(snsrIdx).SensorID(timeIdx, 1) = simData(timeIdx).LaneDetections(snsrIdx).SensorIndex;
        for lineIdx = 1: nLaneLines
            Lane_Line_Measurements(snsrIdx).Curvature(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).Curvature;
            Lane_Line_Measurements(snsrIdx).CurvRate(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).CurvatureDerivative;
            Lane_Line_Measurements(snsrIdx).CurveLength(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).CurveLength;
            Lane_Line_Measurements(snsrIdx).HeadingAngle(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).HeadingAngle;
            Lane_Line_Measurements(snsrIdx).LateralOffset(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).LateralOffset;
            Lane_Line_Measurements(snsrIdx).XMin(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).XExtent(1,1);
            Lane_Line_Measurements(snsrIdx).XMax(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).XExtent(1,2);
            Lane_Line_Measurements(snsrIdx).Width(timeIdx, lineIdx) = simData(timeIdx).LaneDetections(snsrIdx).LaneBoundaries(lineIdx).Width;
        end
    end
end
LaneSimulation.Data = Lane_Line_Measurements;
LaneSimulation.Time = TimeVector;
% ==============================================================================================================================================================
% Ground Truth and Ego Sensor Data
nActors = length(simData(1).ActorPoses);
nDim = 13; % no. of attributes in the groundtruth;
% id, px, py, pz, vx, vy, vz, roll, pitch, yaw, angular_vel_x, angular_vel_y, angular_vel_z
GroundTruthArray = single(zeros(nTimeSamples, nDim));
GroundTruth = struct;
GriundTruth.ActorID = int16(0);
GroundTruth.Data = GroundTruthArray;
GroundTruth = GroundTruth(ones(nActors, 1));
for idx = 1:nTimeSamples
    poses = simData(idx).ActorPoses;
    for idxActor = 1:nActors
        actorData = simData(idx).ActorPoses(idxActor);
        GroundTruth(idxActor).ActorID = actorData.ActorID;
        GroundTruth(idxActor).Data(idx,1) = simData(idx).ActorPoses(idxActor).Position(1,1);
        GroundTruth(idxActor).Data(idx,2) = simData(idx).ActorPoses(idxActor).Position(1,2);
        GroundTruth(idxActor).Data(idx,3) = simData(idx).ActorPoses(idxActor).Position(1,3);
        GroundTruth(idxActor).Data(idx,4) = simData(idx).ActorPoses(idxActor).Velocity(1,1);
        GroundTruth(idxActor).Data(idx,5) = simData(idx).ActorPoses(idxActor).Velocity(1,2);
        GroundTruth(idxActor).Data(idx,6) = simData(idx).ActorPoses(idxActor).Velocity(1,3);
        GroundTruth(idxActor).Data(idx,7) = simData(idx).ActorPoses(idxActor).Roll;
        GroundTruth(idxActor).Data(idx,8) = simData(idx).ActorPoses(idxActor).Pitch;
        GroundTruth(idxActor).Data(idx,9) = simData(idx).ActorPoses(idxActor).Yaw;
        GroundTruth(idxActor).Data(idx,10) = simData(idx).ActorPoses(idxActor).AngularVelocity(1,1);
        GroundTruth(idxActor).Data(idx,11) = simData(idx).ActorPoses(idxActor).AngularVelocity(1,2);
        GroundTruth(idxActor).Data(idx,12) = simData(idx).ActorPoses(idxActor).AngularVelocity(1,3);
    end
end
% perform sanity check by plotting the Ground Truth
figure(1);
startX = GroundTruth(1).Data(1, 1); startY = GroundTruth(1).Data(1, 2);
endX = GroundTruth(1).Data(nTimeSamples, 1); endY = GroundTruth(1).Data(nTimeSamples, 2);
plot(startX ,startY,'-s','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);hold on;
plot(endX ,endY,'-*','MarkerSize',10,'MarkerEdgeColor','blue','MarkerFaceColor',[1 .6 .6]);hold on;
X0 = GroundTruth(1).Data(:, 1); Y0 = GroundTruth(1).Data(:, 2);
X1 = GroundTruth(2).Data(:, 1); Y1 = GroundTruth(2).Data(:, 2);
X2 = GroundTruth(3).Data(:, 1); Y2 = GroundTruth(3).Data(:, 2);
plot(X0,Y0,'b');hold on; plot(X1,Y1,'r'); hold on; plot(X2,Y2,'g');hold off;axis equal
legend('Initial Position of the EV at the start of the simulation',...
       'Final Position of the EV at the end of the simulation',...
       'EV Ground Truth Trajectory',...
       'TV1 Ground Truth Trajectory',...
       'TV2 Ground Truth Trajectory','Location','NorthWest');
disp('Press any Key to Continue'); pause();
% ==============================================================================================================================================================  
% Object Measurement
% Each of the Object measurements ( simData(timeIdx).ObjectDetections{objIdx} ) has the following attributes:
% 1. Time in sec
% 2. Measurement (6 x 1) , px, py, pz, vx, vy, vz
% 3. Measurement noise covarience (6, 6)
% 4. SensorIdx (1-8:Cam, 9-14:Radar)
% 5. Object Classification (only valid for camera)
% 6. Measurement Parameters (a structure with the following fields - not required as of now)
%    1. OriginPosition: [3x1 double]
%    2. Orientation:    [3x3 double]
% 7. ObjectAttributes (a structure which is relevant only for radar)
%    1. SNR
nRadars = 6; nCameras = 8;
nSensors = nRadars + nCameras;
nObjects = int16(200);
sensorIDs = [1,2,3,4,5,6,7,8,9,10,11,12,13,14];
radarIDs = [9,10,11,12,13,14];
cameraIDs = [1,2,3,4,5,6,7,8];
nMeas = int16(zeros(nTimeSamples, 1));
for idx = 1:nTimeSamples
    nMeas = length(simData(idx).ObjectDetections);
end
Measurements = struct;
Measurements.sensorID = int16(0);
Measurements.time = single(zeros(nTimeSamples, 1));
Measurements.px = single(zeros(nTimeSamples, nObjects));
Measurements.py = single(zeros(nTimeSamples, nObjects));
Measurements.vx = single(zeros(nTimeSamples, nObjects));
Measurements.vy = single(zeros(nTimeSamples, nObjects));
Measurements.classID = single(zeros(nTimeSamples, nObjects));
Measurements.SNR = single(zeros(nTimeSamples, nObjects));
Measurements.ErrCov = single(zeros(nTimeSamples, 4, 4, nObjects));
Measurements = Measurements(ones(nSensors, 1));
for snsrIdx = 1:nSensors
    Measurements(snsrIdx).sensorID = snsrIdx;
    for timeIdx = 1:nTimeSamples
        nMeas = length(simData(timeIdx).ObjectDetections);
        for measIdx = 1:nMeas
            if(simData(timeIdx).ObjectDetections{measIdx}.SensorIndex == snsrIdx)
                Measurements(snsrIdx).time(timeIdx) = simData(timeIdx).ObjectDetections{measIdx}.Time;
                Measurements(snsrIdx).px(timeIdx,measIdx) = simData(timeIdx).ObjectDetections{measIdx}.Measurement(1,1);
                Measurements(snsrIdx).py(timeIdx,measIdx) = simData(timeIdx).ObjectDetections{measIdx}.Measurement(2,1);
                Measurements(snsrIdx).vx(timeIdx,measIdx) = simData(timeIdx).ObjectDetections{measIdx}.Measurement(4,1);
                Measurements(snsrIdx).vy(timeIdx,measIdx) = simData(timeIdx).ObjectDetections{measIdx}.Measurement(5,1);
                Measurements(snsrIdx).classID(timeIdx,measIdx) = simData(timeIdx).ObjectDetections{measIdx}.ObjectClassID;
                Measurements(snsrIdx).ErrCov(timeIdx,:,:,measIdx) = simData(timeIdx).ObjectDetections{measIdx}.MeasurementNoise([1,2,4,5],[1,2,4,5]);
                if(sum(snsrIdx == radarIDs))
                   Measurements(snsrIdx).SNR(timeIdx,measIdx) = simData(timeIdx).ObjectDetections{measIdx}.ObjectAttributes{1}.SNR;
                else
                   Measurements(snsrIdx).SNR(timeIdx,measIdx) = single(0);
                end
            end
        end
    end
end
MeasurementSimulation.Data = Measurements;
MeasurementSimulation.Time = TimeVector;
% ==============================================================================================================================================================
% Ego Sensor Measurements
EgoSensorMeasurements.px = GroundTruth(1).Data(:, 1);
EgoSensorMeasurements.py = GroundTruth(1).Data(:, 2);
EgoSensorMeasurements.vx = GroundTruth(1).Data(:, 4);
EgoSensorMeasurements.vy = GroundTruth(1).Data(:, 5);
EgoSensorMeasurements.yaw = GroundTruth(1).Data(:, 9);
EgoSensorMeasurements.yawRate = GroundTruth(1).Data(:, 12);
EgoParameters.Data = EgoSensorMeasurements;
EgoParameters.Time = TimeVector;
% ==============================================================================================================================================================
% Save the Data
SimulationScenario.TotalTime = SimulationTime;
SimulationScenario.SampleTime = dT;
SimulationScenario.SensorMeasurements = MeasurementSimulation;
SimulationScenario.EgoSensorMeasurements = EgoParameters;
SimulationScenario.LaneSensorMeasurements = LaneSimulation;
save('..\Scenario_Simulation_Data\SCENE_1.mat', 'SimulationScenario');
clearvars; %clear all;
load('..\Scenario_Simulation_Data\SCENE_1.mat');
% ==============================================================================================================================================================
% Simulation Time
SamplingTime = SimulationScenario.SampleTime;
SimulationTime = SimulationScenario.TotalTime;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 1
LINE_CURVATURE_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).Curvature;
LINE_CURVDERIVATIVE_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).CurvRate;
LINE_CURVELENGTH_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).CurveLength;
LINE_HEADINGANGLE_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).HeadingAngle;
LINE_LATERALOFFSET_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).LateralOffset;
LINE_XMIN_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).XMin;
LINE_XMAX_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).XMax;
LINE_WIDTH_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).Width;
LINE_detTimeStamp_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 2
LINE_CURVATURE_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).Curvature;
LINE_CURVDERIVATIVE_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).CurvRate;
LINE_CURVELENGTH_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).CurveLength;
LINE_HEADINGANGLE_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).HeadingAngle;
LINE_LATERALOFFSET_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).LateralOffset;
LINE_XMIN_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).XMin;
LINE_XMAX_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).XMax;
LINE_WIDTH_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).Width;
LINE_detTimeStamp_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 3
LINE_CURVATURE_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).Curvature;
LINE_CURVDERIVATIVE_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).CurvRate;
LINE_CURVELENGTH_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).CurveLength;
LINE_HEADINGANGLE_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).HeadingAngle;
LINE_LATERALOFFSET_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).LateralOffset;
LINE_XMIN_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).XMin;
LINE_XMAX_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).XMax;
LINE_WIDTH_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).Width;
LINE_detTimeStamp_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 4
LINE_CURVATURE_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).Curvature;
LINE_CURVDERIVATIVE_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).CurvRate;
LINE_CURVELENGTH_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).CurveLength;
LINE_HEADINGANGLE_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).HeadingAngle;
LINE_LATERALOFFSET_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).LateralOffset;
LINE_XMIN_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).XMin;
LINE_XMAX_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).XMax;
LINE_WIDTH_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).Width;
LINE_detTimeStamp_CAM4 = SimulationScenario.LaneSensorMeasurements.Data(4).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 5
LINE_CURVATURE_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).Curvature;
LINE_CURVDERIVATIVE_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).CurvRate;
LINE_CURVELENGTH_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).CurveLength;
LINE_HEADINGANGLE_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).HeadingAngle;
LINE_LATERALOFFSET_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).LateralOffset;
LINE_XMIN_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).XMin;
LINE_XMAX_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).XMax;
LINE_WIDTH_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).Width;
LINE_detTimeStamp_CAM5 = SimulationScenario.LaneSensorMeasurements.Data(5).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 6
LINE_CURVATURE_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).Curvature;
LINE_CURVDERIVATIVE_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).CurvRate;
LINE_CURVELENGTH_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).CurveLength;
LINE_HEADINGANGLE_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).HeadingAngle;
LINE_LATERALOFFSET_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).LateralOffset;
LINE_XMIN_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).XMin;
LINE_XMAX_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).XMax;
LINE_WIDTH_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).Width;
LINE_detTimeStamp_CAM6 = SimulationScenario.LaneSensorMeasurements.Data(6).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 7
LINE_CURVATURE_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).Curvature;
LINE_CURVDERIVATIVE_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).CurvRate;
LINE_CURVELENGTH_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).CurveLength;
LINE_HEADINGANGLE_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).HeadingAngle;
LINE_LATERALOFFSET_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).LateralOffset;
LINE_XMIN_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).XMin;
LINE_XMAX_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).XMax;
LINE_WIDTH_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).Width;
LINE_detTimeStamp_CAM7 = SimulationScenario.LaneSensorMeasurements.Data(7).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 8
LINE_CURVATURE_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).Curvature;
LINE_CURVDERIVATIVE_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).CurvRate;
LINE_CURVELENGTH_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).CurveLength;
LINE_HEADINGANGLE_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).HeadingAngle;
LINE_LATERALOFFSET_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).LateralOffset;
LINE_XMIN_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).XMin;
LINE_XMAX_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).XMax;
LINE_WIDTH_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).Width;
LINE_detTimeStamp_CAM8 = SimulationScenario.LaneSensorMeasurements.Data(8).Time;
% ==============================================================================================================================================================
% EGO VEHICLE Measurements
EGO_PX = SimulationScenario.EgoSensorMeasurements.Data.px;
EGO_PY = SimulationScenario.EgoSensorMeasurements.Data.py;
EGO_VX = SimulationScenario.EgoSensorMeasurements.Data.vx;
EGO_VY = SimulationScenario.EgoSensorMeasurements.Data.vy;
EGO_YAW = SimulationScenario.EgoSensorMeasurements.Data.yaw;
EGO_YAWRATE = SimulationScenario.EgoSensorMeasurements.Data.yawRate;
EGO_TimeStamp = SimulationScenario.EgoSensorMeasurements.Time;
% ==============================================================================================================================================================
% CAMERA 1 Measurements
CAM1_PX = SimulationScenario.SensorMeasurements.Data(1).px;
CAM1_PY = SimulationScenario.SensorMeasurements.Data(1).py;
CAM1_VX = SimulationScenario.SensorMeasurements.Data(1).vx;
CAM1_VY = SimulationScenario.SensorMeasurements.Data(1).vy;
CAM1_objClassID = SimulationScenario.SensorMeasurements.Data(1).classID;
CAM1_MeasNoise = SimulationScenario.SensorMeasurements.Data(1).ErrCov;
CAM1_detTimeStamp = SimulationScenario.SensorMeasurements.Data(1).time;
% ==============================================================================================================================================================
% CAMERA 2 Measurements
CAM2_PX = SimulationScenario.SensorMeasurements.Data(2).px;
CAM2_PY = SimulationScenario.SensorMeasurements.Data(2).py;
CAM2_VX = SimulationScenario.SensorMeasurements.Data(2).vx;
CAM2_VY = SimulationScenario.SensorMeasurements.Data(2).vy;
CAM2_objClassID = SimulationScenario.SensorMeasurements.Data(2).classID;
CAM2_MeasNoise = SimulationScenario.SensorMeasurements.Data(2).ErrCov;
CAM2_detTimeStamp = SimulationScenario.SensorMeasurements.Data(2).time;
% ==============================================================================================================================================================
% CAMERA 3 Measurements
CAM3_PX = SimulationScenario.SensorMeasurements.Data(3).px;
CAM3_PY = SimulationScenario.SensorMeasurements.Data(3).py;
CAM3_VX = SimulationScenario.SensorMeasurements.Data(3).vx;
CAM3_VY = SimulationScenario.SensorMeasurements.Data(3).vy;
CAM3_objClassID = SimulationScenario.SensorMeasurements.Data(3).classID;
CAM3_MeasNoise = SimulationScenario.SensorMeasurements.Data(3).ErrCov;
CAM3_detTimeStamp = SimulationScenario.SensorMeasurements.Data(3).time;
% ==============================================================================================================================================================
% CAMERA 4 Measurements
CAM4_PX = SimulationScenario.SensorMeasurements.Data(4).px;
CAM4_PY = SimulationScenario.SensorMeasurements.Data(4).py;
CAM4_VX = SimulationScenario.SensorMeasurements.Data(4).vx;
CAM4_VY = SimulationScenario.SensorMeasurements.Data(4).vy;
CAM4_objClassID = SimulationScenario.SensorMeasurements.Data(4).classID;
CAM4_MeasNoise = SimulationScenario.SensorMeasurements.Data(4).ErrCov;
CAM4_detTimeStamp = SimulationScenario.SensorMeasurements.Data(4).time;
% ==============================================================================================================================================================
% CAMERA 5 Measurements
CAM5_PX = SimulationScenario.SensorMeasurements.Data(5).px;
CAM5_PY = SimulationScenario.SensorMeasurements.Data(5).py;
CAM5_VX = SimulationScenario.SensorMeasurements.Data(5).vx;
CAM5_VY = SimulationScenario.SensorMeasurements.Data(5).vy;
CAM5_objClassID = SimulationScenario.SensorMeasurements.Data(5).classID;
CAM5_MeasNoise = SimulationScenario.SensorMeasurements.Data(5).ErrCov;
CAM5_detTimeStamp = SimulationScenario.SensorMeasurements.Data(5).time;
% ==============================================================================================================================================================
% CAMERA 6 Measurements
CAM6_PX = SimulationScenario.SensorMeasurements.Data(6).px;
CAM6_PY = SimulationScenario.SensorMeasurements.Data(6).py;
CAM6_VX = SimulationScenario.SensorMeasurements.Data(6).vx;
CAM6_VY = SimulationScenario.SensorMeasurements.Data(6).vy;
CAM6_objClassID = SimulationScenario.SensorMeasurements.Data(6).classID;
CAM6_MeasNoise = SimulationScenario.SensorMeasurements.Data(6).ErrCov;
CAM6_detTimeStamp = SimulationScenario.SensorMeasurements.Data(6).time;
% ==============================================================================================================================================================
% CAMERA 7 Measurements
CAM7_PX = SimulationScenario.SensorMeasurements.Data(7).px;
CAM7_PY = SimulationScenario.SensorMeasurements.Data(7).py;
CAM7_VX = SimulationScenario.SensorMeasurements.Data(7).vx;
CAM7_VY = SimulationScenario.SensorMeasurements.Data(7).vy;
CAM7_objClassID = SimulationScenario.SensorMeasurements.Data(7).classID;
CAM7_MeasNoise = SimulationScenario.SensorMeasurements.Data(7).ErrCov;
CAM7_detTimeStamp = SimulationScenario.SensorMeasurements.Data(7).time;
% ==============================================================================================================================================================
% CAMERA 8 Measurements
CAM8_PX = SimulationScenario.SensorMeasurements.Data(8).px;
CAM8_PY = SimulationScenario.SensorMeasurements.Data(8).py;
CAM8_VX = SimulationScenario.SensorMeasurements.Data(8).vx;
CAM8_VY = SimulationScenario.SensorMeasurements.Data(8).vy;
CAM8_objClassID = SimulationScenario.SensorMeasurements.Data(8).classID;
CAM8_MeasNoise = SimulationScenario.SensorMeasurements.Data(8).ErrCov;
CAM8_detTimeStamp = SimulationScenario.SensorMeasurements.Data(8).time;
% ==============================================================================================================================================================
% RADAR 1 Measurements
RAD1_PX = SimulationScenario.SensorMeasurements.Data(9).px;
RAD1_PY = SimulationScenario.SensorMeasurements.Data(9).py;
RAD1_VX = SimulationScenario.SensorMeasurements.Data(9).vx;
RAD1_VY = SimulationScenario.SensorMeasurements.Data(9).vy;
RAD1_SNR = SimulationScenario.SensorMeasurements.Data(9).SNR;
RAD1_MeasNoise = SimulationScenario.SensorMeasurements.Data(9).ErrCov;
RAD1_detTimeStamp = SimulationScenario.SensorMeasurements.Data(9).time;
% ==============================================================================================================================================================
% RADAR 2 Measurements
RAD2_PX = SimulationScenario.SensorMeasurements.Data(10).px;
RAD2_PY = SimulationScenario.SensorMeasurements.Data(10).py;
RAD2_VX = SimulationScenario.SensorMeasurements.Data(10).vx;
RAD2_VY = SimulationScenario.SensorMeasurements.Data(10).vy;
RAD2_SNR = SimulationScenario.SensorMeasurements.Data(10).SNR;
RAD2_MeasNoise = SimulationScenario.SensorMeasurements.Data(10).ErrCov;
RAD2_detTimeStamp = SimulationScenario.SensorMeasurements.Data(10).time;
% ==============================================================================================================================================================
% RADAR 3 Measurements
RAD3_PX = SimulationScenario.SensorMeasurements.Data(11).px;
RAD3_PY = SimulationScenario.SensorMeasurements.Data(11).py;
RAD3_VX = SimulationScenario.SensorMeasurements.Data(11).vx;
RAD3_VY = SimulationScenario.SensorMeasurements.Data(11).vy;
RAD3_SNR = SimulationScenario.SensorMeasurements.Data(11).SNR;
RAD3_MeasNoise = SimulationScenario.SensorMeasurements.Data(11).ErrCov;
RAD3_detTimeStamp = SimulationScenario.SensorMeasurements.Data(11).time;
% ==============================================================================================================================================================
% RADAR 4 Measurements
RAD4_PX = SimulationScenario.SensorMeasurements.Data(12).px;
RAD4_PY = SimulationScenario.SensorMeasurements.Data(12).py;
RAD4_VX = SimulationScenario.SensorMeasurements.Data(12).vx;
RAD4_VY = SimulationScenario.SensorMeasurements.Data(12).vy;
RAD4_SNR = SimulationScenario.SensorMeasurements.Data(12).SNR;
RAD4_MeasNoise = SimulationScenario.SensorMeasurements.Data(12).ErrCov;
RAD4_detTimeStamp = SimulationScenario.SensorMeasurements.Data(12).time;
% ==============================================================================================================================================================
% RADAR 5 Measurements
RAD5_PX = SimulationScenario.SensorMeasurements.Data(13).px;
RAD5_PY = SimulationScenario.SensorMeasurements.Data(13).py;
RAD5_VX = SimulationScenario.SensorMeasurements.Data(13).vx;
RAD5_VY = SimulationScenario.SensorMeasurements.Data(13).vy;
RAD5_SNR = SimulationScenario.SensorMeasurements.Data(13).SNR;
RAD5_MeasNoise = SimulationScenario.SensorMeasurements.Data(13).ErrCov;
RAD5_detTimeStamp = SimulationScenario.SensorMeasurements.Data(13).time;
% ==============================================================================================================================================================
% RADAR 6 Measurements
RAD6_PX = SimulationScenario.SensorMeasurements.Data(14).px;
RAD6_PY = SimulationScenario.SensorMeasurements.Data(14).py;
RAD6_VX = SimulationScenario.SensorMeasurements.Data(14).vx;
RAD6_VY = SimulationScenario.SensorMeasurements.Data(14).vy;
RAD6_SNR = SimulationScenario.SensorMeasurements.Data(14).SNR;
RAD6_MeasNoise = SimulationScenario.SensorMeasurements.Data(14).ErrCov;
RAD6_detTimeStamp = SimulationScenario.SensorMeasurements.Data(14).time;
% ==========================================================================> END OF SCRIPT <===================================================================
