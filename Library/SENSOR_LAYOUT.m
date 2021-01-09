% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author      : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Functions for specifying the layout of sensors around the ego vehicle, Contains the following function 
%             : setSensorExtrinsicParam() : set sensor extrinsics (radar and camera)
%             : setRadarIntrinsicParam()  : set camera intrinsics 
%             : setCameraIntrinsicParam() : set radar intrinsics
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef SENSOR_LAYOUT
    methods(Static)
        % ======================================================================================================================================================
        % Sensor extrinsic parameters (common to both radar and camera)
        function [SensorEXTRINSICparam] = setSensorExtrinsicParam(nSensors, ...
                                                                  SENSOR_TYPE, ...
                                                                  IS_SENSOR_ACTIVE, ...
                                                                  SENSOR_MOUNT_X, ...
                                                                  SENSOR_MOUNT_Y, ...
                                                                  SENSOR_MOUNT_Z, ...
                                                                  SENSOR_MOUNT_ROLL, ...
                                                                  SENSOR_MOUNT_PITCH, ...
                                                                  SENSOR_MOUNT_YAW, ...
                                                                  SENSOR_nMEAS, ...
                                                                  SensorEXTRINSICparam, ROTATION_CONV)
                 for snsrIdx = 1:nSensors
                     DEG2RAD = single(pi/180);
                     SensorEXTRINSICparam(1,snsrIdx).SensorID = uint8(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).SensorType = SENSOR_TYPE(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).isActive = IS_SENSOR_ACTIVE(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).MountX = SENSOR_MOUNT_X(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).MountY = SENSOR_MOUNT_Y(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).MountZ = SENSOR_MOUNT_Z(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).MountYaw = SENSOR_MOUNT_YAW(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).MountPitch = SENSOR_MOUNT_PITCH(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).MountRoll = SENSOR_MOUNT_ROLL(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).nMeas = SENSOR_nMEAS(snsrIdx);
                     SensorEXTRINSICparam(1,snsrIdx).RotMat2D = [ cos(SENSOR_MOUNT_YAW(snsrIdx)*DEG2RAD), ...
                                                                 -sin(ROTATION_CONV*SENSOR_MOUNT_YAW(snsrIdx)*DEG2RAD); ...
                                                                  sin(ROTATION_CONV*SENSOR_MOUNT_YAW(snsrIdx)*DEG2RAD), ...
                                                                  cos(SENSOR_MOUNT_YAW(snsrIdx)*DEG2RAD)];
                     SensorEXTRINSICparam(1,snsrIdx).TranslationVec = [SENSOR_MOUNT_X(snsrIdx); SENSOR_MOUNT_Y(snsrIdx)];
                 end
        end
        % ======================================================================================================================================================
        % RADAR Internal Parameters
        function [RadarINTRINSICparam] = setRadarIntrinsicParam(nRadarTypes, ...
                                                                RADAR_MAX_RANGE, ...
                                                                RADAR_MAX_AZIMUTH, ...
                                                                RADAR_MAX_ELEVATION, ...
                                                                RADAR_MAX_RANGE_RATE, ...
                                                                RADAR_RANGE_RES, ...
                                                                RADAR_AZIMUTH_RES, ...
                                                                RADAR_ELEVATION_RES, ...
                                                                RADAR_RANGE_RATE_RES, ...
                                                                RADAR_RANGE_ERR_VAR, ...
                                                                RADAR_AZIMUTH_ERR_VAR, ...
                                                                RADAR_ELEVATION_ERR_VAR, ...
                                                                RADAR_RANGE_RATE_ERR_VAR, ...
                                                                RADAR_PD, ...
                                                                RADAR_FA, ...
                                                                RADAR_FOV_BOUNDARY_PTS_RANGE, ...
                                                                RADAR_FOV_BOUNDARY_PTS_AZIMUTH, ...
                                                                RadarINTRINSICparam)
                 for radIdx = 1:nRadarTypes
                     RadarINTRINSICparam(1,radIdx).RadarType = uint8(radIdx);
                     RadarINTRINSICparam(1,radIdx).MaxRange = RADAR_MAX_RANGE(radIdx);
                     RadarINTRINSICparam(1,radIdx).MaxAzimuth = RADAR_MAX_AZIMUTH(radIdx);
                     RadarINTRINSICparam(1,radIdx).MaxElevation = RADAR_MAX_ELEVATION(radIdx);
                     RadarINTRINSICparam(1,radIdx).MaxRangeRate = RADAR_MAX_RANGE_RATE(radIdx);
                     RadarINTRINSICparam(1,radIdx).RangeResolution = RADAR_RANGE_RES(radIdx);
                     RadarINTRINSICparam(1,radIdx).AzimuthResolution = RADAR_AZIMUTH_RES(radIdx);
                     RadarINTRINSICparam(1,radIdx).ElevationResolution = RADAR_ELEVATION_RES(radIdx);
                     RadarINTRINSICparam(1,radIdx).RangeRateResolution = RADAR_RANGE_RATE_RES(radIdx);
                     RadarINTRINSICparam(1,radIdx).RangeErrVariance = RADAR_RANGE_ERR_VAR(radIdx);
                     RadarINTRINSICparam(1,radIdx).AzimuthErrVariance = RADAR_AZIMUTH_ERR_VAR(radIdx);
                     RadarINTRINSICparam(1,radIdx).ElevationErrVariance = RADAR_ELEVATION_ERR_VAR(radIdx);
                     RadarINTRINSICparam(1,radIdx).RangeRateErrVariance = RADAR_RANGE_RATE_ERR_VAR(radIdx);
                     RadarINTRINSICparam(1,radIdx).ProbOfDetection = RADAR_PD(radIdx);
                     RadarINTRINSICparam(1,radIdx).FalseAlarmRate = RADAR_FA(radIdx);
                     RadarINTRINSICparam(1,radIdx).FOVRangePoints = RADAR_FOV_BOUNDARY_PTS_RANGE(radIdx,:);
                     RadarINTRINSICparam(1,radIdx).FOVAzimuthPts = RADAR_FOV_BOUNDARY_PTS_AZIMUTH(radIdx,:);
                 end
        end
        % ======================================================================================================================================================
        % CAMERA Internal Parameters
        function [CameraINTRINSICparam] = setCameraIntrinsicParam(nCameraTypes, ...
                                                                  CAMERA_MAX_RANGE, ...
                                                                  CAMERA_MAX_AZIMUTH, ...
                                                                  CAMERA_MAX_ELEVATION, ...
                                                                  CAMERA_LONG_ERR_VAR, ...
                                                                  CAMERA_LAT_ERR_VAR, ...
                                                                  CAMERA_PD, ...
                                                                  CAMERA_FA, ...
                                                                  CAMERA_FOV_BOUNDARY_PTS_RANGE, ...
                                                                  CAMERA_FOV_BOUNDARY_PTS_AZIMUTH, ...
                                                                  CameraINTRINSICparam)
                 for camIdx = 1:nCameraTypes
                     CameraINTRINSICparam(camIdx).CameraType = uint8(camIdx);
                     CameraINTRINSICparam(camIdx).RectificationMatrix = single(zeros(3,3));
                     CameraINTRINSICparam(camIdx).ProjectionMatrix = single(zeros(3,3));
                     CameraINTRINSICparam(camIdx).MaxRange = CAMERA_MAX_RANGE(camIdx);
                     CameraINTRINSICparam(camIdx).MaxAzimuth = CAMERA_MAX_AZIMUTH(camIdx);
                     CameraINTRINSICparam(camIdx).MaxElevation = CAMERA_MAX_ELEVATION(camIdx);
                     CameraINTRINSICparam(camIdx).LongitudinalErrVariance = CAMERA_LONG_ERR_VAR(camIdx);
                     CameraINTRINSICparam(camIdx).LateralErrVariance = CAMERA_LAT_ERR_VAR(camIdx);
                     CameraINTRINSICparam(camIdx).ProbOfDetection = CAMERA_PD(camIdx);
                     CameraINTRINSICparam(camIdx).FalseAlarmRate = CAMERA_FA(camIdx);
                     CameraINTRINSICparam(camIdx).FOVRangePoints = CAMERA_FOV_BOUNDARY_PTS_RANGE(camIdx, :);
                     CameraINTRINSICparam(camIdx).FOVAzimuthPts = CAMERA_FOV_BOUNDARY_PTS_AZIMUTH(camIdx, :);
                 end
        end
        % ======================================================================================================================================================
        function [FOV_boundary] = GenerateFOVBoundaryPointsCartesian(FOVMaxRange, FOVMaxAzimuth, nPts)
                  DEG2RAD = pi/180;
                  FOV_boundary = single(zeros(2,nPts + nPts -1 + nPts - 2));
                  dTheta = 2*FOVMaxAzimuth/(nPts-1);
                  thetaInit = -FOVMaxAzimuth;
                  FOV_boundary(1,1) = FOVMaxRange*cos(thetaInit*DEG2RAD);
                  FOV_boundary(2,1) = FOVMaxRange*sin(thetaInit*DEG2RAD);
                  for idx = 2:nPts
                      theta = thetaInit + (idx -1)*dTheta;
                      FOV_boundary(1,idx) = FOVMaxRange*cos(theta*DEG2RAD);
                      FOV_boundary(2,idx) = FOVMaxRange*sin(theta*DEG2RAD);
                  end
                  FOV_boundary = [FOV_boundary, FOV_boundary(:,1)];
        end
        % ======================================================================================================================================================
        function [SensorFOVPtsCartesian] = GenerateFOVBoundaryPoints(SensorINTRINSICparam, nSensorTypes, nPts)
                 SensorFOVPtsCartesian = struct;
                 SensorFOVPtsCartesian.SensorID = uint16(0);
                 SensorFOVPtsCartesian.XCoord = single(0);
                 SensorFOVPtsCartesian.YCoord = single(0);
                 SensorFOVPtsCartesian = SensorFOVPtsCartesian(ones(1, nSensorTypes));
                 for idx = 1:nSensorTypes
                     FOVPts = SENSOR_LAYOUT.GenerateFOVBoundaryPointsCartesian(SensorINTRINSICparam(idx).MaxRange, SensorINTRINSICparam(idx).MaxAzimuth, nPts);
                     SensorFOVPtsCartesian(idx).XCoord = FOVPts(1,:);
                     SensorFOVPtsCartesian(idx).YCoord = FOVPts(2,:);
                     SensorFOVPtsCartesian(idx).SensorID = uint16(idx);
                 end
        end
        % ======================================================================================================================================================
        function [FOVPtsEGOframe] = TransformFOVtoEGOframe(SensorParam, nSensors, nSensorTypes, nPts)
            FOVPtsEGOframe = struct;
            FOVPtsEGOframe.Xcoord = single(0);
            FOVPtsEGOframe.Ycoord = single(0);
            FOVPtsEGOframe = FOVPtsEGOframe(ones(1, nSensors));
            SensorFOVPtsCartesian = SENSOR_LAYOUT.GenerateFOVBoundaryPoints(SensorParam.Intrinsic, nSensorTypes, nPts);
            for idx = 1:nSensors
                snsrType = SensorParam.Extrinsic(idx).SensorType;
                PtsLocalFrame  = [SensorFOVPtsCartesian(snsrType).XCoord(1,:); SensorFOVPtsCartesian(snsrType).YCoord(1,:)];
                RotMatrix = SensorParam.Extrinsic(idx).RotMat2D; 
                TranslationVector = SensorParam.Extrinsic(idx).TranslationVec;
                PtsEGOframe = RotMatrix * PtsLocalFrame + TranslationVector;
                FOVPtsEGOframe(idx).Xcoord = PtsEGOframe(1,:);
                FOVPtsEGOframe(idx).Ycoord = PtsEGOframe(2,:);
            end
        end
        % ======================================================================================================================================================
    end
end