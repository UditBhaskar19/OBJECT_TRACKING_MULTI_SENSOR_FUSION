% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : Functions to arrange sensor data
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef SENSOR_INTERFACE_LANE
    methods(Static)
        % ======================================================================================================================================================
        function [isValid] = isMeasurementValid(CurveLength)
            % Check of the measurements are valid
            % INPUT  : px, py : measurement from a sensor
            % OUTPUT : valid measurement flag
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            if(isnan(CurveLength))
                 isValid = false;
            else
                 isValid = true;
            end
         end
         % =====================================================================================================================================================
         function [CAMERA_CAN_BUS] = set_CAMERA_SENSOR_MEAS_DATA(CAM_Sensor_Simulated_Data, timeIdx, snsrIdx, nMeas, CAMERA_CAN_BUS)
             % Set CAMERA Sensor Data with the Array of Structure
             % INPUT : CAM_Sensor_Simulated_Data : a structure of 2D arrays of simulated sensor data
             %         timeIdx : time index
             %         snsrIdx : camera sensor index
             %         nMeas :  maximum number of measurements possible
             %         CAMERA_CAN_BUS : initialized array of structure of camera sensor measurements
             % OUTPUT : CAMERA_CAN_BUS : array of structure of camera sensor measurements with the following fields
             %        : LateralOffset : 
             %        : HeadingAngle : 
             %        : Curvature : 
             %        : CurvatureDerivative : 
             %        : CurveLength : 
             %        : LineWidth : 
             %        : MaximumValidX :
             %        : MinimumValidX :
             %        : detTimeStamp  :
             % --------------------------------------------------------------------------------------------------------------------------------------------------
             nValidMeas = int16(0);
             for objIdx = 1:nMeas
                 if( SENSOR_INTERFACE_LANE.isMeasurementValid(CAM_Sensor_Simulated_Data.CurveLength(timeIdx,objIdx)))
                     nValidMeas = nValidMeas + int16(1);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).LineID = uint16(nValidMeas);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).sensorID = uint8(snsrIdx);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).LateralOffset = single(CAM_Sensor_Simulated_Data.LateralOffset(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).HeadingAngle = single(CAM_Sensor_Simulated_Data.HeadingAngle(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).Curvature = single(CAM_Sensor_Simulated_Data.Curvature(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).CurvatureDerivative = single(CAM_Sensor_Simulated_Data.CurvatureDerivative(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).CurveLength = single(CAM_Sensor_Simulated_Data.CurveLength(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).LineWidth = single(CAM_Sensor_Simulated_Data.LineWidth(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).MaximumValidX = single(CAM_Sensor_Simulated_Data.MaximumValidX(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).MinimumValidX = single(CAM_Sensor_Simulated_Data.MinimumValidX(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).detTimeStamp = single(CAM_Sensor_Simulated_Data.detTimeStamp(timeIdx));
                 else
                     CAMERA_CAN_BUS(snsrIdx, objIdx).LineID = uint16(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).sensorID = uint8(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).LateralOffset = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).HeadingAngle = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).Curvature = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).CurvatureDerivative = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).CurveLength = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).LineWidth = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).MaximumValidX = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).MinimumValidX = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).detTimeStamp = single(0);
                 end  
             end                                                  
         end
         % =====================================================================================================================================================
         function [CAMERA_CAN_BUS] = CAM_SENSOR_INTERFACE(CAM1_Sensor_Simulated_Data, ...
                                                          CAM2_Sensor_Simulated_Data, ...
                                                          CAMERA_CAN_BUS, t, nCameras, nMeas)
             % Interface Camera Sensor Data with the Array of Structure
             % INPUT  : CAM1_Sensor_Simulated_Data : Camera 1 simulated measurements
             %          CAM2_Sensor_Simulated_Data : Camera 2 simulated measurements
             %          CAMERA_CAN_BUS : init array of structure of camera measurements
             %          t : time index
             %          nCameras : number of cameras
             %          nMeas : number of measurements
             % OUTPUT : CAMERA_CAN_BUS : array of structure of camera measurements
             % -------------------------------------------------------------------------------------------------------------------------------------------------
             for snsrIdx = 1:nCameras
                 switch(snsrIdx)
                     case 1
                         CAMERA_CAN_BUS = SENSOR_INTERFACE_LANE.set_CAMERA_SENSOR_MEAS_DATA(CAM1_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 2
                         CAMERA_CAN_BUS = SENSOR_INTERFACE_LANE.set_CAMERA_SENSOR_MEAS_DATA(CAM2_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     otherwise
                         error('number of CAMERAs exceeds the upper limit');
                  end            
             end
         end
         % =====================================================================================================================================================
         function [EGO_CAN_BUS] = EGO_SENSOR_INTERFACE(EGO_Sensor_Simulated_Data, EGO_CAN_BUS, t)
             % Interface EGO Sensor Data with structure
             % INPUT  : EGO_Sensor_Simulated_Data : Simulated measurements of the ego sensors
             %        : EGO_CAN_BUS : init structure of ego sensor measurements having the following parameters
             %        : t : time index
             % OUTPUT : EGO_CAN_BUS : structure of ego sensor measurements having the following parameters :
             %        : px, py, vx, vy, yaw, yawRate
             % -------------------------------------------------------------------------------------------------------------------------------------------------
             EGO_CAN_BUS.px = single(EGO_Sensor_Simulated_Data.px(t,1));
             EGO_CAN_BUS.py = single(EGO_Sensor_Simulated_Data.py(t,1));
             EGO_CAN_BUS.vx = single(EGO_Sensor_Simulated_Data.vx(t,1));
             EGO_CAN_BUS.vy = single(EGO_Sensor_Simulated_Data.vy(t,1));
             EGO_CAN_BUS.yaw = single(EGO_Sensor_Simulated_Data.yaw(t,1));
             EGO_CAN_BUS.yawRate = single(EGO_Sensor_Simulated_Data.yawRate(t,1));
             EGO_CAN_BUS.detTimeStamp = single(EGO_Sensor_Simulated_Data.detTimeStamp(t,1)); 
         end
         % =====================================================================================================================================================
         function LANE_LINE_MEAS_MAT = CREATE_LANE_LINE_MEAS_MAT(nCameras, nLineMeas, CAMERA_CAN_LANE_LINE_BUS, LANE_LINE_MEAS_MAT)  
           % Creates a matrix of valid line measurements also convert degree to radians
           % INPUTS : nCameras : number of cameras
           %          nLineMeas : maximum number of line measurements
           %          CAMERA_CAN_LANE_LINE_BUS : array of structure of Line measurement 
           %          LANE_LINE_MEAS_MAT : initialized data structure for valid line measurements  
           % OUTPUT : LANE_LINE_MEAS_MAT : data structure for valid line measurements  
           %          MeasArray      : coordinate transformed measurement array  (2D array)
           %          MeasRef        : Measurement reference array (meas ID from array of struct and sensor ID)
           %          ValidMeasCount : array of number of valid  measurements from each sensor
           %          ValidCumulativeMeasCount : cumulative valid measurement number count 
           % ---------------------------------------------------------------------------------------------------------------------------------------------------
           LANE_LINE_MEAS_MAT.ValidMeasCount(:) = int16(0);
           TotalCount = int16(0);
           Z = single(zeros(size(LANE_LINE_MEAS_MAT.MeasArray,1),1));
           DEG2RAD = single(pi/180);
           for snsrIdx = 1:nCameras
               for objIdx = 1:nLineMeas
                   if(CAMERA_CAN_LANE_LINE_BUS(snsrIdx, objIdx).LineID ~= 0)
                      % extract the measurement parameters
                      Z(1,1) = CAMERA_CAN_LANE_LINE_BUS(snsrIdx, objIdx).LateralOffset;
                      Z(2,1) = DEG2RAD*CAMERA_CAN_LANE_LINE_BUS(snsrIdx, objIdx).HeadingAngle;
                      Z(3,1) = DEG2RAD*CAMERA_CAN_LANE_LINE_BUS(snsrIdx, objIdx).Curvature;
                      Z(4,1) = DEG2RAD*CAMERA_CAN_LANE_LINE_BUS(snsrIdx, objIdx).CurvatureDerivative;
                      Z(5,1) = CAMERA_CAN_LANE_LINE_BUS(snsrIdx, objIdx).CurveLength;
                      TotalCount = TotalCount + 1;
                      LANE_LINE_MEAS_MAT.ValidMeasCount(1,snsrIdx) = LANE_LINE_MEAS_MAT.ValidMeasCount(1,snsrIdx) + 1;
                      LANE_LINE_MEAS_MAT.MeasRef(1, TotalCount) = single(objIdx);  % save the can array index in the measurement array
                      LANE_LINE_MEAS_MAT.MeasRef(2, TotalCount) = single(CAMERA_CAN_LANE_LINE_BUS(snsrIdx, objIdx).sensorID); % sensor ID
                      LANE_LINE_MEAS_MAT.MeasArray(:, TotalCount) = Z;
                    end
               end
           end
           LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount = cumsum(LANE_LINE_MEAS_MAT.ValidMeasCount);
         end
        % ======================================================================================================================================================
    end
end