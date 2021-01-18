% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author      : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Functions for performing coordinate transformation of the measurements (sensor frame to ego vehicle frame)
%             : Contains the following functions,
%             : CTS_SENSOR_FRAME_TO_EGO_FRAME()  : Coordinate Transformation of the sensor measurements from sensor frame to vehicle frame
%             : ctsSnsrToEgoFrame()              : Position, velocity and measurement noise transformation 
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef SPATIAL_ALLIGNMENT
    methods(Static)
        % ======================================================================================================================================================
        function [MEAS_TRANSFORMED_ARRAY] = CTS_SENSOR_FRAME_TO_EGO_FRAME(nSensors, nMeas, SENSOR_INSTALLATION_PARAMETERS, MEAS_CAN_BUS, MEAS_TRANSFORMED_ARRAY)  
           % Perform coordinate transformation of the sensor data from the sensor frame to ego vehicle frame
           % INPUTS : nSensors : number of sensors
           %          nMeas    : maximum number of measurements
           %          SENSOR_INSTALLATION_PARAMETERS : sensor extrinsic parameters
           %          MEAS_CAN_BUS : array of structure of measurement 
           %          MEAS_TRANSFORMED_ARRAY : initialized data structure for coordinate transformed measurements  
           % OUTPUT : MEAS_TRANSFORMED_ARRAY : data structure for coordinate transformed measurements with the following fields
           %          MeasArray      : coordinate transformed measurement array  (2D array)
           %          MeasCovariance : coordinate transformed measurement covariance array (3D)
           %          MeasRef        : Measurement reference array (meas ID from array of struct and sensor ID)
           %          ValidMeasCount : array of number of valid  measurements from each sensor
           %          ValidCumulativeMeasCount : cumulative valid measurement number count 
           % ---------------------------------------------------------------------------------------------------------------------------------------------------
           MEAS_TRANSFORMED_ARRAY.ValidMeasCount(:) = int16(0);
           TotalCount = int16(0);
           for snsrIdx = 1:nSensors
               for objIdx = 1:nMeas
                   if(MEAS_CAN_BUS(snsrIdx, objIdx).measID ~= 0)
                      % extract the measurement parameters
                      measVectorPosition = [MEAS_CAN_BUS(snsrIdx, objIdx).px;  MEAS_CAN_BUS(snsrIdx, objIdx).py];
                      measPosiCovariance = MEAS_CAN_BUS(snsrIdx, objIdx).measNoise;
                      % Apply the transformation on position , velocity and the measurement covariance
                      [Z, R] = SPATIAL_ALLIGNMENT.ctsSnsrToEgoFrame(measVectorPosition, measPosiCovariance, ...
                                                                    SENSOR_INSTALLATION_PARAMETERS(snsrIdx).RotMat2D, ...
                                                                    SENSOR_INSTALLATION_PARAMETERS(snsrIdx).TranslationVec);
                      % Update the transformed data
                      TotalCount = TotalCount + 1;
                      MEAS_TRANSFORMED_ARRAY.ValidMeasCount(1,snsrIdx) = MEAS_TRANSFORMED_ARRAY.ValidMeasCount(1,snsrIdx) + 1;
                      MEAS_TRANSFORMED_ARRAY.MeasRef(1, TotalCount) = single(objIdx);  % save the can array index in the measurement array
                      MEAS_TRANSFORMED_ARRAY.MeasRef(2, TotalCount) = single(MEAS_CAN_BUS(snsrIdx, objIdx).sensorID); % sensor ID
                      MEAS_TRANSFORMED_ARRAY.MeasArray(:, TotalCount) = Z;
                      MEAS_TRANSFORMED_ARRAY.MeasCovariance(:, :, TotalCount) = R;
                    end
               end
           end
           MEAS_TRANSFORMED_ARRAY.ValidCumulativeMeasCount = cumsum(MEAS_TRANSFORMED_ARRAY.ValidMeasCount);
        end
        % ======================================================================================================================================================
        function [Zcts, Rcts] = ctsSnsrToEgoFrame(Zpos, Rpos, Rot, Translation)
            % Perform Coordinate transformation on the sensor data (sensor frame to ego vehicle frame)
            % INPUTS : Zpos : measurement position vector (px, py)
            %          Rpos : measurement position noise covarience matrix (2D)
            %          ROT  : eular Rotation matrix
            %          TRANS: translation vector
            % OUTPUTS : Zcts : coordinate transformed measurements vector (in the order px, py)
            %         : Rcts : rotated measurement noise covariance (in the order px, py)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            Zcts = Rot * Zpos + Translation;
            Rcts = Rot * Rpos * Rot';
            %Rcts = Rpos;
        end
        % ======================================================================================================================================================
    end
end