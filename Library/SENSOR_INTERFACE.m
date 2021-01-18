% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Functions to arrange sensor data
%             : isMeasurementValid()        : Check if a measurement is not junk
%             : set_RADAR_SENSOR_MEAS_DATA  : set the radar measurement data as an array of structures
%             : set_CAMERA_SENSOR_MEAS_DATA : set the camera measurement data as an array of structures
%             : RAD_SENSOR_INTERFACE        : set radar sensor interface
%             : CAM_SENSOR_INTERFACE        : set camera sensor interface
%             : EGO_SENSOR_INTERFACE        : set ego sensor interface
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef SENSOR_INTERFACE
    methods(Static)
        % ======================================================================================================================================================
        function [isValid] = isMeasurementValid(px, py)
            % Check of the measurements are valid
            % INPUT  : px, py : measurement from a sensor
            % OUTPUT : valid measurement flag
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            if( (abs(px) + abs(py) ) >= 0.000001)
                 isValid = true;
            else
                 isValid = false;
            end
        end
        % ======================================================================================================================================================
        function [RADAR_CAN_BUS] = set_RADAR_SENSOR_MEAS_DATA(RAD_Sensor_Simulated_Data, timeIdx, snsrIdx, nMeas, RADAR_CAN_BUS)
            % Set Radar Sensor Data with the Array of Structure
            % INPUT : RAD_Sensor_Simulated_Data : a structure of 2D arrays of simulated sensor data
            %         timeIdx : time index
            %         snsrIdx : radar sensor index
            %         nMeas :  maximum number of measurements possible
            %         RADAR_CAN_BUS : initialized array of structure of radar sensor measurements
            % OUTPUT : RADAR_CAN_BUS : array of structure of radar sensor measurements with the following fields
            %        : px, py : measurements
            %        : measNoise : measurement noise covariance
            %        : measID : valid meas id
            %        : sensorID : sensor ID that gave the detection
            %        : detTimeStamp : detected time of the measurement
            %        : snr : signal to noise ratio (applicable only for radar)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nValidMeas = int16(0);
            %MeasNoiseCov = single([0.3, 0; ...   % (px, py)
            %                       0, 0.6]);
            MeasNoiseCov = single([2   , 0; 
                                   0  , 3.3]);
            for objIdx = 1:nMeas
                if( SENSOR_INTERFACE.isMeasurementValid(RAD_Sensor_Simulated_Data.px(timeIdx,objIdx), ...
                                                        RAD_Sensor_Simulated_Data.py(timeIdx,objIdx)))
                    nValidMeas = nValidMeas + int16(1);
                    RADAR_CAN_BUS(snsrIdx, objIdx).px = single(RAD_Sensor_Simulated_Data.px(timeIdx,objIdx));
                    RADAR_CAN_BUS(snsrIdx, objIdx).py = single(RAD_Sensor_Simulated_Data.py(timeIdx,objIdx));
                    azimuth = atan(RADAR_CAN_BUS(snsrIdx, objIdx).py/RADAR_CAN_BUS(snsrIdx, objIdx).px);
                    Rotation = [cos(azimuth), -sin(azimuth);...
                                sin(azimuth), cos(azimuth)];
                    MeasNoiseCov = Rotation * MeasNoiseCov * Rotation';
                    RADAR_CAN_BUS(snsrIdx, objIdx).measNoise = MeasNoiseCov;
                    RADAR_CAN_BUS(snsrIdx, objIdx).measID = uint16(nValidMeas);
                    RADAR_CAN_BUS(snsrIdx, objIdx).sensorID = uint8(snsrIdx);
                    RADAR_CAN_BUS(snsrIdx, objIdx).detTimeStamp = single(RAD_Sensor_Simulated_Data.detTimeStamp(timeIdx,1)); 
                    RADAR_CAN_BUS(snsrIdx, objIdx).snr = single(RAD_Sensor_Simulated_Data.snr(timeIdx,objIdx));
                else
                    RADAR_CAN_BUS(snsrIdx, objIdx).px = single(0);
                    RADAR_CAN_BUS(snsrIdx, objIdx).py = single(0);
                    RADAR_CAN_BUS(snsrIdx, objIdx).measNoise = single(zeros(size(MeasNoiseCov)));
                    RADAR_CAN_BUS(snsrIdx, objIdx).measID = uint16(0);
                    RADAR_CAN_BUS(snsrIdx, objIdx).sensorID = uint8(snsrIdx);
                    RADAR_CAN_BUS(snsrIdx, objIdx).detTimeStamp = single(0); 
                    RADAR_CAN_BUS(snsrIdx, objIdx).snr = single(0);
                end 
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
             %        : px, py, vx, vy : measurements
             %        : measNoise : measurement noise covariance
             %        : measID : valid meas id
             %        : sensorID : sensor ID that gave the detection
             %        : detTimeStamp : detected time of the measurement
             %        : objClassID : object classification (applicable only for camera)
             % --------------------------------------------------------------------------------------------------------------------------------------------------
             nValidMeas = int16(0);
             %MeasNoiseCov = single([0.7, 0; ... % (px, py)
             %                       0, 0.4]);
             MeasNoiseCov = single([3.7, 0; 
                                    0  , 2.1]);
             for objIdx = 1:nMeas
                 if( SENSOR_INTERFACE.isMeasurementValid(CAM_Sensor_Simulated_Data.px(timeIdx,objIdx), ...
                                                         CAM_Sensor_Simulated_Data.py(timeIdx,objIdx)))
                     nValidMeas = nValidMeas + int16(1);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).px = single(CAM_Sensor_Simulated_Data.px(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).py = single(CAM_Sensor_Simulated_Data.py(timeIdx,objIdx));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).measNoise = MeasNoiseCov;
                     CAMERA_CAN_BUS(snsrIdx, objIdx).measID = uint16(nValidMeas);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).sensorID = uint8(snsrIdx);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).detTimeStamp = single(CAM_Sensor_Simulated_Data.detTimeStamp(timeIdx,1)); 
                     CAMERA_CAN_BUS(snsrIdx, objIdx).objClassID = single(CAM_Sensor_Simulated_Data.objClassID(timeIdx,objIdx));
                 else
                     CAMERA_CAN_BUS(snsrIdx, objIdx).px = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).py = single(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).measNoise = single(zeros(size(MeasNoiseCov)));
                     CAMERA_CAN_BUS(snsrIdx, objIdx).measID = uint16(0);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).sensorID = uint8(snsrIdx);
                     CAMERA_CAN_BUS(snsrIdx, objIdx).detTimeStamp = single(0); 
                     CAMERA_CAN_BUS(snsrIdx, objIdx).objClassID = single(0);
                 end  
             end                                                  
         end
         % =====================================================================================================================================================
         function [RADAR_CAN_BUS] = RAD_SENSOR_INTERFACE(RAD1_Sensor_Simulated_Data, ...
                                                         RAD2_Sensor_Simulated_Data, ...
                                                         RAD3_Sensor_Simulated_Data, ...
                                                         RAD4_Sensor_Simulated_Data, ...
                                                         RAD5_Sensor_Simulated_Data, ...
                                                         RAD6_Sensor_Simulated_Data, ...
                                                         RADAR_CAN_BUS, t, nRadars, nMeas)
             % Interface Radar Sensor Data with the Array of Structure
             % INPUT  : RAD1_Sensor_Simulated_Data : Radar 1 simulated measurements
             %          RAD2_Sensor_Simulated_Data : Radar 2 simulated measurements
             %          RAD3_Sensor_Simulated_Data : Radar 3 simulated measurements
             %          RAD4_Sensor_Simulated_Data : Radar 4 simulated measurements
             %          RAD5_Sensor_Simulated_Data : Radar 5 simulated measurements
             %          RAD6_Sensor_Simulated_Data : Radar 6 simulated measurements
             %          RADAR_CAN_BUS : init array of structure of radar measurements
             %          t : time index
             %          nRadars : number of radars
             %          nMeas : number of measurements
             % OUTPUT : RADAR_CAN_BUS : array of structure of radar measurements
             % -------------------------------------------------------------------------------------------------------------------------------------------------
             for snsrIdx = 1:nRadars
                 switch(snsrIdx)
                     case 1
                         RADAR_CAN_BUS = SENSOR_INTERFACE.set_RADAR_SENSOR_MEAS_DATA(RAD1_Sensor_Simulated_Data, t, snsrIdx, nMeas, RADAR_CAN_BUS);
                     case 2
                         RADAR_CAN_BUS = SENSOR_INTERFACE.set_RADAR_SENSOR_MEAS_DATA(RAD2_Sensor_Simulated_Data, t, snsrIdx, nMeas, RADAR_CAN_BUS);
                     case 3
                         RADAR_CAN_BUS = SENSOR_INTERFACE.set_RADAR_SENSOR_MEAS_DATA(RAD3_Sensor_Simulated_Data, t, snsrIdx, nMeas, RADAR_CAN_BUS);
                     case 4
                         RADAR_CAN_BUS = SENSOR_INTERFACE.set_RADAR_SENSOR_MEAS_DATA(RAD4_Sensor_Simulated_Data, t, snsrIdx, nMeas, RADAR_CAN_BUS);
                     case 5
                         RADAR_CAN_BUS = SENSOR_INTERFACE.set_RADAR_SENSOR_MEAS_DATA(RAD5_Sensor_Simulated_Data, t, snsrIdx, nMeas, RADAR_CAN_BUS);
                     case 6
                         RADAR_CAN_BUS = SENSOR_INTERFACE.set_RADAR_SENSOR_MEAS_DATA(RAD6_Sensor_Simulated_Data, t, snsrIdx, nMeas, RADAR_CAN_BUS);
                     otherwise
                         error('number of radars exceeds the upper limit');
                 end            
              end
         end
         % =====================================================================================================================================================
         function [CAMERA_CAN_BUS] = CAM_SENSOR_INTERFACE(CAM1_Sensor_Simulated_Data, ...
                                                          CAM2_Sensor_Simulated_Data, ...
                                                          CAM3_Sensor_Simulated_Data, ...
                                                          CAM4_Sensor_Simulated_Data, ...
                                                          CAM5_Sensor_Simulated_Data, ...
                                                          CAM6_Sensor_Simulated_Data, ...
                                                          CAM7_Sensor_Simulated_Data, ...
                                                          CAM8_Sensor_Simulated_Data, ...
                                                          CAMERA_CAN_BUS, t, nCameras, nMeas)
             % Interface Camera Sensor Data with the Array of Structure
             % INPUT  : CAM1_Sensor_Simulated_Data : Camera 1 simulated measurements
             %          CAM2_Sensor_Simulated_Data : Camera 2 simulated measurements
             %          CAM3_Sensor_Simulated_Data : Camera 3 simulated measurements
             %          CAM4_Sensor_Simulated_Data : Camera 4 simulated measurements
             %          CAM5_Sensor_Simulated_Data : Camera 5 simulated measurements
             %          CAM6_Sensor_Simulated_Data : Camera 6 simulated measurements
             %          CAM7_Sensor_Simulated_Data : Camera 7 simulated measurements
             %          CAM8_Sensor_Simulated_Data : Camera 8 simulated measurements
             %          CAMERA_CAN_BUS : init array of structure of camera measurements
             %          t : time index
             %          nCameras : number of cameras
             %          nMeas : number of measurements
             % OUTPUT : CAMERA_CAN_BUS : array of structure of camera measurements
             % -------------------------------------------------------------------------------------------------------------------------------------------------
             for snsrIdx = 1:nCameras
                 switch(snsrIdx)
                     case 1
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM1_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 2
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM2_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 3
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM3_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 4
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM4_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 5
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM5_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 6
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM6_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 7
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM7_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
                     case 8
                         CAMERA_CAN_BUS = SENSOR_INTERFACE.set_CAMERA_SENSOR_MEAS_DATA(CAM8_Sensor_Simulated_Data, t, snsrIdx, nMeas, CAMERA_CAN_BUS);
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
    end
end