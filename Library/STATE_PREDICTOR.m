% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : State prediction function library, Contains the following functions
%             : EgoCompensationCV()    : Track Parameters Ego motion compensation of position and velocity 
%             : EgoCompensationCA()    : Track Parameters Ego motion compensation of position, velocity and acceleration
%             : LINEAR_PROCESS_MODEL() : Linear Prediction of the Tracks
%             : EGO_COMPENSATION()     : Ego motion compensation of the Tracks
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef STATE_PREDICTOR
    methods(Static)
        % ======================================================================================================================================================
        function StateIntertialFrame  = EgoCompensationCV(CompensationMatrix, x)
            % performs ego morion compensation of the track state by considering only px, py, vx, vy
            % INPUT  : CompensationMatrix : Transposed rotation matrix for compensation operation
            %        : x : state vector in the order (px, vx, py, vy)
            % OUTPUT : StateIntertialFrame : ego compensated state vector in the order (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            posIndex = [1,3]; velIndex = [2,4];
            StateIntertialFrame(posIndex,1) = CompensationMatrix * x(posIndex,1);  % ego motion compensation of the position components
            StateIntertialFrame(velIndex,1) = CompensationMatrix * x(velIndex,1);  % ego motion compemsation of the velocity components
        end
        % ======================================================================================================================================================
        function StateIntertialFrame  = EgoCompensationCA(CompensationMatrix, x)
            % performs ego motion compensation of the track state by considering only px, py, vx, vy, ax, ay
            % INPUT : CompensationMatrix : Transposed rotation matrix for compensation operation
            %       : x : state vector in the order (px, vx,ax, py, vy, ay)
            % OUTPUT : StateIntertialFrame : ego compensated state vector in the order (px, vx, ax, py, vy, ay)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            posIndex = [1,4]; velIndex = [2,5]; accIndex = [3,6];
            StateIntertialFrame(posIndex,1) = CompensationMatrix * x(posIndex,1);  % ego compensation of the position components
            StateIntertialFrame(velIndex,1) = CompensationMatrix * x(velIndex,1);  % ego compemsation of the velocity components
            StateIntertialFrame(accIndex,1) = CompensationMatrix * x(accIndex,1);  % ego compemsation of the acceleration components
        end
        % ======================================================================================================================================================
        function [TRACK_STATE_ESTIMATES] = LINEAR_PROCESS_MODEL(TRACK_STATE_ESTIMATES_in, motionmodel)
            % State prediction of the tracked objects using camera state estimates
            % INPUTS : TRACK_STATE_ESTIMATES :
            %        : nValidObj :
            %        : motionmodel :
            % OUTPUT : TRACK_STATE_ESTIMATES
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_STATE_ESTIMATES = TRACK_STATE_ESTIMATES_in;
            if(TRACK_STATE_ESTIMATES.nValidTracks == 0) % do not perform state prediction if no objects are present
                return;
            end
            for idx = 1:TRACK_STATE_ESTIMATES.nValidTracks
                % prediction of the track estimate
                if(strcmp(motionmodel.name, 'ca'))
                   state.x = [TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px; ... 
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx; ...
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ax; ...
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py; ...
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy; ...
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ay];
                   state.P  = TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ErrCOV;
                elseif(strcmp(motionmodel.name, 'cv'))
                   state.x = [TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px; ...
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx; ...
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py; ...
                              TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy];
                   covIndex = [1,2,4,5];
                   state.P  = TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ErrCOV(covIndex,covIndex);
                end
                state = GAUSSIAN_DENSITY.predict(state, motionmodel);
                % Write back (Fusion Prediction)
                if(strcmp(motionmodel.name, 'ca')) 
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px = state.x(1,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx = state.x(2,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ax = state.x(3,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py = state.x(4,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy = state.x(5,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ay = state.x(6,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ErrCOV = state.P;
                elseif(strcmp(motionmodel.name, 'cv'))
                   covIndex = [1,2,4,5];
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px = state.x(1,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx = state.x(2,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py = state.x(3,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy = state.x(4,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ax = single(0);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ay = single(0);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ErrCOV(covIndex,covIndex) = state.P;
                end
            end
        end
        % ======================================================================================================================================================
        function [TRACK_STATE_ESTIMATES] = EGO_COMPENSATION(TRACK_STATE_ESTIMATES_in, EGO_SENSOR_DATA, dT, model)
            % Perform ego compensation of the state from t-1 before prediction. Due to the yaw rate of the ego vehicle, the ego
            % vehicle would have turned by an angle yaw, to represent the prediction in the current ego vehicle frame , we compensate
            % the track position from t-1 by an angle deltaYAW
            % INPUTS: TRACK_STATE_ESTIMATES : Structure of the track state estimates from the previous cycle
            %         nValidObj             : number of valid objects
            %         EGO_SENSOR_DATA       : ego vehicle data
            %         dT                    : sample time
            %         model                 : indicates if ego compensation needs to be applied to acceleration also
            % OUTPUT: TRACK_STATE_ESTIMATES : ego compensated track state estimates
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_STATE_ESTIMATES = TRACK_STATE_ESTIMATES_in;
            if(strcmp(model, 'none') || TRACK_STATE_ESTIMATES.nValidTracks == 0) % do not perform ego compensation
                return;
            end
            dYaw = EGO_SENSOR_DATA.yawRate * dT; %DEG2RAD = single(pi/180); dYaw = dYaw*DEG2RAD;
            yawRateCompensationMat = [ cos(-dYaw), -sin(-dYaw); ...
                                       sin(-dYaw),  cos(-dYaw)];
            for idx = 1:TRACK_STATE_ESTIMATES.nValidTracks                                                     
                % px,py, vx,vy, ax,ay ego compensation
                if(strcmp(model, 'cv'))
                    x = [TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy];
                    x = STATE_PREDICTOR.EgoCompensationCV(yawRateCompensationMat, x);
                elseif(strcmp(model, 'ca'))
                    x = [TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ax; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy; ...
                         TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ay];
                    x = STATE_PREDICTOR.EgoCompensationCA(yawRateCompensationMat, x); 
                end
                % write back  
                if(strcmp(model, 'cv'))
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px = x(1,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx = x(2,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py = x(3,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy = x(4,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ax = single(0);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ay = single(0);
                elseif(strcmp(model, 'ca'))
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.px = x(1,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vx = x(2,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ax = x(3,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.py = x(4,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.vy = x(5,1);
                   TRACK_STATE_ESTIMATES.TrackParam(1,idx).StateEstimate.ay = x(6,1);
                end
            end
        end
        % ======================================================================================================================================================
    end
end