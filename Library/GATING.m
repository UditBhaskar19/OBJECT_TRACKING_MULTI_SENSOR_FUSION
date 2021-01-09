% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Measurement to Track and Track to Track Gating function library, contains the following functions
%             : GatingAndLogLikelihood()             :  Perform Ellipsoidal Gating and Compute the likelihood of the predicted measurement in logarithmic scale
%             : REMOVE_GATING_AMBUIGITY()            :  Remove gating ambiguity : It is assumed that each measurement is originated from a single target
%             : GATE_MEASUREMENTS()                  :  Gating of Sensor measurements , create association matrix for track and sensor measurements
%             : FIND_GATED_MEASUREMENT_INDEX()       :  extract the measurement indexes from the gated measurement clusters
%             : FIND_UNGATED_CLUSTERS()              :  Find the list of unassociated radar and camera cluster centers
%             : GATE_FUSED_TRACK_WITH_LOCAL_TRACKS() :  Gating of fused tracks with the local tracks
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef GATING
    methods(Static)
        % ======================================================================================================================================================
        function [LogLikelihood, isGated] = GatingAndLogLikelihood(Z, measmodel, state_pred, P_D, GammaSq)
            % Perform Ellipsoidal Gating and Compute the likelihood of the predicted measurement in logarithmic scale
            % Ellipsoidal Gating is performed individually for (px, py) AND (vx, vy)
            % INPUT :       z : measurement , struct with 2 fields, x: meas vector, R: meas noise covariance
            %        measmodel: a structure specifies the measurement model parameters  
            %       state_pred: a structure with two fields:
            %                x: predicted object state mean (state dimension) x 1 vector 
            %                P: predicted object state covariance (state dimension) x (state dimension) matrix
            %              P_D: probability of detection
            %  GammaPosSquare : Square of Gamma for position based ellipsoidal gating
            %  GammaVelSquare : Square of Gamma for velocity based ellipsoidal gating
            % OUTPUT : LogLikelihood : log likelihood of the predicted state (valid only if the the state is gated with a measurement)
            %        :       isGated : boolean flag indicating if the measurement is gated
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            INVALID = single(99);
            yk = Z.x - measmodel.h(state_pred.x);
            H = measmodel.H(state_pred.x);   % Measurement model Jacobian
            S = H * state_pred.P * H' + Z.R; % Innovation covariance
            S = (S + S')/2;                  % Make sure matrix S is positive definite
            mDist = yk' * (S\yk);            % mahalanobis distance
            measDim = size(Z.x,1);           % measurement dimension
            if measDim == 1
               Ck = 2;
            elseif measDim == 2
               Ck = pi;
            elseif measDim == 3
               Ck = 4*pi/3;
            elseif measDim == 4
               Ck = pi^2/2;
            end
            if (mDist <= GammaSq)
                Vk = Ck*sqrt(det(GammaSq*S));
                LogLikelihood = log(P_D) + log(Vk) - 0.5*log(det(2*pi*S)) - 0.5*mDist; % - log(numGatedMeas);
                isGated = true;
            else
                LogLikelihood = -INVALID;
                isGated = false;
            end
        end
        % ======================================================================================================================================================
        function [ASSOCIATION_MAT] = REMOVE_GATING_AMBUIGITY(ASSOCIATION_MAT, nValidTracks, nMeasSnsr)
            % Remove gating ambiguity : It is assumed that each measurement is originated from a single target , i.e no measurements are shared. 
            % If a measurement is shared by multiple tracks, the track corresponding to maximum likelihood is kept and the other likelihoods are discarded. 
            % Alternate logics include JPDAF, Murtys-k best assignment, Hungarian Assignment, but this one is the simpliest and computationally efficient
            % INPUTS : ASSOCIATION_MAT : Track to measurement association matrix for a single sensor measurements
            %        : nValidTracks : number of valid tracks
            %        : nMeasSnsr : number of valid measurements per sensor
            % OUTPUTS : ASSOCIATION_MAT : Gating Ambiguity removed Track to measurement association matrix
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            INVALID = single(-99);
            for idxMeas = 1:nMeasSnsr
                [maxLikelihood, index] = max(ASSOCIATION_MAT(1:nValidTracks,idxMeas));
                if(maxLikelihood > INVALID - 1)
                   ASSOCIATION_MAT(1:nValidTracks, idxMeas) = INVALID;
                   ASSOCIATION_MAT(index, idxMeas) = maxLikelihood;
                end
            end
        end
        % ======================================================================================================================================================
        function [ASSIGNMENT_MAT, GATED_MEAS_INDEX] = GATE_MEASUREMENTS(Track_Estimates, Measurements, measmodel, ...
                                                                        SensorParam, GammaSq,  motionmodel, P_G, ...
                                                                        ASSOCIATION_MAT, ASSIGNMENT_MAT, GATED_MEAS_INDEX)
            % Gating of Sensor measurements , create association matrix for track and sensor measurements
            % separate functions might be needed for radar and camera , because the gating procedure might be different for sensors with different modalities 
            % INPUTS : Track_Estimates    : Estimated Track Data Structure 
            %        : Measurements       : Coordinate Transformed Radar measurements
            %        : measmodel          : Measurement model 
            %        : SensorParam        : sensor intrinsic and extrinsic parameters
            %        : GammaSq            : Gating Threshold
            %        : motionmodel        : Motion Model
            %        : P_G                : probability of gating
            %        : ASSOCIATION_MAT    : (INITIALIZED) Association matrix (nTracks , nTracks + nMeas)
            %        : ASSIGNMENT_MAT     : (INITIALIZED) Structure holding the ASSOCIATION_MAT for each radar sensors
            %        : GATED_MEAS_INDEX   : (INITIALIZED) boolean flag array indicating gated measurement indexes
            % OUTPUT : ASSIGNMENT_MAT     : (UPDATED) Structure holding the ASSOCIATION_MAT for each radar sensors
            %          GATED_MEAS_INDEX   : (UPDATED) boolean flag array indicating gated measurement indexes
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nValidTracks = Track_Estimates.nValidTracks;
            nMeas = Measurements.ValidCumulativeMeasCount(end); % Total number of clusters
            GATED_MEAS_INDEX(:) = false;
            for idx = 1:length(ASSIGNMENT_MAT)
                ASSIGNMENT_MAT(1,idx).AssociationMat(:) = single(0);
                ASSIGNMENT_MAT(1,idx).nMeas = single(0);
            end
            if(nValidTracks == 0 || nMeas == 0)  % do not perform gating if no objects or no measurements are present
               return;
            end
            INVALID = single(-99);
            for snsrIdx = 1:length(SensorParam.Extrinsic)     % iterate over each of the sensors      
                nMeasSnsr = Measurements.ValidMeasCount(snsrIdx);  % extract the number of measurements returned by the sensor
                %if(nMeasSnsr ~= 0)  % if the sensor has returned measurements then only do the following                              
                   if snsrIdx == 1  % compute the measurement index offset in the measurement matrix 
                      startIndexOffet = 0;
                   else
                      startIndexOffet = Measurements.ValidCumulativeMeasCount(snsrIdx-1);
                   end
                   % extract P_D, Lambda and init the association matrix
                   P_D = SensorParam.Intrinsic(1, SensorParam.Extrinsic(1,snsrIdx).SensorType).ProbOfDetection;
                   ASSOCIATION_MAT(1:nValidTracks, 1:(int16(nValidTracks) + int16(nMeasSnsr))) = INVALID; % may not be required
                   for objIdx = 1:nValidTracks % for each of the objects extract the state data and perform gating
                       if(strcmp(motionmodel.name, 'ca'))
                          statePred.x = [Track_Estimates.TrackParam(1,objIdx).StateEstimate.px; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.vx; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.ax; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.py; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.vy; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.ay];
                          statePred.P =  Track_Estimates.TrackParam(1,objIdx).StateEstimate.ErrCOV;
                       elseif(strcmp(motionmodel.name, 'cv'))
                          covIndex = [1,2,4,5];
                          statePred.x = [Track_Estimates.TrackParam(1,objIdx).StateEstimate.px; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.vx; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.py; ...
                                         Track_Estimates.TrackParam(1,objIdx).StateEstimate.vy];
                          statePred.P =  Track_Estimates.TrackParam(1,objIdx).StateEstimate.ErrCOV(covIndex,covIndex);
                       end
                       ASSOCIATION_MAT(objIdx, int16(objIdx) + int16(nMeasSnsr)) = log(1-P_D*P_G); % cost for missdetection
                       nGatedMeas = 0;
                       for idxMeas = 1:nMeasSnsr
                           measIndex = startIndexOffet + idxMeas;
                           z.x = Measurements.MeasArray(:,measIndex);
                           z.R = Measurements.MeasCovariance(:,:,measIndex);
                           [LogLikelihood, isGated] = GATING.GatingAndLogLikelihood(z, measmodel, statePred, P_D, GammaSq);
                           ASSOCIATION_MAT(objIdx, idxMeas) = LogLikelihood;
                           if(isGated)
                              GATED_MEAS_INDEX(1,measIndex) = isGated;
                              nGatedMeas = nGatedMeas + 1;
                           end
                       end
                       for idxMeas = 1:nMeasSnsr
                           measIndex = startIndexOffet + idxMeas;
                           if(GATED_MEAS_INDEX(1,measIndex))
                              %ASSOCIATION_MAT(objIdx, idxMeas) = ASSOCIATION_MAT(objIdx, idxMeas) - log(nGatedMeas); 
                           end
                       end
                   end
                   [ASSOCIATION_MAT] = GATING.REMOVE_GATING_AMBUIGITY(ASSOCIATION_MAT, nValidTracks, nMeasSnsr); % Resolve the Gate
                   ASSIGNMENT_MAT(1,snsrIdx).AssociationMat = ASSOCIATION_MAT;
                   ASSIGNMENT_MAT(1,snsrIdx).nMeas = nMeasSnsr;
                %end
            end
        end
        % ======================================================================================================================================================
        function [GATED_MEAS_INDEX] = FIND_GATED_MEASUREMENT_INDEX(GATED_MEAS_INDEX, SENSOR_MEASUREMENTS, ...
                                                                   GATED_CLUSTER_INDEX, CLUSTER_MEASUREMENTS, CLUSTERS)                                                
            % extract the measurement indexes from the gated measurement clusters
            % INPUTS : GATED_CLUSTER_INDEX  : initialized gated measurememnt index array
            %          SENSOR_MEASUREMENTS  : sensor measurements (for details refer to 'SensorFusion_Script3_LOAD_DATA_STRUCTURE_PARAMETERS')
            %          GATED_CLUSTER_INDEX  : index of the gated clusters
            %          CLUSTER_MEASUREMENTS : Measurement clusters from each sensors
            %          CLUSTERS             : measurement clusters from all sensors
            % OUTPUTS: GATED_MEAS_INDEX     : gated measurememnt index array updated
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            % GATED_MEAS_INDEX(1,1:nSnsrMeas) = false;
            GATED_MEAS_INDEX(:) = false;
            nSnsrClusters = CLUSTER_MEASUREMENTS.ValidCumulativeMeasCount(end);
            nSnsrMeas = SENSOR_MEASUREMENTS.ValidCumulativeMeasCount(end);
            GATED_CLSTR_INDEX_LIST = find(GATED_CLUSTER_INDEX(1,1:nSnsrClusters) ~= 0);   %list of gated radar cluster index
            GATED_CLSTR_LIST = CLUSTER_MEASUREMENTS.ClusterRef(1,GATED_CLSTR_INDEX_LIST); %list of gated radar cluster ID
            GATED_CLSTR_LIST = unique(GATED_CLSTR_LIST);
            if(~isempty(GATED_CLSTR_LIST))
                for idx = 1:length(GATED_CLSTR_LIST)
                    GATED_MEAS_INDEX_temp = find(CLUSTERS.ClustIDAssig(1,1:nSnsrMeas) == GATED_CLSTR_LIST(idx));  % extract the measurement index
                    GATED_MEAS_INDEX(1, GATED_MEAS_INDEX_temp) = true;  % indicate those measurements as gated
                end
            end
        end
        % ======================================================================================================================================================
        function [UNASSOCIATED_CLUSTERS, cntMeasClst] = FIND_UNGATED_CLUSTERS(nSnsrMeas, GATED_MEAS_INDEX, CLUSTERS_MEAS, UNASSOCIATED_CLUSTERS)
            % Find the list of unassociated radar and camera cluster centers
            % INPUTS : nSnsrMeas             : number of sensor measurements
            %          GATED_MEAS_INDEX      : gated measurememnt index array
            %          CLUSTERS_MEAS         : Measurement clusters from each sensors
            %          UNASSOCIATED_CLUSTERS : Unassociated clusters initialized data structure
            % OUTPUTS: UNASSOCIATED_CLUSTERS : Unassociated clusters updated data structure
            %          cntMeasClst           : number of ungated easurement clusters
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            UNASSOCIATED_CLUSTERS(:) = uint16(0);
            cntMeasClst = uint16(0);
            if(nSnsrMeas ~= 0)
               UNGATED_MEAS_INDEX_LIST = find(GATED_MEAS_INDEX(1,1:nSnsrMeas) == 0); % list of ungated sensor index
               nUngatedMeas = length(UNGATED_MEAS_INDEX_LIST);                       % number of ungated radar meas
               isMeasVisited = false(1,nSnsrMeas);                        
               for idx = 1:nUngatedMeas
                   ungatedMeasIdx = UNGATED_MEAS_INDEX_LIST(1,idx);
                   if(~isMeasVisited(1,ungatedMeasIdx))
                       clusterID = CLUSTERS_MEAS.ClustIDAssig(1,ungatedMeasIdx);     % Cluster ID
                       cntMeasClst = cntMeasClst + 1;
                       UNASSOCIATED_CLUSTERS(1,cntMeasClst) = clusterID;
                       MeasList = (CLUSTERS_MEAS.ClustIDAssig == clusterID);         % find the measurement index with the same radar cluster ID
                       isMeasVisited(1,MeasList) = true;
                   end
               end
            end
        end
        % ======================================================================================================================================================
        function [GATED_TRACK_INFO, UNGATED_TRACK_INFO] = GATE_FUSED_TRACK_WITH_LOCAL_TRACKS(TRACK_ESTIMATES_FUS, TRACK_ESTIMATES_RAD, TRACK_ESTIMATES_CAM)
            % Gating of fused tracks with the local tracks
            % INPUTS : TRACK_ESTIMATES_FUS   : Fused Track predictions
            %          TRACK_ESTIMATES_RAD   : Local Track estimates from Radar sensor 
            %          TRACK_ESTIMATES_CAM   : Local Track estimates from Camera sensor 
            % OUTPUTS: GATED_TRACK_INFO      : Unassociated clusters updated data structure
            %          UNGATED_TRACK_INFO    : number of ungated easurement clusters
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            % initialize the Gated info
            maxNumFusedTracks = uint16(100); maxNumLocalTracks = uint16(100);
            GATED_TRACK_INFO = struct;
            GATED_TRACK_INFO.nGatedRadarTracks = uint16(0);
            GATED_TRACK_INFO.nGatedCameraTracks = uint16(0);
            GATED_TRACK_INFO.RadarTracks = uint16(zeros(1, maxNumLocalTracks));
            GATED_TRACK_INFO.CameraTracks = uint16(zeros(1, maxNumLocalTracks));
            GATED_TRACK_INFO = GATED_TRACK_INFO(ones(1,maxNumFusedTracks));
            
            % initialize the UnGated info
            UNGATED_TRACK_INFO = struct;
            UNGATED_TRACK_INFO.UngatedRadarTracks = true(1, maxNumLocalTracks);
            UNGATED_TRACK_INFO.UngatedCameraTracks = true(1, maxNumLocalTracks);
            
            % if the local tracks are available and the fused tracks are also available then execute this function
            if(((TRACK_ESTIMATES_RAD.nValidTracks == 0) && (TRACK_ESTIMATES_CAM.nValidTracks == 0)) || (TRACK_ESTIMATES_FUS.nValidTracks == 0))
                return;
            end
            
            % init structures
            GammaSqPos = 16; %GammaSqVel = 10;
            posCovIdx = [1,4]; velCovIdx = [2,5];
            
            xFusPos = struct;                 xFusVel = struct;                 xLocalPos = struct;                 xLocalVel   = struct;
            xFusPos.x = single(zeros(2,1));   xFusVel.x = single(zeros(2,1));   xLocalPos.x = single(zeros(2,1));   xLocalVel.x = single(zeros(2,1));
            xFusPos.P = single(zeros(2,2));   xFusVel.P = single(zeros(2,2));   xLocalPos.R = single(zeros(2,2));   xLocalVel.R = single(zeros(2,2));
            
            % create the association matrix for radar local tracks
            for i = 1:TRACK_ESTIMATES_FUS.nValidTracks
                nGatedTracksRad = uint16(0); nGatedTracksCam = uint16(0);
                xFusPos.x(1,1) = TRACK_ESTIMATES_FUS.TrackParam(i).StateEstimate.px;  
                xFusPos.x(2,1) = TRACK_ESTIMATES_FUS.TrackParam(i).StateEstimate.py;
                xFusPos.P      = TRACK_ESTIMATES_FUS.TrackParam(i).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                xFusVel.x(1,1) = TRACK_ESTIMATES_FUS.TrackParam(i).StateEstimate.vx;  
                xFusVel.x(2,1) = TRACK_ESTIMATES_FUS.TrackParam(i).StateEstimate.vy;
                xFusVel.P      = TRACK_ESTIMATES_FUS.TrackParam(i).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                for j = 1:TRACK_ESTIMATES_RAD.nValidTracks % for each of the radar tracks 
                    xLocalPos.x(1,1) = TRACK_ESTIMATES_RAD.TrackParam(j).StateEstimate.px;  
                    xLocalPos.x(2,1) = TRACK_ESTIMATES_RAD.TrackParam(j).StateEstimate.py;
                    xLocalPos.P      = TRACK_ESTIMATES_RAD.TrackParam(j).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                    xLocalVel.x(1,1) = TRACK_ESTIMATES_RAD.TrackParam(j).StateEstimate.vx;  
                    xLocalVel.x(2,1) = TRACK_ESTIMATES_RAD.TrackParam(j).StateEstimate.vy;
                    xLocalVel.P      = TRACK_ESTIMATES_RAD.TrackParam(j).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                    dist = sqrt((xFusPos.x(1,1) - xLocalPos.x(1,1))^2 + (xFusPos.x(2,1) - xLocalPos.x(2,1))^2);
                    if(dist <= sqrt(GammaSqPos)) % if Gated set the gated track info
                       nGatedTracksRad = nGatedTracksRad + 1;
                       GATED_TRACK_INFO(i).RadarTracks(1,nGatedTracksRad) = j;
                       UNGATED_TRACK_INFO.UngatedRadarTracks(1,j) = false;
                    end
                end
                for j = 1:TRACK_ESTIMATES_CAM.nValidTracks % for each of the camera tracks 
                    xLocalPos.x(1,1) = TRACK_ESTIMATES_CAM.TrackParam(j).StateEstimate.px;  
                    xLocalPos.x(2,1) = TRACK_ESTIMATES_CAM.TrackParam(j).StateEstimate.py;
                    xLocalPos.P      = TRACK_ESTIMATES_CAM.TrackParam(j).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                    xLocalVel.x(1,1) = TRACK_ESTIMATES_CAM.TrackParam(j).StateEstimate.vx;  
                    xLocalVel.x(2,1) = TRACK_ESTIMATES_CAM.TrackParam(j).StateEstimate.vy;
                    xLocalVel.P      = TRACK_ESTIMATES_CAM.TrackParam(j).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                    dist = sqrt((xFusPos.x(1,1) - xLocalPos.x(1,1))^2 + (xFusPos.x(2,1) - xLocalPos.x(2,1))^2);
                    if(dist <= sqrt(GammaSqPos))
                       nGatedTracksCam = nGatedTracksCam + 1;
                       GATED_TRACK_INFO(i).CameraTracks(1,nGatedTracksCam) = j;
                       UNGATED_TRACK_INFO.UngatedCameraTracks(1,j) = false;
                    end
                end
                % Update the Gated meas count info
                GATED_TRACK_INFO(i).nGatedRadarTracks = nGatedTracksRad;
                GATED_TRACK_INFO(i).nGatedCameraTracks = nGatedTracksCam;
            end
        end
        % ======================================================================================================================================================
    end
end