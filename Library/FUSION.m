% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Measurement to Track, Track to Track fusion functions library, contains the following :-
%             : AssociationHypothesis()              : Compute Association log probabilities for a Track (single sensor measurements)
%             : DATA_ASSOCIATION()                   : Data Association using Radar/Camera sensor measurements (single sensor measurements)
%             : HOMOGENEOUS_SENSOR_FUSION_RADARS()   : Sensor Fusion with multiple Radars
%             : HOMOGENEOUS_SENSOR_FUSION_CAMERAS()  : Sensor Fusion with multiple Cameras
%             : TRACK_FUSION_HETEROGENEOUS_SENSORS() : Sensor Fusion from Radar and Camera Local Tracks (Track to Track fusion)
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef FUSION
    methods(Static)
        % ======================================================================================================================================================
        function [Beta, BetaSum, MixtureComponents, nHypothesis] = AssociationHypothesis(TrackPred, ASSIGNMENT_MAT, MEAS_CTS, measmodel, snsrIdx, idxObj)
            % Compute Association log probabilities for a Track (single sensor measurements)
            % INPUTS : TrackPred: a structure with two fields:
            %                  x: predicted object state mean (state dimension) x 1 vector 
            %                  P: predicted object state covariance (state dimension) x (state dimension) matrix
            %          ASSOCIATION_MAT: structure with the following fields (single sensor measurements):
            %          AssociationMat : association matrix of log likelihoods
            %                   nMeas : number of sensor measurements
            %                MEAS_CTS : coordinate transformed measurements all sensors
            %                snsrIdx  : sensor index
            %                idxObj   : Track index
            % OUTPUTS : Beta : Normalized Log probabilities
            %           BetaSum : sum of log probabilities before normalization (product of probabilities)
            %           MixtureComponents : Gaussian mixture components after association
            %           nHypothesis : number of Hypothesis
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nHypothesis = uint16(0);         % count total number of hypothesis
            Components = struct;
            Components.x = single(zeros(4,1));
            Components.P = single(zeros(4,4));
            MixtureComponents = Components(ones(1,200));
            Beta = single(zeros(1,200));
            INVALID_WT = single(-99);
            
            nMeasSnsr = ASSIGNMENT_MAT.nMeas;
            if snsrIdx == 1  % compute the measurement index offset in the measurement matrix 
               startIndexOffet = 0;
            else
               startIndexOffet = MEAS_CTS.ValidCumulativeMeasCount(snsrIdx-1);
            end
            
            nHypothesis = nHypothesis + 1;
            MixtureComponents(1,nHypothesis) = TrackPred;    % Predicted Track for missdetection hypothesis
            Beta(1,nHypothesis) = ASSIGNMENT_MAT.AssociationMat(idxObj, int16(idxObj) + int16(nMeasSnsr)); % miss detection weight
            
            for idxMeas = 1:nMeasSnsr
                if( ASSIGNMENT_MAT.AssociationMat(idxObj, idxMeas) ~= INVALID_WT )
                    idx = startIndexOffet + idxMeas;
                    z = MEAS_CTS.MeasArray(:,idx);
                    R = MEAS_CTS.MeasCovariance(:,:,idx);
                    [~, K] = GAUSSIAN_DENSITY.InovationCovAndKalmanGain(TrackPred, measmodel, R);
                    TrackUpd = GAUSSIAN_DENSITY.update(TrackPred, z, K, measmodel);  % kalman filter update
                    nHypothesis = nHypothesis + 1;
                    MixtureComponents(1,nHypothesis) = TrackUpd;
                    Beta(1,nHypothesis) = ASSIGNMENT_MAT.AssociationMat(idxObj, idxMeas); %detection weight
                end
            end
            [Beta, BetaSum] = GAUSSIAN_DENSITY.normalizeLogWeights(Beta(1,1:nHypothesis));
        end
        % ======================================================================================================================================================
        function [TRACK_DATA, FUSION_INFO] = DATA_ASSOCIATION(TRACK_DATA_in, ASSIGNMENT_MAT, MEAS_CTS, measmodel, FUSION_INFO)
            % Data Association using Radar sensor measurements
            % INPUTS : TRACK_DATA_in  : Track Data structure
            %        : ASSIGNMENT_MAT : Track to Measurement association matrix
            %        : MEAS_CTS       : Coordinate Transformed sensor measurements
            %        : measmodel      : measurement model
            %        : FUSION_INFO    : Initialized Structure to hold Data association results
            % OUTPUT : TRACK_DATA     : Track Data
            %        : FUSION_INFO    : Data association results
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_DATA = TRACK_DATA_in;
            for row = 1:size(FUSION_INFO,1)
                for col = 1:size(FUSION_INFO, 2)
                    FUSION_INFO(row,col).Beta(:) = single(0);
                    FUSION_INFO(row,col).BetaSum = single(0);
                    FUSION_INFO(row,col).nHypothesis = single(0);
                end
            end
            StateCovIndex = [1,2,4,5];
            for idxObj = 1:TRACK_DATA.nValidTracks
                TrackPred.x = [TRACK_DATA.TrackParam(1,idxObj).StateEstimate.px;
                               TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vx;
                               TRACK_DATA.TrackParam(1,idxObj).StateEstimate.py;
                               TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vy];
                TrackPred.P =  TRACK_DATA.TrackParam(1,idxObj).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex);
                for snsrIdx = 1:length(ASSIGNMENT_MAT)
                    [Beta, BetaSum, MixtureComponents, nHypothesis] = FUSION.AssociationHypothesis(TrackPred, ASSIGNMENT_MAT(1,snsrIdx), ...
                                                                                                            MEAS_CTS,  measmodel, snsrIdx, idxObj);
                    [Xfus, Pfus] = GAUSSIAN_DENSITY.momentMatching(Beta(1,1:nHypothesis), MixtureComponents(1,1:nHypothesis));
                    %[Xfus, Pfus] = GAUSSIAN_DENSITY.WeightedAverage(Beta(1,1:nHypothesis), MixtureComponents(1,1:nHypothesis));
                    FUSION_INFO(idxObj,snsrIdx).Beta = Beta;
                    FUSION_INFO(idxObj,snsrIdx).BetaSum = BetaSum;
                    FUSION_INFO(idxObj,snsrIdx).MixtureComponents = MixtureComponents;
                    FUSION_INFO(idxObj,snsrIdx).nHypothesis = nHypothesis;
                    FUSION_INFO(idxObj,snsrIdx).x = Xfus;
                    FUSION_INFO(idxObj,snsrIdx).P = Pfus;
                end
            end
        end
        % ======================================================================================================================================================
        function [TRACK_DATA] = HOMOGENEOUS_SENSOR_FUSION_RADARS(TRACK_DATA_in, FUSION_INFO_RAD)
            % Sensor Fusion with multiple radars
            % INPUTS : TRACK_DATA_in   : Track Data structure
            %        : FUSION_INFO_RAD : Data association results from each Radars
            % OUTPUT : TRACK_DATA      : Data Fusion results (all Radar sensors)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_DATA = TRACK_DATA_in;
            if(TRACK_DATA.nValidTracks == 0)
                return;
            end
            
            StateCovIndex = [1,2,4,5]; %for state err covariance (corresponding to px, vx, py, vy)
            nRadars = size(FUSION_INFO_RAD,2); % number of cameras
            validIndex = zeros(1, nRadars);
            nHypothesis = zeros(1, nRadars);
            Beta = single(zeros(1, nRadars));
            
            for idxObj = 1:TRACK_DATA_in.nValidTracks
                % initialize some parameters to zeros
                TRACK_DATA.TrackParam(1,idxObj).Status.Predicted = false;
                TRACK_DATA.TrackParam(1,idxObj).Status.Gated = false;
                TRACK_DATA.TrackParam(1,idxObj).SensorSource.RadarCatch = false;
                TRACK_DATA.TrackParam(1,idxObj).SensorSource.RadarSource(:) = false;
                count = 0;
                for idxSnsr = 1:nRadars
                    Beta(1,idxSnsr) = FUSION_INFO_RAD(idxObj, idxSnsr).BetaSum;
                    nHypothesis(1,idxSnsr) = FUSION_INFO_RAD(idxObj, idxSnsr).nHypothesis;
                end
                if(sum(nHypothesis) == nRadars)  % only prediction
                   MixtureComponents = FUSION_INFO_RAD(idxObj, 1:nRadars);
                   [Beta, ~] = GAUSSIAN_DENSITY.normalizeLogWeights(Beta);
                   [Xfus, Pfus] = GAUSSIAN_DENSITY.momentMatching(Beta, MixtureComponents);
                   %[Xfus, Pfus] = GAUSSIAN_DENSITY.WeightedAverage(Beta, MixtureComponents);
                   %[Xfus, Pfus] = GAUSSIAN_DENSITY.CovarianceIntersection(Beta, MixtureComponents);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.px = Xfus(1,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vx = Xfus(2,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.py = Xfus(3,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vy = Xfus(4,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex) = Pfus;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Predicted = true;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Gated = false;
                   TRACK_DATA.TrackParam(1,idxObj).SensorSource.RadarCatch = false;
                   TRACK_DATA.TrackParam(1,idxObj).SensorSource.RadarSource(:) = false;
                elseif(sum(nHypothesis) > nRadars) % merge estimates from different sensors
                   for idxSnsr = 1:nRadars
                       if(FUSION_INFO_RAD(idxObj, idxSnsr).nHypothesis > 1)  % consider only those state estimate which was updated by at least one measurement
                          count = count + 1;
                          validIndex(1,count) = idxSnsr;
                          TRACK_DATA.TrackParam(1,idxObj).SensorSource.RadarSource(1,idxSnsr) = true;
                       end
                   end
                   MixtureComponents = FUSION_INFO_RAD(idxObj, validIndex(1,1:count));
                   [Beta, ~] = GAUSSIAN_DENSITY.normalizeLogWeights(Beta(1,validIndex(1,1:count)));
                   [Xfus, Pfus] = GAUSSIAN_DENSITY.momentMatching(Beta, MixtureComponents);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.px = Xfus(1,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vx = Xfus(2,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.py = Xfus(3,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vy = Xfus(4,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex) = Pfus;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Predicted = false;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Gated = true;
                   TRACK_DATA.TrackParam(1,idxObj).SensorSource.RadarCatch = true;
                end
            end
        end
        % ======================================================================================================================================================
        function [TRACK_DATA] = HOMOGENEOUS_SENSOR_FUSION_CAMERAS(TRACK_DATA_in, FUSION_INFO_CAM)
            % Sensor Fusion with multiple cameras
            % INPUTS : TRACK_DATA_in   : Track Data structure
            %        : FUSION_INFO_CAM : Data association results from each Cameras
            % OUTPUT : TRACK_DATA      : Data Fusion results (all Camera sensors)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_DATA = TRACK_DATA_in;
            if(TRACK_DATA.nValidTracks == 0)
                return;
            end
            
            StateCovIndex = [1,2,4,5]; %for state err covariance (corresponding to px, vx, py, vy)
            nCameras = size(FUSION_INFO_CAM,2); % number of cameras
            validIndex = zeros(1, nCameras);
            nHypothesis = zeros(1, nCameras);
            Beta = single(zeros(1, nCameras));
            
            for idxObj = 1:TRACK_DATA_in.nValidTracks
                % initialize some parameters to zeros
                TRACK_DATA.TrackParam(1,idxObj).Status.Predicted = false;
                TRACK_DATA.TrackParam(1,idxObj).Status.Gated = false;
                TRACK_DATA.TrackParam(1,idxObj).SensorSource.CameraCatch = false;
                TRACK_DATA.TrackParam(1,idxObj).SensorSource.CameraSource(:) = false;
                count = 0;
                for idxSnsr = 1:nCameras
                    Beta(1,idxSnsr) = FUSION_INFO_CAM(idxObj, idxSnsr).BetaSum;
                    nHypothesis(1,idxSnsr) = FUSION_INFO_CAM(idxObj, idxSnsr).nHypothesis;
                end
                if(sum(nHypothesis) == nCameras)  % only prediction
                   MixtureComponents = FUSION_INFO_CAM(idxObj, 1:nCameras);
                   [Beta, ~] = GAUSSIAN_DENSITY.normalizeLogWeights(Beta);
                   [Xfus, Pfus] = GAUSSIAN_DENSITY.momentMatching(Beta, MixtureComponents);
                   %[Xfus, Pfus] = GAUSSIAN_DENSITY.WeightedAverage(Beta, MixtureComponents);
                   %[Xfus, Pfus] = GAUSSIAN_DENSITY.CovarianceIntersection(Beta, MixtureComponents);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.px = Xfus(1,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vx = Xfus(2,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.py = Xfus(3,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vy = Xfus(4,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex) = Pfus;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Predicted = true;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Gated = false;
                   TRACK_DATA.TrackParam(1,idxObj).SensorSource.CameraCatch = false;
                   TRACK_DATA.TrackParam(1,idxObj).SensorSource.CameraSource(:) = false;
                elseif(sum(nHypothesis) > nCameras) % merge estimates from different sensors
                   for idxSnsr = 1:nCameras
                       if(FUSION_INFO_CAM(idxObj, idxSnsr).nHypothesis > 1)  % consider only those state estimate which was updated by at least one measurement
                          count = count + 1;
                          validIndex(1,count) = idxSnsr;
                          TRACK_DATA.TrackParam(1,idxObj).SensorSource.CameraSource(1,idxSnsr) = true;
                       end
                   end
                   MixtureComponents = FUSION_INFO_CAM(idxObj, validIndex(1,1:count));
                   [Beta, ~] = GAUSSIAN_DENSITY.normalizeLogWeights(Beta(1,validIndex(1,1:count)));
                   [Xfus, Pfus] = GAUSSIAN_DENSITY.momentMatching(Beta, MixtureComponents);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.px = Xfus(1,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vx = Xfus(2,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.py = Xfus(3,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.vy = Xfus(4,1);
                   TRACK_DATA.TrackParam(1,idxObj).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex) = Pfus;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Predicted = false;
                   TRACK_DATA.TrackParam(1,idxObj).Status.Gated = true;
                   TRACK_DATA.TrackParam(1,idxObj).SensorSource.CameraCatch = true;
                end
            end
        end
        % ======================================================================================================================================================
        function [TRACK_ESTIMATES_FUS] = TRACK_FUSION_HETEROGENEOUS_SENSORS(TRACK_ESTIMATES_FUS_in, TRACK_ESTIMATES_RAD, TRACK_ESTIMATES_CAM, GATED_TRACK_INFO)
            % Sensor Fusion with multiple cameras
            % INPUTS : TRACK_ESTIMATES_FUS_in : Fused Track Data structure
            %        : TRACK_ESTIMATES_RAD    : Local Track estimates from Radar sensors
            %        : TRACK_ESTIMATES_CAM    : Local Track estimates from Camera sensors
            %        : GATED_TRACK_INFO       : Local Track to Fused Track gating info
            % OUTPUT : TRACK_ESTIMATES_FUS    : Data Fusion results (Radar Tracks + Camera Tracks)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            % execute this function only if there are valid tracks
            TRACK_ESTIMATES_FUS = TRACK_ESTIMATES_FUS_in;
            if(TRACK_ESTIMATES_FUS.nValidTracks == 0)
                return;
            end
            
            % initializations
            StateCovIndex = [1,2,4,5]; 
            nLocalTracksCam = 100; nLocalTracksRad = 100; nLocalTracks = nLocalTracksCam + nLocalTracksRad;
            dim = 4; XTracks = struct; XTracks.x = single(zeros(dim, 1)); XTracks.P = single(zeros(dim, dim)); XTracks = XTracks(ones(1,nLocalTracks));
            weights = single(zeros(1, dim)); CameraSource(:) = false; RadarSource(:) = false;
            
            % Association of the local tracks with the fused track
            for idx = 1:TRACK_ESTIMATES_FUS.nValidTracks  % for each of the predicted fused tracks
                count = 0;
                nRadGatedTracks = GATED_TRACK_INFO(idx).nGatedRadarTracks;   % number of gated radar local tracks
                nCamGatedTracks = GATED_TRACK_INFO(idx).nGatedCameraTracks;  % number of gated camera local tracks
                nGatedTracks = nRadGatedTracks + nCamGatedTracks;            % total number of gated radar + camera local tracks
                GatedTrack = false; PredictedTrack = true;
                RadarCatch = false; CameraCatch = false; CameraSource(:) = false; RadarSource(:) = false;
                for i = 1:nRadGatedTracks
                    count = count + 1;
                    j = GATED_TRACK_INFO(idx).RadarTracks(1,i);
                    XTracks(1,count).x(1,1) = TRACK_ESTIMATES_RAD.TrackParam(1,j).StateEstimate.px;
                    XTracks(1,count).x(2,1) = TRACK_ESTIMATES_RAD.TrackParam(1,j).StateEstimate.vx;
                    XTracks(1,count).x(3,1) = TRACK_ESTIMATES_RAD.TrackParam(1,j).StateEstimate.py;
                    XTracks(1,count).x(4,1) = TRACK_ESTIMATES_RAD.TrackParam(1,j).StateEstimate.vy;
                    XTracks(1,count).P      = TRACK_ESTIMATES_RAD.TrackParam(1,j).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex); 
                    % update various flags here
                    RadarCatch     = (RadarCatch     || TRACK_ESTIMATES_RAD.TrackParam(1,j).SensorSource.RadarCatch);
                    RadarSource    = (RadarSource     | TRACK_ESTIMATES_RAD.TrackParam(1,j).SensorSource.RadarSource);
                    GatedTrack     = (GatedTrack     || TRACK_ESTIMATES_RAD.TrackParam(1,j).Status.Gated);
                    PredictedTrack = (PredictedTrack && TRACK_ESTIMATES_RAD.TrackParam(1,j).Status.Predicted);
                end
                for i = 1:nCamGatedTracks
                    count = count + 1;
                    j = GATED_TRACK_INFO(idx).CameraTracks(1,i);
                    XTracks(1,count).x(1,1) = TRACK_ESTIMATES_CAM.TrackParam(1,j).StateEstimate.px;
                    XTracks(1,count).x(2,1) = TRACK_ESTIMATES_CAM.TrackParam(1,j).StateEstimate.vx;
                    XTracks(1,count).x(3,1) = TRACK_ESTIMATES_CAM.TrackParam(1,j).StateEstimate.py;
                    XTracks(1,count).x(4,1) = TRACK_ESTIMATES_CAM.TrackParam(1,j).StateEstimate.vy;
                    XTracks(1,count).P      = TRACK_ESTIMATES_CAM.TrackParam(1,j).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex);
                    % update various flags here
                    CameraCatch    = (CameraCatch    || TRACK_ESTIMATES_CAM.TrackParam(1,j).SensorSource.CameraCatch);
                    CameraSource   = (CameraSource    | TRACK_ESTIMATES_CAM.TrackParam(1,j).SensorSource.CameraSource);
                    GatedTrack     = (GatedTrack     || TRACK_ESTIMATES_CAM.TrackParam(1,j).Status.Gated);
                    PredictedTrack = (PredictedTrack && TRACK_ESTIMATES_CAM.TrackParam(1,j).Status.Predicted);
                end
                if(nGatedTracks == 0) % only prediction , no fusion
                    % update various counters here
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.RadarCatch          = false;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.CameraCatch         = false;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.RadarAndCameraCatch = false;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.RadarSource(:)      = false;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.CameraSource(:)     = false;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).Status.Predicted                 = true;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).Status.Gated                     = false;
                elseif(nGatedTracks > 0)  % else track fusion
                    weights(1, 1:count) = 1/nGatedTracks;  % assign equal weights to all local sensor estimates
                    [Xfus, Pfus] = GAUSSIAN_DENSITY.CovarianceIntersection(XTracks(1, 1:count), weights(1, 1:count));
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).StateEstimate.px = Xfus(1,1); 
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).StateEstimate.vx = Xfus(2,1);
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).StateEstimate.py = Xfus(3,1);
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).StateEstimate.vy = Xfus(4,1);
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex) = Pfus;
                    RadarCameraCatch = (RadarCatch && CameraCatch);
                    % update various counters here
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.CameraCatch      = CameraCatch;  
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.RadarCatch       = RadarCatch;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.RadarCameraCatch = RadarCameraCatch;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.CameraSource     = CameraSource;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).SensorSource.RadarSource      = RadarSource;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).Status.Predicted              = PredictedTrack;
                    TRACK_ESTIMATES_FUS.TrackParam(1,idx).Status.Gated                  = GatedTrack;
                end
            end
        end
        % ======================================================================================================================================================
    end
end