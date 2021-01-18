% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Track Management function library , Contains the following functions :
%             : ComputeTrackInitScore()             : Use counter based logic for computing the score for Track initialization
%             : ChooseNewTrackID()                  : Select a unique ID for labeling a Track
%             : SelectAndReuseLostTrackID()         : Reuse the ID for the lost Track
%             : INIT_NEW_TRACK()                    : Initialize a new Track by assigning a unique ID and setting Track parameters
%             : MAINTAIN_EXISTING_TRACK()           : Maintain the existing tracks
%             : DELETE_LOST_TRACK()                 : Delete lost track info and reuse the track ID
%             : SET_NEW_TRACK_INFO()                : Set new Fused Track info from the local tracks
%             : FORM_NEW_TRACKS_FROM_LOCAL_TRACKS() : Group ungated local tracks for determination of new fused track 
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef TRACK_MANAGER
    methods(Static)
        % ======================================================================================================================================================
        function [TrackInitScore, TRACK_DATA] = ComputeTrackInitScore(TRACK_DATA, idx, dT, alpha1, alpha2)
            % Use counter based logic for computing the score for Track initialization recursively
            % An ungated measurement is initialized as a new track and a initialization score is recursively computed , if it is above
            % a threshold then the track is set as a 'confirmed' track
            % INPUTS  : TRACK_DATA : data structure corresponding to Track Data ( for details refer to the script 'SensorFusion_Script3_LOAD_DATA_STRUCTURE_PARAMETERS.m')
            %         : idx        : a track index for refering to the track 'TRACK_DATA.TrackParam(idx)'
            %         : dT         : sample time
            %         : alpha1     : threshold for track gated counter
            %         : alpha2     : threshold for sum of gated and predicted counter
            % OUTPUTS : TRACK_DATA : Track data structure containing the updated track management data
            %         : TrackInitScore : Computed track initialization score
            % -------------------------------------------------------------------------------------------------------------------------------------------------- 
            if(TRACK_DATA.TrackParam(1,idx).Status.Gated)
              TRACK_DATA.TrackParam(1,idx).Quality.GatedCounter = TRACK_DATA.TrackParam(1,idx).Quality.GatedCounter + 1;
            elseif(TRACK_DATA.TrackParam(1,idx).Status.Predicted)
              TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter = TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter + 1;
            end
            TRACK_DATA.TrackParam(1,idx).Quality.TrackedTime = TRACK_DATA.TrackParam(1,idx).Quality.TrackedTime + dT;
            Gt = TRACK_DATA.TrackParam(1,idx).Quality.GatedCounter;
            Pt = TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter;
            St = Gt + Pt;
            if(St <= alpha2)
               if(Gt >= alpha1)
                  TrackInitScore = 1; % track is confirmed
               else
                  TrackInitScore = 2; % track is still new
               end
            elseif(St > alpha2)
                  TrackInitScore = 3; % track is Lost
            end
        end
        % ======================================================================================================================================================
        function [NewID, TRACK_DATA] = ChooseNewTrackID(TRACK_DATA) 
            % Select a unique ID for Track initialization
            % INPUTS  : TRACK_DATA : data structure corresponding to Track Data, Track Management data is located here
            % OUTPUTS : TRACK_DATA : Track data structure containing the updated track management data
            %         : NewID : Unique track id which shall be used for track initialization
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            maxTrackID = uint16(100); minTrackID = uint16(1);
            NewID = TRACK_DATA.TrackIDList(1,TRACK_DATA.FirstAvailableIDindex);       % choose a new id 
            TRACK_DATA.TrackIDList(1,TRACK_DATA.FirstAvailableIDindex) = uint16(0);   % set it as '0' since it has been used
            TRACK_DATA.IsTrackIDused(1, TRACK_DATA.FirstAvailableIDindex) = true;     % mark the id as 'used'
            if(TRACK_DATA.FirstAvailableIDindex == maxTrackID)
                TRACK_DATA.FirstAvailableIDindex = minTrackID;
            else
                TRACK_DATA.FirstAvailableIDindex = TRACK_DATA.FirstAvailableIDindex + 1;
            end
        end
        % ======================================================================================================================================================
        function TRACK_DATA = SelectAndReuseLostTrackID(TRACK_DATA, idx)
            % Select and Recycle the ID of the Track which is no longer of any interest
            % INPUTS  : TRACK_DATA : data structure corresponding to Track Data, Track Management data is located here
            %         : idx        : a track index for refering to the track 'TRACK_DATA.TrackParam(idx)' , id is extracted for this track  
            % OUTPUTS : TRACK_DATA : Track data structure containing the updated track management data
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            maxTrackID = uint16(100); minTrackID = uint16(1);
            LostTrackID = TRACK_DATA.TrackParam(1,idx).id;
            LastIDindex = TRACK_DATA.LastAvailableIDindex;
            FirstIDindex = TRACK_DATA.FirstAvailableIDindex;
            % compute the last index where the lost Track ID shall be placed
            if(LastIDindex == maxTrackID)
               LastIDindex = minTrackID;
            else
               LastIDindex = LastIDindex + 1;
            end
            LostIDindex = find(TRACK_DATA.TrackIDList(1,LastIDindex:(FirstIDindex-1)) == LostTrackID);
            temp = TRACK_DATA.TrackIDList(1,LastIDindex);
            TRACK_DATA.TrackIDList(1,LastIDindex) = LostTrackID;
            TRACK_DATA.TrackIDList(1,LostIDindex) = temp;
            temp = TRACK_DATA.IsTrackIDused(1, LastIDindex);
            TRACK_DATA.IsTrackIDused(1, LastIDindex) = false;
            TRACK_DATA.IsTrackIDused(1, LostIDindex) = temp;
            TRACK_DATA.LastAvailableIDindex = LastIDindex;
        end
        % ======================================================================================================================================================
        function [TRACK_DATA, nObjNew] = INIT_NEW_TRACK(CLUSTERS_MEAS, UNASSOCIATED_CLUSTERS, cntMeasClst, TRACK_DATA_in, dT)
            % Set New Track parameters from the unassociated clusters, the
            % Clusters are from the measurements of either radar and camera sensors.
            % Track Initialization of the Local Tracks from Radar and Camera Sensors systems
            % INPUTS : CLUSTERS_MEAS         : Measurement clusters , ( for details refer to the script 'SensorFusion_Script3_LOAD_DATA_STRUCTURE_PARAMETERS.m')
            %        : UNASSOCIATED_CLUSTERS : measurement clusters not gated with any existing/new tracks
            %        : cntMeasClst           : number of ungated measurement clusters
            %        : TRACK_DATA_in         : data structure corresponding to Track Data
            %        : dT                    : sampling time
            % OUTPUTS : TRACK_DATA : Updated Track Data
            %         : nObjNew    : number of new tracks
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nObjNew = 0;
            TRACK_DATA = TRACK_DATA_in;
            if(TRACK_DATA.nValidTracks == 0 && cntMeasClst == 0) % if no unassociated clusters and valid objects are present then do not set new track 
               return;
            end
            posCovIdx = [1,4]; velCovIdx = [2,5]; %StateCovIndex = [1,2,4,5];
            sigmaSq = 2; alpha1 = 5; alpha2 = 8;
            objIndex = TRACK_DATA.nValidTracks;
            
            for idx = 1:TRACK_DATA.nValidTracks  % if the track is a 'new' track update the track init function
                if(TRACK_DATA.TrackParam(1,idx).Status.New)
                   [TrackInitScore, TRACK_DATA] = TRACK_MANAGER.ComputeTrackInitScore(TRACK_DATA, idx, dT, alpha1, alpha2);
                    if(TrackInitScore == 1)       % set the track as 'confirmed' track
                       TRACK_DATA.TrackParam(1,idx).Status.New = false;       % the track is no more 'new'
                       TRACK_DATA.TrackParam(1,idx).Status.Existing = true;   % the track is existing/confirmed
                       TRACK_DATA.TrackParam(1,idx).Status.Lost = false;      % the track is not lost
                       TRACK_DATA.TrackParam(1,idx).Quality.GatedCounter =  uint16(0);    % reset the gated counter
                       TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter = uint16(0); % reset the predicted counter
                    elseif(TrackInitScore == 2)   % keep it as 'new' track
                       TRACK_DATA.TrackParam(1,idx).Status.New = true;        % the track is still 'new'
                       TRACK_DATA.TrackParam(1,idx).Status.Existing = false;  % the track is not existing/stll not confirmed
                       TRACK_DATA.TrackParam(1,idx).Status.Lost = false;      % the track is not lost
                    elseif(TrackInitScore == 3) % tag the track status as 'lost' for deletion
                       TRACK_DATA.TrackParam(1,idx).Status.New = false;       % the track is no more 'new'
                       TRACK_DATA.TrackParam(1,idx).Status.Existing = false;  % the track is not existing
                       TRACK_DATA.TrackParam(1,idx).Status.Lost = true;       % the track is lost
                     end
                end
            end
            
            for idx = 1:cntMeasClst % iterate over each of the unassocisted measurement cluster
                MeasClstID = UNASSOCIATED_CLUSTERS(1,idx);
                nObjNew = nObjNew + 1;
                index = objIndex + nObjNew;
                % choose a new Track ID
                [newId, TRACK_DATA] = TRACK_MANAGER.ChooseNewTrackID(TRACK_DATA);     
                TRACK_DATA.TrackParam(1,index).id = newId;              % assign a new ID to the new Track
                % Update the Track Status , sensor catch info , and tracked time
                TRACK_DATA.TrackParam(1,index).Status.New = true;
                TRACK_DATA.TrackParam(1,index).Status.Existing = false;
                TRACK_DATA.TrackParam(1,index).Status.Lost = false;
                TRACK_DATA.TrackParam(1,index).Status.Gated = true;
                TRACK_DATA.TrackParam(1,index).Quality.TrackedTime = TRACK_DATA.TrackParam(1,index).Quality.TrackedTime + dT;
                TRACK_DATA.TrackParam(1,index).Quality.GatedCounter = TRACK_DATA.TrackParam(1,index).Quality.GatedCounter + 1;
                % Update Track Estimates
                TRACK_DATA.TrackParam(1,index).StateEstimate.px = CLUSTERS_MEAS.ClusterCenters(1,MeasClstID);
                TRACK_DATA.TrackParam(1,index).StateEstimate.py = CLUSTERS_MEAS.ClusterCenters(2,MeasClstID);
                TRACK_DATA.TrackParam(1,index).StateEstimate.vx = single(0);
                TRACK_DATA.TrackParam(1,index).StateEstimate.vy = single(0);
                TRACK_DATA.TrackParam(1,index).StateEstimate.ax = single(0);
                TRACK_DATA.TrackParam(1,index).StateEstimate.ay = single(0);
                TRACK_DATA.TrackParam(1,index).StateEstimate.ErrCOV(posCovIdx,posCovIdx) = CLUSTERS_MEAS.ClusterCovariance(:,:,MeasClstID);
                TRACK_DATA.TrackParam(1,index).StateEstimate.ErrCOV(velCovIdx,velCovIdx) = single([sigmaSq,0; 0,sigmaSq]);
            end
            TRACK_DATA.nValidTracks = TRACK_DATA.nValidTracks + nObjNew;
        end
        % ======================================================================================================================================================
        function [TRACK_DATA] = MAINTAIN_EXISTING_TRACK(TRACK_DATA_in, dT)
            % Maintain the existing Track information
            % INPUTS : TRACK_DATA_in         : data structure corresponding to Track Data
            %        : dT                    : sampling time
            % OUTPUTS : TRACK_DATA : Updated Track Data
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_DATA = TRACK_DATA_in;
            if(TRACK_DATA.nValidTracks == 0) % if no unassociated clusters are present then do not execute this function 
                return;
            end
            thresholdPredCounter = 60; % delete if the track is not gated for 3 seconds continuously
            for idx = 1:TRACK_DATA.nValidTracks
                if(TRACK_DATA.TrackParam(1,idx).Status.Existing)
                   if(TRACK_DATA.TrackParam(1,idx).Status.Gated) % if the track gets gated once reset the predicted counter to 0
                      %TRACK_DATA.TrackParam(1,idx).Quality.GatedCounter = TRACK_DATA.TrackParam(1,idx).Quality.GatedCounter + 1;
                      TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter = uint16(0); 
                   elseif(TRACK_DATA.TrackParam(1,idx).Status.Predicted) % else increment the predicted counter
                      TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter = TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter + 1;
                   end
                   if(TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter >= thresholdPredCounter) % if consecutive predicted count is >= threshold then delete
                      TRACK_DATA.TrackParam(1,idx).Status.Lost = true;
                      TRACK_DATA.TrackParam(1,idx).Status.Existing = false;
                   end
                   TRACK_DATA.TrackParam(1,idx).Quality.TrackedTime = TRACK_DATA.TrackParam(1,idx).Quality.TrackedTime + dT;
                end
            end
        end
        % ======================================================================================================================================================
        function [TRACK_DATA, LostTrackIDs] = DELETE_LOST_TRACK(TRACK_DATA_in, TrackParamInit) 
            % Delete lost track info and reuse the track ID
            % INPUTS  : TRACK_DATA_in  : data structure corresponding to Track Data
            %         : TrackParamInit : track parameters initialized, ( for details refer to the script 'SensorFusion_Script3_LOAD_DATA_STRUCTURE_PARAMETERS.m')
            % OUTPUTS : TRACK_DATA    : Updated Track Data excluding the Lost Track
            %         : LostTrackIDs  : List of IDs from the lost Track 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_DATA = TRACK_DATA_in;
            if(TRACK_DATA_in.nValidTracks == 0) % if no unassociated clusters are present then do not set new track 
                return;
            end
            nTracksLost = 0; nSurvivingTracks = 0;
            LostTrackIDs = uint16(zeros(1,100));
            for idx = 1:TRACK_DATA_in.nValidTracks
                TRACK_DATA.TrackParam(1,idx) = TrackParamInit;
                if(~TRACK_DATA_in.TrackParam(1,idx).Status.Lost)    % set the track data if the track is not lost
                    nSurvivingTracks = nSurvivingTracks + 1;
                    TRACK_DATA.TrackParam(1,nSurvivingTracks) = TRACK_DATA_in.TrackParam(1,idx);
                elseif(TRACK_DATA_in.TrackParam(1,idx).Status.Lost) % reuse the Track IDs if the track is lost
                   nTracksLost = nTracksLost + 1;
                   LostTrackIDs(1,nTracksLost) = TRACK_DATA_in.TrackParam(1,idx).id;
                   TRACK_DATA_in = TRACK_MANAGER.SelectAndReuseLostTrackID(TRACK_DATA_in, idx);
                end
            end
            TRACK_DATA.nValidTracks  = TRACK_DATA_in.nValidTracks - nTracksLost;
            TRACK_DATA.TrackIDList   = TRACK_DATA_in.TrackIDList;       
            TRACK_DATA.IsTrackIDused = TRACK_DATA_in.IsTrackIDused;        
            TRACK_DATA.FirstAvailableIDindex = TRACK_DATA_in.FirstAvailableIDindex;        
            TRACK_DATA.LastAvailableIDindex  = TRACK_DATA_in.LastAvailableIDindex;
        end
        % ======================================================================================================================================================
        function [TRACK_DATA] = SET_NEW_TRACK_INFO(TRACK_DATA_in, FUSED_TRACKS, nNewTracks, dT)
            % Set new track info (Specifically for TRACK to TRACK fusion)
            % INPUTS  : TRACK_DATA_in  : data structure corresponding to Track Data
            %         : FUSED_TRACKS   : New Track info for the fused tracks
            %         : nNewTracks     : number of new Tracks
            %         : dT             : sampling time
            % OUTPUTS : TRACK_DATA    : Updated Track Data excluding the Lost Track
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            TRACK_DATA = TRACK_DATA_in;
            if(TRACK_DATA.nValidTracks == 0 && nNewTracks == 0) % if no unassociated clusters and valid objects are present then do not set new track 
               return;
            end
            
            StateParamIndex = [1,2,4,5];
            nObjNew = 0;
            objIndex = TRACK_DATA.nValidTracks;
            alpha1 = 5; alpha2 = 8;
            
            for idx = 1:TRACK_DATA.nValidTracks   % if the track is a 'new' track update the track init function
                if(TRACK_DATA.TrackParam(1,idx).Status.New)
                   [TrackInitScore, TRACK_DATA] = TRACK_MANAGER.ComputeTrackInitScore(TRACK_DATA, idx, dT, alpha1, alpha2);
                    if(TrackInitScore == 1)       % set the track as 'confirmed' track
                       TRACK_DATA.TrackParam(1,idx).Status.New = false;       % the track is no more 'new'
                       TRACK_DATA.TrackParam(1,idx).Status.Existing = true;   % the track is existing/confirmed
                       TRACK_DATA.TrackParam(1,idx).Status.Lost = false;      % the track is not lost
                       TRACK_DATA.TrackParam(1,idx).Quality.GatedCounter =  uint16(0);    % reset the gated counter
                       TRACK_DATA.TrackParam(1,idx).Quality.PredictedCounter = uint16(0); % reset the predicted counter
                    elseif(TrackInitScore == 2)   % keep it as 'new' track
                       TRACK_DATA.TrackParam(1,idx).Status.New = true;        % the track is still 'new'
                       TRACK_DATA.TrackParam(1,idx).Status.Existing = false;  % the track is not existing/stll not confirmed
                       TRACK_DATA.TrackParam(1,idx).Status.Lost = false;      % the track is not lost
                    elseif(TrackInitScore == 3)   % tag the track status as 'lost' for deletion
                       TRACK_DATA.TrackParam(1,idx).Status.New = false;       % the track is no more 'new'
                       TRACK_DATA.TrackParam(1,idx).Status.Existing = false;  % the track is not existing
                       TRACK_DATA.TrackParam(1,idx).Status.Lost = true;       % the track is lost
                    end
                end
            end
            
            for idx = 1:nNewTracks % iterate over each of the unassocisted Local Tracks
                nObjNew = nObjNew + 1;
                index = objIndex + nObjNew;
                % Choose a new Track ID
                [newId, TRACK_DATA] = TRACK_MANAGER.ChooseNewTrackID(TRACK_DATA);     
                TRACK_DATA.TrackParam(1,index).id = newId; % assign a new ID to the new Track
                % Update the Track Status , sensor catch info , and tracked time
                TRACK_DATA.TrackParam(1,index).SensorSource.RadarCatch       = FUSED_TRACKS(1,idx).RadarCatch;
                TRACK_DATA.TrackParam(1,index).SensorSource.CameraCatch      = FUSED_TRACKS(1,idx).CameraCatch;
                TRACK_DATA.TrackParam(1,index).SensorSource.RadarSource      = FUSED_TRACKS(1,idx).RadarSource;
                TRACK_DATA.TrackParam(1,index).SensorSource.CameraSource     = FUSED_TRACKS(1,idx).CameraCatch;
                TRACK_DATA.TrackParam(1,index).SensorSource.RadarCameraCatch = FUSED_TRACKS(1,idx).RadarCameraCatch;
                TRACK_DATA.TrackParam(1,index).Status.New                    = FUSED_TRACKS(1,idx).New;
                TRACK_DATA.TrackParam(1,index).Status.Existing               = FUSED_TRACKS(1,idx).Existing;
                TRACK_DATA.TrackParam(1,index).Status.Predicted              = FUSED_TRACKS(1,idx).Predicted;
                TRACK_DATA.TrackParam(1,index).Status.Gated                  = FUSED_TRACKS(1,idx).Gated;
                TRACK_DATA.TrackParam(1,index).Quality.TrackedTime           = TRACK_DATA.TrackParam(1,index).Quality.TrackedTime + dT;
                TRACK_DATA.TrackParam(1,index).Quality.GatedCounter          = TRACK_DATA.TrackParam(1,index).Quality.GatedCounter + 1;
                % Update Track Estimates
                TRACK_DATA.TrackParam(1,index).StateEstimate.px = FUSED_TRACKS(1,idx).Xfus(1,1);
                TRACK_DATA.TrackParam(1,index).StateEstimate.vx = FUSED_TRACKS(1,idx).Xfus(2,1);
                TRACK_DATA.TrackParam(1,index).StateEstimate.py = FUSED_TRACKS(1,idx).Xfus(3,1);
                TRACK_DATA.TrackParam(1,index).StateEstimate.vy = FUSED_TRACKS(1,idx).Xfus(4,1);
                TRACK_DATA.TrackParam(1,index).StateEstimate.ax = single(0);
                TRACK_DATA.TrackParam(1,index).StateEstimate.ay = single(0);
                TRACK_DATA.TrackParam(1,index).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex) = FUSED_TRACKS(1,idx).Pfus;
            end
            TRACK_DATA.nValidTracks = TRACK_DATA.nValidTracks + nObjNew;
        end
        % ======================================================================================================================================================
        function [FUSED_TRACKS, nNewTracks] = FORM_NEW_TRACKS_FROM_LOCAL_TRACKS(TRACK_DATA_RAD, TRACK_DATA_CAM, UNGATED_TRACK_INFO)
            % Group ungated local tracks for determination of new fused track 
            % INPUTS  : TRACK_DATA_RAD     : data structure corresponding to Track Data from radar sensors
            %         : TRACK_DATA_CAM     : data structure corresponding to Track Data from camera sensors
            %         : UNGATED_TRACK_INFO : Ungated Local Track info (Camera Local Tracks and Radar Local Tracks)        
            % OUTPUTS : FUSED_TRACKS   : New Track info for the fused tracks
            %         : nNewTracks     : number of new Tracks
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nNewTracks = 0;
            % if the number of local tracks is '0', then do not execute this function
            if( (TRACK_DATA_RAD.nValidTracks == 0) && (TRACK_DATA_CAM.nValidTracks == 0) )
                return;
            end
            
            % ungated radar and camera track indexes
            UnGatedRadTrackIdx = find(UNGATED_TRACK_INFO.UngatedRadarTracks(1, 1:TRACK_DATA_RAD.nValidTracks) == true);
            UnGatedCamTrackIdx = find(UNGATED_TRACK_INFO.UngatedCameraTracks(1, 1:TRACK_DATA_CAM.nValidTracks) == true);
            nUngatedTracksRAD  = length(UnGatedRadTrackIdx); nUngatedTracksCAM = length(UnGatedCamTrackIdx);
          
            % Initialize data structure for New Merged Tracks (Currently these parameters are updated, the remaining parameters shall be updated later)
            dim = 4; nRadars = 6; nCameras = 8; nLocalTracks = 100; nFusedTracks = 100;
            FUSED_TRACKS = struct;
            % Track kinematics
            FUSED_TRACKS.Xfus = single(zeros(dim,1));     % px, vx, py, vy of the fused track
            FUSED_TRACKS.Pfus = single(zeros(dim,dim));   % noise covariance of the estimated fused track
            FUSED_TRACKS.Xrad = single(zeros(dim,1));     % px, vx, py, vy of the radar track     
            FUSED_TRACKS.Prad = single(zeros(dim,dim));   % noise covariance of the radar track   
            FUSED_TRACKS.Xcam = single(zeros(dim,1));     % px, vx, py, vy of the camera track
            FUSED_TRACKS.Pcam = single(zeros(dim,dim));   % noise covariance of the camera track   
            % Sensor catches 
            FUSED_TRACKS.CameraCatch = false;              % is the track estimated from the camera measurements
            FUSED_TRACKS.RadarCatch = false;               % is the track estimated from the radar measurements 
            FUSED_TRACKS.RadarCameraCatch = false;         % is the track estimated from Radar & Camera measurements 
            FUSED_TRACKS.CameraSource = false(1,nCameras); % camera sensors that detected the fused track
            FUSED_TRACKS.RadarSource = false(1,nRadars);   % radar sensors that detected the fused track
            % Track Status Parameters
            FUSED_TRACKS.New = false;        % is the fused track new (it is new if all the associated local tracks are new)
            FUSED_TRACKS.Existing = false;   % it is existing if at least one associated local track is 'existing' 
            FUSED_TRACKS.Predicted = false;  % it is predicted if all all the associated local tracks are predicted
            FUSED_TRACKS.Gated = false;      % it is gated if atleast one local track is 'gated'
            FUSED_TRACKS = FUSED_TRACKS(ones(1, nFusedTracks));
           
            % if we dont have any ungated local tracks then do not execute this function
            if( nUngatedTracksRAD == 0 && nUngatedTracksCAM == 0 )
               return;
            end
            
            % initialization of data structures for algorithm execution
            nNewTracks = 0;
            CameraTrackIDs = uint16(zeros(1, nLocalTracks));
            RadarTrackIDs = uint16(zeros(1, nLocalTracks));
            isCameraTrackGrouped = false(1, nLocalTracks);
            isRadarTrackGrouped = false(1, nLocalTracks);
            X_i = single(zeros(dim, 1)); X_j = single(zeros(dim, 1)); Xfus = single(zeros(dim, 1)); Xrad = single(zeros(dim, 1)); Xcam = single(zeros(dim, 1));
            Pfus = single(zeros(dim, dim)); Pspread = single(zeros(dim, dim));
            StateParamIndex = [1,2,4,5]; posCovIdx = [1,4]; velCovIdx = [2,5];
            gammaPos = 10; gammaVel = 10; %gamma = 100.2767; 
            
            % Start the grouping
            for ii = 1:nUngatedTracksCAM                % loop over only the ungated tracks
                nCamTracks = 0;                         % Used later for grouping/merging
                i = UnGatedCamTrackIdx(1,ii);
                if(~isCameraTrackGrouped(i))
                    isCameraTrackGrouped(i) = true;
                    nCamTracks = nCamTracks + 1;
                    CameraTrackIDs(1, nCamTracks) = i;  % Update the Camera Track ID here (Used later for grouping)
                    % Track State from Camera Track 'i'
                    X_i(1,1) = TRACK_DATA_CAM.TrackParam(i).StateEstimate.px;
                    X_i(2,1) = TRACK_DATA_CAM.TrackParam(i).StateEstimate.vx;
                    X_i(3,1) = TRACK_DATA_CAM.TrackParam(i).StateEstimate.py;
                    X_i(4,1) = TRACK_DATA_CAM.TrackParam(i).StateEstimate.vy;
                    %P_i     = TRACK_DATA_CAM.TrackParam(i).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                    P_i_pos  = TRACK_DATA_CAM.TrackParam(i).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                    P_i_vel  = TRACK_DATA_CAM.TrackParam(i).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                    % Find all radar Tracks 'j' which are gated with the camera track 'i'
                    nRadTracks = 0; 
                    for jj = 1:nUngatedTracksRAD
                        j = UnGatedRadTrackIdx(1,jj);
                        if(~isRadarTrackGrouped(j))
                          % Track State from Radar Track 'j'
                          X_j(1,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.px;
                          X_j(2,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.vx;
                          X_j(3,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.py;
                          X_j(4,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.vy;
                          %P_j     = TRACK_DATA_RAD.TrackParam(j).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                          P_j_pos  = TRACK_DATA_RAD.TrackParam(j).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                          P_j_vel  = TRACK_DATA_RAD.TrackParam(j).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                          % compute the statistical distance between the Radar Track j and Camera Track i
                          %X = X_i - X_j; P = P_i + P_j;
                          Xpos = X_i([1,3],1) - X_j([1,3],1); Ppos = P_i_pos + P_j_pos;
                          Xvel = X_i([2,4],1) - X_j([2,4],1); Pvel = P_i_vel + P_j_vel;
                          %dist = X' * (P\X); % Statistical dist
                          distPos = Xpos' * (Ppos\Xpos);
                          distVel = Xvel' * (Pvel\Xvel);
                          if(distPos <= gammaPos  && distVel <= gammaVel)
                             isRadarTrackGrouped(j) = true;
                             nRadTracks = nRadTracks + 1;
                             RadarTrackIDs(1, nRadTracks) = j;  % Update the Radar Track ID here (Used later for grouping)
                          end
                       end
                   end
                   % Find all camera Tracks 'j' which are gated with the camera track 'i'
                   for jj = (ii+1):nUngatedTracksCAM  
                       j  = UnGatedCamTrackIdx(1,jj);
                       if(~isCameraTrackGrouped(j))
                          % Track State from Camera Track 'j'
                          X_j(1,1) = TRACK_DATA_CAM.TrackParam(j).StateEstimate.px;
                          X_j(2,1) = TRACK_DATA_CAM.TrackParam(j).StateEstimate.vx;
                          X_j(3,1) = TRACK_DATA_CAM.TrackParam(j).StateEstimate.py;
                          X_j(4,1) = TRACK_DATA_CAM.TrackParam(j).StateEstimate.vy;
                          %P_j     = TRACK_DATA_CAM.TrackParam(j).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                          P_j_pos  = TRACK_DATA_CAM.TrackParam(j).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                          P_j_vel  = TRACK_DATA_CAM.TrackParam(j).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                          % compute the statistical distance between the Radar Track j and Camera Track i
                          %X = X_i - X_j; P = P_i + P_j;
                          Xpos = X_i([1,3],1) - X_j([1,3],1); Ppos = P_i_pos + P_j_pos;
                          Xvel = X_i([2,4],1) - X_j([2,4],1); Pvel = P_i_vel + P_j_vel;
                          %dist = X' * (P\X); % Statistical dist
                          distPos = Xpos' * (Ppos\Xpos);
                          distVel = Xvel' * (Pvel\Xvel);
                          if(distPos <= gammaPos && distVel <= gammaVel)
                             isCameraTrackGrouped(j) = true;
                             nCamTracks = nCamTracks + 1;
                             CameraTrackIDs(1, nCamTracks) = j; % Update the Camera Track ID here (Used later for grouping)
                          end
                       end
                   end
                   
                   % Compute the Track Cluster estimates (This track has either a camera only cluster of both Radar and Camera)
                   Xfus(:) = single(0); Xrad(:) = single(0); Xcam(:) = single(0);
                   Pfus(:) = single(0); Prad(:) = single(0); Pcam(:) = single(0); Pspread(:) = single(0);
                   NewTrack = true; ExistingTrack = false; GatedTrack = false; PredictedTrack = true;
                   RadarCatch = false; CameraCatch = false; CameraSource(:) = false; RadarSource(:) = false;
                   nLocalTracks = nCamTracks + nRadTracks;
                   weight = 1/nLocalTracks;
                   for idx = 1:nCamTracks  % weighted mean and covariance from camera tracks
                       index = CameraTrackIDs(1, idx);
                       X_i(1,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.px;
                       X_i(2,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.vx;
                       X_i(3,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.py;
                       X_i(4,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.vy;
                       P_i = TRACK_DATA_CAM.TrackParam(index).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                       Xfus = Xfus + weight.* X_i;
                       Pfus = Pfus + weight.* P_i;
                       Xcam = X_i; Pcam = P_i;
                       CameraCatch    = (CameraCatch    ||  TRACK_DATA_CAM.TrackParam(index).SensorSource.CameraCatch);
                       CameraSource   = (CameraSource    |  TRACK_DATA_CAM.TrackParam(index).SensorSource.CameraSource);
                       NewTrack       = (NewTrack       &&  TRACK_DATA_CAM.TrackParam(index).Status.New);
                       ExistingTrack  = (ExistingTrack  ||  TRACK_DATA_CAM.TrackParam(index).Status.Existing);
                       GatedTrack     = (GatedTrack     ||  TRACK_DATA_CAM.TrackParam(index).Status.Gated);
                       PredictedTrack = (PredictedTrack &&  TRACK_DATA_CAM.TrackParam(index).Status.Predicted);
                   end
                   for idx = 1:nRadTracks  % weighted mean and covariance from radar tracks
                       index = RadarTrackIDs(1, idx);
                       X_i(1,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.px;
                       X_i(2,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vx;
                       X_i(3,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.py;
                       X_i(4,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vy;
                       P_i = TRACK_DATA_RAD.TrackParam(index).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                       Xfus = Xfus + weight.* X_i;
                       Pfus = Pfus + weight.* P_i;
                       Xrad = X_i; Prad = P_i;
                       RadarCatch     = (RadarCatch     || TRACK_DATA_RAD.TrackParam(index).SensorSource.RadarCatch);
                       RadarSource    = (RadarSource     | TRACK_DATA_RAD.TrackParam(index).SensorSource.RadarSource);
                       NewTrack       = (NewTrack       && TRACK_DATA_RAD.TrackParam(index).Status.New);
                       ExistingTrack  = (ExistingTrack  || TRACK_DATA_RAD.TrackParam(index).Status.Existing);
                       GatedTrack     = (GatedTrack     || TRACK_DATA_RAD.TrackParam(index).Status.Gated);
                       PredictedTrack = (PredictedTrack && TRACK_DATA_RAD.TrackParam(index).Status.Predicted);
                   end
                   NewTrack = ~ExistingTrack;
                   RadarCameraCatch = (RadarCatch && CameraCatch);
                   for idx = 1:nCamTracks % spread of the mean
                       index = CameraTrackIDs(1, idx);
                       X_i(1,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.px;
                       X_i(2,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.vx;
                       X_i(3,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.py;
                       X_i(4,1) = TRACK_DATA_CAM.TrackParam(index).StateEstimate.vy;
                       Pspread = Pspread + weight.*( (Xfus - X_i)*(Xfus - X_i)' );
                   end
                   for idx = 1:nRadTracks % spread of the mean
                       index = RadarTrackIDs(1, idx);
                       X_i(1,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.px;
                       X_i(2,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vx;
                       X_i(3,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.py;
                       X_i(4,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vy;
                       Pspread = Pspread + weight.*( (Xfus - X_i)*(Xfus - X_i)' );
                   end
                   Pfus = Pfus + Pspread;
                   CameraTrackIDs(:) = uint16(0); RadarTrackIDs(:) = uint16(0);  % reset to 0
                   % update the Merged Track in the output
                   nNewTracks = nNewTracks + 1;
                   FUSED_TRACKS(1, nNewTracks).Xfus = Xfus;   FUSED_TRACKS(1, nNewTracks).Xrad = Xrad;   FUSED_TRACKS(1, nNewTracks).Xcam = Xcam;
                   FUSED_TRACKS(1, nNewTracks).Pfus = Pfus;   FUSED_TRACKS(1, nNewTracks).Prad = Prad;   FUSED_TRACKS(1, nNewTracks).Pcam = Pcam;
                   FUSED_TRACKS(1, nNewTracks).CameraCatch      = CameraCatch;  
                   FUSED_TRACKS(1, nNewTracks).RadarCatch       = RadarCatch;
                   FUSED_TRACKS(1, nNewTracks).RadarCameraCatch = RadarCameraCatch;
                   FUSED_TRACKS(1, nNewTracks).CameraSource     = CameraSource;
                   FUSED_TRACKS(1, nNewTracks).RadarSource      = RadarSource;
                   FUSED_TRACKS(1, nNewTracks).New              = NewTrack;
                   FUSED_TRACKS(1, nNewTracks).Existing         = ExistingTrack;
                   FUSED_TRACKS(1, nNewTracks).Predicted        = PredictedTrack;
                   FUSED_TRACKS(1, nNewTracks).Gated            = GatedTrack;             
                end
            end
            
            for ii = 1:nUngatedTracksRAD
                nRadTracks = 0;
                i = UnGatedRadTrackIdx(1,ii); 
                if(~isRadarTrackGrouped(i))% && UnGatedRadTracks(i))
                    isRadarTrackGrouped(i) = true;
                    nRadTracks = nRadTracks + 1;
                    RadarTrackIDs(1, nRadTracks) = i; % Update the Radar Track ID here (Used later for grouping)
                    % Track State from Radar Track 'i'
                    X_i(1,1) = TRACK_DATA_RAD.TrackParam(i).StateEstimate.px;
                    X_i(2,1) = TRACK_DATA_RAD.TrackParam(i).StateEstimate.vx;
                    X_i(3,1) = TRACK_DATA_RAD.TrackParam(i).StateEstimate.py;
                    X_i(4,1) = TRACK_DATA_RAD.TrackParam(i).StateEstimate.vy;
                    %P_i     = TRACK_DATA_RAD.TrackParam(i).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                    P_i_pos  = TRACK_DATA_RAD.TrackParam(i).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                    P_i_vel  = TRACK_DATA_RAD.TrackParam(i).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                    for jj = (ii+1):nUngatedTracksRAD
                        j = UnGatedRadTrackIdx(1,jj);
                        if(~isRadarTrackGrouped(j))
                            % Track State from Radar Track 'j'
                            X_j(1,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.px;
                            X_j(2,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.vx;
                            X_j(3,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.py;
                            X_j(4,1) = TRACK_DATA_RAD.TrackParam(j).StateEstimate.vy;
                            %P_j     = TRACK_DATA_RAD.TrackParam(j).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                            P_j_pos  = TRACK_DATA_RAD.TrackParam(j).StateEstimate.ErrCOV(posCovIdx,posCovIdx);
                            P_j_vel  = TRACK_DATA_RAD.TrackParam(j).StateEstimate.ErrCOV(velCovIdx,velCovIdx);
                            % compute the statistical distance between the Radar Track j and Camera Track i
                            %X = X_i - X_j; P = P_i + P_j;
                            Xpos = X_i([1,3],1) - X_j([1,3],1); Ppos = P_i_pos + P_j_pos;
                            Xvel = X_i([2,4],1) - X_j([2,4],1); Pvel = P_i_vel + P_j_vel;
                            %dist = X' * (P\X); % Statistical dist
                            distPos = Xpos' * (Ppos\Xpos);
                            distVel = Xvel' * (Pvel\Xvel);
                            if(distPos <= gammaPos && distVel <= gammaVel)
                               isRadarTrackGrouped(j) = true;
                               nRadTracks = nRadTracks + 1;
                               RadarTrackIDs(1, nRadTracks) = j; % Update the Radar Track ID here (Used later for grouping)
                            end
                        end
                    end
                    % Compute the Track Cluster estimates (This track has Radar only cluster)
                    Xfus(:) = single(0); Xrad(:) = single(0); 
                    Pfus(:) = single(0); Prad(:) = single(0); Pcam(:) = single(0); Pspread(:) = single(0);
                    NewTrack = false; ExistingTrack = false; GatedTrack = false; PredictedTrack = false; 
                    RadarCatch = false; CameraCatch = false; CameraSource(:) = false; RadarSource(:) = false;
                    nLocalTracks = nRadTracks;
                    weight = 1/nLocalTracks;
                    for idx = 1:nRadTracks  % weighted mean and covariance from radar tracks
                        index = RadarTrackIDs(1, idx);
                        X_i(1,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.px;
                        X_i(2,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vx;
                        X_i(3,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.py;
                        X_i(4,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vy;
                        P_i = TRACK_DATA_RAD.TrackParam(index).StateEstimate.ErrCOV(StateParamIndex,StateParamIndex);
                        Xfus = Xfus + weight.* X_i;
                        Pfus = Pfus + weight.* P_i;
                        Xrad = X_i; Prad = P_i;
                        RadarCatch     = (RadarCatch     || TRACK_DATA_RAD.TrackParam(index).SensorSource.RadarCatch);
                        RadarSource    = (RadarSource     | TRACK_DATA_RAD.TrackParam(index).SensorSource.RadarSource);
                        NewTrack       = (NewTrack       && TRACK_DATA_RAD.TrackParam(index).Status.New);
                        ExistingTrack  = (ExistingTrack  || TRACK_DATA_RAD.TrackParam(index).Status.Existing);
                        GatedTrack     = (GatedTrack     || TRACK_DATA_RAD.TrackParam(index).Status.Gated);
                        PredictedTrack = (PredictedTrack && TRACK_DATA_RAD.TrackParam(index).Status.Predicted);
                    end
                    NewTrack = ~ExistingTrack;
                    RadarCameraCatch = (RadarCatch && CameraCatch);
                    for idx = 1:nRadTracks % spread of the mean
                        index = RadarTrackIDs(1, idx);
                        X_i(1,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.px;
                        X_i(2,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vx;
                        X_i(3,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.py;
                        X_i(4,1) = TRACK_DATA_RAD.TrackParam(index).StateEstimate.vy;
                        Pspread = Pspread + weight.*( (Xfus - X_i)*(Xfus - X_i)' );
                    end
                    Pfus = Pfus + Pspread;
                    RadarTrackIDs(:) = uint16(0);  % reset to 0
                    % update the Merged Track in the output
                    nNewTracks = nNewTracks + 1;
                    FUSED_TRACKS(1, nNewTracks).Xfus = Xfus;   FUSED_TRACKS(1, nNewTracks).Xrad = Xrad;   FUSED_TRACKS(1, nNewTracks).Xcam = Xcam;
                    FUSED_TRACKS(1, nNewTracks).Pfus = Pfus;   FUSED_TRACKS(1, nNewTracks).Prad = Prad;   FUSED_TRACKS(1, nNewTracks).Pcam = Pcam;
                    FUSED_TRACKS(1, nNewTracks).CameraCatch      = CameraCatch;  
                    FUSED_TRACKS(1, nNewTracks).RadarCatch       = RadarCatch;
                    FUSED_TRACKS(1, nNewTracks).RadarCameraCatch = RadarCameraCatch;
                    FUSED_TRACKS(1, nNewTracks).CameraSource     = CameraSource;
                    FUSED_TRACKS(1, nNewTracks).RadarSource      = RadarSource;
                    FUSED_TRACKS(1, nNewTracks).New              = NewTrack;
                    FUSED_TRACKS(1, nNewTracks).Existing         = ExistingTrack;
                    FUSED_TRACKS(1, nNewTracks).Predicted        = PredictedTrack;
                    FUSED_TRACKS(1, nNewTracks).Gated            = GatedTrack;
                end
            end
        end
        % ======================================================================================================================================================
    end
end