% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Track Trajectory Computation function library , Contains the following functions :
%             : INIT_TRAJECTORY_HISTORY()           : Initialize Trajectory history buffer of the newly appeared tracks
%             : TRAJECTORY_HISTORY_COMPUTATION()    : Update/Expand the  Track trajectory buffer, ego compensate the trajectory history
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef TRAJECTORY
    methods(Static)
        % ======================================================================================================================================================
        function TRAJECTORY_HISTORY = INIT_TRAJECTORY_HISTORY(TRAJECTORY_HISTORY_in, TRACK_ESTIMATES_FUS)
            
            % initializations
            TRAJECTORY_HISTORY = TRAJECTORY_HISTORY_in;
            TrackIDExistFlag = false(1,100); TrackIDNewFlag = false(1,100); TrackIDLostFlag = false(1,100);
    
            % Identify Existing Tracks IDs and Update the 'TRAJECTORY_HISTORY' (Lost Tracks are not considered)
            count = 0;
            for i = 1:TRACK_ESTIMATES_FUS.nValidTracks
                if(TRACK_ESTIMATES_FUS.TrackParam(i).Status.Existing)
                   isFound = false;
                   for j = 1:TRAJECTORY_HISTORY_in.nConfirmedTracks
                       if(TRACK_ESTIMATES_FUS.TrackParam(i).id == TRAJECTORY_HISTORY_in.TRACK_HISTORY(j).id)
                          count = count + 1;
                          TrackIDExistFlag(1,i) = true;
                          TRAJECTORY_HISTORY.TRACK_HISTORY(count) = TRAJECTORY_HISTORY_in.TRACK_HISTORY(j);
                          TRAJECTORY_HISTORY.TRACK_HISTORY(count).TrackEstRefIdx = i;
                          isFound = true;
                          break;
                       end
                   end
                   if(~isFound)
                      TrackIDNewFlag(1,i) = true;
                   end
                end
            end
    
            % Identify and count the number of Lost tracks 
            for i = 1:TRAJECTORY_HISTORY_in.nConfirmedTracks
                isFound = false;
                for j = 1:TRACK_ESTIMATES_FUS.nValidTracks
                    if(TRACK_ESTIMATES_FUS.TrackParam(j).Status.Existing && (TRAJECTORY_HISTORY_in.TRACK_HISTORY(i).id == TRACK_ESTIMATES_FUS.TrackParam(j).id))
                       isFound = true;
                       break;
                    end
                end
                if(~isFound)
                   TrackIDLostFlag(1,i) = true;
                end
            end
            nLostTracks = length(find(TrackIDLostFlag(1,1:TRAJECTORY_HISTORY_in.nConfirmedTracks) == true));
            TRAJECTORY_HISTORY.nConfirmedTracks = TRAJECTORY_HISTORY.nConfirmedTracks - nLostTracks;
    
            % Set the new Tracks ID and other Tracjectory Parameters
            TrackIDNewIndex = find(TrackIDNewFlag(1,1:TRACK_ESTIMATES_FUS.nValidTracks) == true);
            nNewTracks = length(TrackIDNewIndex);
            for i = 1:nNewTracks
                index = i + TRAJECTORY_HISTORY.nConfirmedTracks;
                writeIdx =  TRAJECTORY_HISTORY.TRACK_HISTORY(index).WriteIndex;
                TRAJECTORY_HISTORY.TRACK_HISTORY(index).TrackEstRefIdx = TrackIDNewIndex(1,i);
                TRAJECTORY_HISTORY.TRACK_HISTORY(index).id  = TRACK_ESTIMATES_FUS.TrackParam(TrackIDNewIndex(1,i)).id;
                TRAJECTORY_HISTORY.TRACK_HISTORY(index).HistoryBufferPx(1,writeIdx) = TRACK_ESTIMATES_FUS.TrackParam(TrackIDNewIndex(1,i)).StateEstimate.px;
                TRAJECTORY_HISTORY.TRACK_HISTORY(index).HistoryBufferPy(1,writeIdx) = TRACK_ESTIMATES_FUS.TrackParam(TrackIDNewIndex(1,i)).StateEstimate.py;
                TRAJECTORY_HISTORY.TRACK_HISTORY(index).BufferStartIndex = TRAJECTORY_HISTORY.TRACK_HISTORY(index).BufferStartIndex + 1;
                TRAJECTORY_HISTORY.TRACK_HISTORY(index).WriteIndex = TRAJECTORY_HISTORY.TRACK_HISTORY(index).WriteIndex + 1;
            end
            TRAJECTORY_HISTORY.nConfirmedTracks = TRAJECTORY_HISTORY.nConfirmedTracks + nNewTracks;
        end
        % ======================================================================================================================================================
        function TRAJECTORY_HISTORY = TRAJECTORY_HISTORY_COMPUTATION(TRAJECTORY_HISTORY, TRACK_ESTIMATES_FUS, EGO_CAN_BUS, dT)
            DistTheshold = single(1.0);
            DEG2RAD = single(pi/180);
            dyaw  = (-1) * DEG2RAD * EGO_CAN_BUS.yawRate * dT;
            vEgo  = sqrt( (EGO_CAN_BUS.vx)^2 + (EGO_CAN_BUS.vy)^2 );
            vxEgo = vEgo*cos(dyaw);  
            vyEgo = vEgo*sin(dyaw);
            dXEgo = [vxEgo*dT ; vyEgo*dT];
            yawRateCompensationMat = [ cos(dyaw), -sin(dyaw); ...
                                       sin(dyaw),  cos(dyaw)];
            for i = 1:TRAJECTORY_HISTORY.nConfirmedTracks
            if(~TRAJECTORY_HISTORY.TRACK_HISTORY(i).isInitialized)    % if the trajectory is not initialized then initialize
                TRAJECTORY_HISTORY.TRACK_HISTORY(i).isInitialized = true;
            
            elseif(TRAJECTORY_HISTORY.TRACK_HISTORY(i).isInitialized) % if the trajectory is initialized 
                   % 1. perform ego compensation for all points in the history buffer
                   positionVec = [TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex); ...
                                  TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex)];
                   positionTransformed = yawRateCompensationMat * positionVec - dXEgo;
                   TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex) = positionTransformed(1, :);
                   TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex) = positionTransformed(2, :);
                   % 2. Compute the distance between Track data and the last point in the history buffer
                   objIdx  = TRAJECTORY_HISTORY.TRACK_HISTORY(i).TrackEstRefIdx;
                   idxPrev = TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex;
                   px_t_1 =  TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,idxPrev);
                   py_t_1 =  TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,idxPrev);
                   px_t   =  TRACK_ESTIMATES_FUS.TrackParam(objIdx).StateEstimate.px;
                   py_t   =  TRACK_ESTIMATES_FUS.TrackParam(objIdx).StateEstimate.py;
                   dL = sqrt((px_t - px_t_1)^2 + (py_t - py_t_1)^2);
                   TRAJECTORY_HISTORY.TRACK_HISTORY(i).SegmentLength = TRAJECTORY_HISTORY.TRACK_HISTORY(i).SegmentLength + dL;
                   TRAJECTORY_HISTORY.TRACK_HISTORY(i).Length        = TRAJECTORY_HISTORY.TRACK_HISTORY(i).Length + dL;
                   if(TRAJECTORY_HISTORY.TRACK_HISTORY(i).SegmentLength >= DistTheshold)
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).SegmentLength = TRAJECTORY_HISTORY.TRACK_HISTORY(i).SegmentLength - DistTheshold;
                      if(TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex == length(TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,:)))
                         TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex = length(TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,:));
                         TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex-1) = ...
                                            TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,2:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex);
                         TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex-1) = ...
                                            TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,2:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex);
                      end
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex) = px_t;
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex) = py_t; 
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex = TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex + 1;
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex = TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex - 1;
                   elseif(TRAJECTORY_HISTORY.TRACK_HISTORY(i).SegmentLength < DistTheshold)
                      if(TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex == length(TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,:)))
                         TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex = length(TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,:));
                         TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex-1) = ...
                                            TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,2:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex);
                         TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,1:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex-1) = ...
                                            TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,2:TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex);
                      end
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPx(1,TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex) = px_t;
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).HistoryBufferPy(1,TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex) = py_t; 
                      TRAJECTORY_HISTORY.TRACK_HISTORY(i).BufferStartIndex = TRAJECTORY_HISTORY.TRACK_HISTORY(i).WriteIndex;
                   end
            end
            end
            
        end
        % ======================================================================================================================================================
    end
end