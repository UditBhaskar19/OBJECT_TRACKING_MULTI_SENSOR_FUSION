% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Clustering functions for radar and camera measurements, contains the following functions :
%             : SEGREGATE_CLUSTER()                 : Segregated Clusters, from the combined sensor measurement array and sensor measurement clusters 
%             : CLUSTER_RADAR_MEASUREMENTS_DBSCAN() : clusters radar measurements using DBSCAN algorithm
%             : CLUSTER_CAMERA_MEASUREMENTS_NN()    : clusters camera measurements using Nearest Neighbour algorithm
%             : MERGE_RADAR_CAMERA_CLUSTER          : Merge Radar and Camera Clusters
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef CLUSTERING
    methods(Static)
        % ======================================================================================================================================================
        function [CLUSTER_DATA] = SEGREGATE_CLUSTER(SENSORMeasCTS, SENSOR_CLUSTERS)
            % Segregated Clusters, from the combined sensor measurement array and sensor measurement clusters (Output of DBSCAN/NN) , 
            % the cluster centres are recomputed with respect to each of the sensors
            % segregate cluster members (sensor measurements) w.r.t each of the sensor source and recompute the cluster statistics
            % INPUTS : SENSORMeasCTS   : Measurements from Sensors
            %          SENSOR_CLUSTERS : Cluster centers and covariance (Output of DBSCAN/NN)
            % OUTPUT : CLUSTER_DATA    : Segregated Clusters
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nSensors = length(SENSORMeasCTS.ValidCumulativeMeasCount); % total number of sensors installed around the ego vehicle
            nMeas = SENSORMeasCTS.ValidCumulativeMeasCount(end);       % total number of measurements received  
            nSnsrClsters = SENSOR_CLUSTERS.nClusters;                  % total number of measurement clusters (all radars combined)
            countTotal = int16(0);
            
            % Initializations for the output
            CLUSTER_DATA = struct;
            CLUSTER_DATA.MeasArray = single(zeros(2 , uint16(nSensors)*200));
            CLUSTER_DATA.MeasCovariance = single(zeros(2, 2, uint16(nSensors)*200));
            CLUSTER_DATA.SensorSource = uint16(zeros(1 , uint16(nSensors)*200));
            CLUSTER_DATA.ClusterRef = uint16(zeros(1 , uint16(nSensors)*200));
            CLUSTER_DATA.ValidMeasCount = uint16(zeros(1,nSensors));
            CLUSTER_DATA.ValidCumulativeMeasCount = uint16(zeros(1,nSensors));
            
            for snsrIdx = 1:nSensors            % iterate over each of the sensors
                for idxClstr = 1:nSnsrClsters   % for each of the Radar clusters
                    % 1. extract the associated cluster IDs
                    clusterID = SENSOR_CLUSTERS.ClusterIDs(1,idxClstr); 
                    % 2. extract the measurement IDs forming the cluster
                    measIDsIndex = find(clusterID == SENSOR_CLUSTERS.ClustIDAssig(1,1:nMeas)); 
                    % 3. extract the associated measurement IDs for each of the sensors
                    measIDs = measIDsIndex(snsrIdx == SENSORMeasCTS.MeasRef(2,measIDsIndex)); 
                    if(~isempty(measIDs))
                        countTotal = countTotal + 1;
                        weight = 1/length(measIDs);
                        x = 0; R = 0;xSpread = 0;
                        for idxMeas = 1:length(measIDs)   % for each of the extracted measurements compute the weighted mean and covariance
                            x = x + weight.*SENSORMeasCTS.MeasArray(:,measIDs(idxMeas));
                            R = R + weight.*SENSORMeasCTS.MeasCovariance(:,:,measIDs(idxMeas));
                        end
                        for idxMeas = 1:length(measIDs)   % compute the spread of the mean
                            z = SENSORMeasCTS.MeasArray(:,measIDs(idxMeas));
                            xSpread = xSpread + weight.*( (x - z)*(x - z)' );
                        end
                        %R = R + xSpread;
                        CLUSTER_DATA.MeasArray(:,countTotal) = x;
                        CLUSTER_DATA.MeasCovariance(:,:,countTotal) = R;
                        CLUSTER_DATA.SensorSource(1,countTotal) = snsrIdx;
                        CLUSTER_DATA.ClusterRef(1,countTotal) = clusterID;
                        CLUSTER_DATA.ValidMeasCount(1,snsrIdx) = CLUSTER_DATA.ValidMeasCount(1,snsrIdx) + 1;
                    end
                end
            end
            CLUSTER_DATA.ValidCumulativeMeasCount = cumsum(CLUSTER_DATA.ValidMeasCount);
        end
        % ======================================================================================================================================================
        function [RADAR_CLUSTERS]  = CLUSTER_RADAR_MEASUREMENTS_DBSCAN(RADARMeasCTS, MANAGE_CLUSTER , RADAR_CLUSTERS, epsPos)
            % clusters radar measurements using DBSCAN algorithm
            % INPUT:  RADARMeasCTS : array of radar measurements transformed from sensor frame to ego vehicle frame
            %         MANAGE_CLUSTERS   : A structure of arrays for maintaining information for clustering, contains the following fields
            %         measTypeCore      : 
            %         clusterMember     :
            %         clusterMemberFlag :
            %         measurementVisited:
            %         RADAR_CLUSTERS    : initialized structure of radar clusters and related information
            %         epsPos, epsVel    : clustering eps threshold for position and velocity
            % OUTPUT: RADAR_CLUSTERS    : structure of radar clusters and related information with the following fields :
            %         nClusters         : number of clusters formed
            %         ClusterSizes      : number of measurements in each of the clusters
            %         ClusterCenters    : cluster mean(s)
            %         ClusterCovariance : covariance(s) of the cluster mean(s)
            %         ClusterIDs        : cluster ID(s)
            %         ClustIDAssig      : measurement to cluster ID association
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            % Reset everything to zero
            RADAR_CLUSTERS.nClusters = uint16(0);
            RADAR_CLUSTERS.ClusterCenters(:) = single(0);
            RADAR_CLUSTERS.ClusterCovariance(:) = single(0);
            RADAR_CLUSTERS.ClusterSizes(:) = uint16(0);
            RADAR_CLUSTERS.ClusterIDs(:) = uint16(0);
            RADAR_CLUSTERS.ClustIDAssig(:) = uint16(0);
            sizeClust = uint16(0); ClusterID = uint16(0);
            pxIdx = 1; pyIdx = 2;  % Constant used as arrayIndex
            nMeas = RADARMeasCTS.ValidCumulativeMeasCount(end); % number of valid measurements
            % Identify Core Point;
            for i = 1:nMeas
                measCnt = 0;
                for j = 1:nMeas
                    Positiondiff = norm([RADARMeasCTS.MeasArray(pxIdx,i) - RADARMeasCTS.MeasArray(pxIdx,j), ...
                                         RADARMeasCTS.MeasArray(pyIdx,i) - RADARMeasCTS.MeasArray(pyIdx,j)]);
                    if(Positiondiff <= epsPos)
                       measCnt = measCnt + 1;
                    end
                end
                if(measCnt == 1)
                   MANAGE_CLUSTER.measTypeCore(i) = false;  % isolated point
                elseif(measCnt > 1)
                   MANAGE_CLUSTER.measTypeCore(i) = true;   % point that can be expanded
                end
            end
            
            for i = 1:nMeas
                % Single measurement Cluster Identification
                if (~MANAGE_CLUSTER.measurementVisited(i) && ~MANAGE_CLUSTER.measTypeCore(i) )  % if the measurement is not visited and it cannot be expanded then a single meas is a cluster
                     MANAGE_CLUSTER.measurementVisited(i) = true;                               % mark the meas as visited so that we may not have to visit again
                     RADAR_CLUSTERS.nClusters = RADAR_CLUSTERS.nClusters + 1;                   % increment the number of clusters
                     RADAR_CLUSTERS.ClusterCenters(:,RADAR_CLUSTERS.nClusters) = [RADARMeasCTS.MeasArray(pxIdx,i); ...
                                                                                  RADARMeasCTS.MeasArray(pyIdx,i)]; % cluster center
                     RADAR_CLUSTERS.ClusterCovariance(:, :, RADAR_CLUSTERS.nClusters) = RADARMeasCTS.MeasCovariance(:, :, i);
                     RADAR_CLUSTERS.ClusterSizes(RADAR_CLUSTERS.nClusters) = uint16(1);              % cluster size
                     ClusterID = ClusterID + 1;                        % increment the cluster ID
                     RADAR_CLUSTERS.ClusterIDs(RADAR_CLUSTERS.nClusters) = ClusterID; % cluster ID
                     RADAR_CLUSTERS.ClustIDAssig = ClusterID;                         % meas to cluster association
                     % Initial Expansion of the core members
                elseif(~MANAGE_CLUSTER.measurementVisited(i) && MANAGE_CLUSTER.measTypeCore(i) ) % if the meas is not visited and it can be expanded , perform expansion
                        MANAGE_CLUSTER.measurementVisited(i) = true; % mark the meas 'i' as visited so that we may not have to visit again
                        MANAGE_CLUSTER.clusterMemberFlag(i) = true;  % mark the meas 'i' as part of a cluster
                        sizeClust = sizeClust + 1;
                        MANAGE_CLUSTER.clusterMember(sizeClust) = i; % add the 'i' meas Id to the cluster list
                        for j = 1:nMeas % find the new points that can be added around 'i' only if 'j' has not beed added previously and 'j' is a core point
                            if(i ~= j && ~MANAGE_CLUSTER.clusterMemberFlag(j) && MANAGE_CLUSTER.measTypeCore(j))
                               Positiondiff = norm([RADARMeasCTS.MeasArray(pxIdx,i) - RADARMeasCTS.MeasArray(pxIdx,j), ...
                                                    RADARMeasCTS.MeasArray(pyIdx,i) - RADARMeasCTS.MeasArray(pyIdx,j)]);
                               if( Positiondiff <= epsPos)
				                   sizeClust = sizeClust + 1;
			                       MANAGE_CLUSTER.clusterMemberFlag(j) = true; % mark the meas 'j' as part of a cluster
			                       MANAGE_CLUSTER.clusterMember(sizeClust) = j; % add the 'j' meas Id to the cluster list
                               end
                            end
                        end
                        ClusterID = ClusterID + 1;                               % next available cluster ID 
                        RADAR_CLUSTERS.nClusters = RADAR_CLUSTERS.nClusters + 1; % increase the cluster count
                        RADAR_CLUSTERS.ClustIDAssig(i) = ClusterID;              % meas 'i' to cluster association 
                        index = 1;
                        % Expand Cluster
                        while(index <= sizeClust)
                              newCoreIdx = MANAGE_CLUSTER.clusterMember(index); 
                              if(~MANAGE_CLUSTER.measurementVisited(newCoreIdx)) % check if the new core point has not been expanded
                                  MANAGE_CLUSTER.measurementVisited(newCoreIdx) = true; % if the new core point has not been expanded then mark it as expanded and perform the expansion
                                  for j = 1:nMeas
                                      if( newCoreIdx ~= j && ~MANAGE_CLUSTER.clusterMemberFlag(j) && MANAGE_CLUSTER.measTypeCore(j))
                                          Positiondiff = norm([RADARMeasCTS.MeasArray(pxIdx,newCoreIdx) - RADARMeasCTS.MeasArray(pxIdx,j), ...
                                                               RADARMeasCTS.MeasArray(pyIdx,newCoreIdx) - RADARMeasCTS.MeasArray(pyIdx,j)]);
                                          if(Positiondiff <= epsPos)
                                             sizeClust = sizeClust + 1;
					                         MANAGE_CLUSTER.clusterMemberFlag(j) = true;
						                     MANAGE_CLUSTER.clusterMember(sizeClust) = j; 
                                          end
                                      end
                                  end
                              end
                              RADAR_CLUSTERS.ClustIDAssig(newCoreIdx) = ClusterID; % meas 'newCoreIdx' to cluster association
                              index = index + 1; % go to the next core point added to the cluster list
                         end  % end of while
                         RADAR_CLUSTERS.ClusterIDs(RADAR_CLUSTERS.nClusters) = ClusterID; % cluster ID
                         RADAR_CLUSTERS.ClusterSizes(RADAR_CLUSTERS.nClusters) = sizeClust; % cluster size
                         % Compute the mean of the cluster center
                         for n = 1:sizeClust
                             RADAR_CLUSTERS.ClusterCenters(:, RADAR_CLUSTERS.nClusters) = RADAR_CLUSTERS.ClusterCenters(:, RADAR_CLUSTERS.nClusters) + ...
                                                          [RADARMeasCTS.MeasArray(pxIdx ,MANAGE_CLUSTER.clusterMember(n)); ...
                                                           RADARMeasCTS.MeasArray(pyIdx ,MANAGE_CLUSTER.clusterMember(n))]; % cluster center;
                             RADAR_CLUSTERS.ClusterCovariance(:, :, RADAR_CLUSTERS.nClusters) = RADAR_CLUSTERS.ClusterCovariance(:, :, RADAR_CLUSTERS.nClusters) + ...
                                                           RADARMeasCTS.MeasCovariance(:, :, MANAGE_CLUSTER.clusterMember(n));
                         end
                         RADAR_CLUSTERS.ClusterCenters(:, RADAR_CLUSTERS.nClusters) = RADAR_CLUSTERS.ClusterCenters(:, RADAR_CLUSTERS.nClusters)/single(sizeClust);
                         RADAR_CLUSTERS.ClusterCovariance(:, :, RADAR_CLUSTERS.nClusters) = RADAR_CLUSTERS.ClusterCovariance(:, :, RADAR_CLUSTERS.nClusters)/single(sizeClust);
                         % reset all to 0 (one Cluster has formed)
		                 MANAGE_CLUSTER.clusterMember(:) = uint16(0);
	                     MANAGE_CLUSTER.clusterMemberFlag(:) = uint16(0);
		                 %MANAGE_CLUSTER.sizeClust = uint16(0);
                         sizeClust = uint16(0);
                 end % end of outer if condition
             end % end of outer for loop    
        end
        % ======================================================================================================================================================
        function [CAMERA_CLUSTERS] = CLUSTER_CAMERA_MEASUREMENTS_NN(CAMERAMeasCTS, MANAGE_CLUSTER, CAMERA_CLUSTERS, epsPos)
            % clusters camera measurements using Nearest Neighbour algorithm
            % INPUT:  CAMERAMeasCTS : array of camera measurements transformed from sensor frame to ego vehicle frame
            %         MANAGE_CLUSTERS    : A structure of arrays for maintaining information for clustering, contains the following fields
            %         clusterMember      :
            %         clusterMemberFlag  :
            %         measurementVisited :
            %         CAMERA_CLUSTERS    : initialized structure of camera clusters and related information
            %         epsPos, epsVel     : clustering eps threshold for position and velocity
            % OUTPUT: CAMERA_CLUSTERS    : structure of radar clusters and related information with the following fields :
            %         nClusters          : number of clusters formed
            %         ClusterSizes      : number of measurements in each of the clusters
            %         ClusterCenters    : cluster mean(s)
            %         ClusterCovariance : covariance(s) of the cluster mean(s)
            %         ClusterIDs        : cluster ID(s)
            %         ClustIDAssig      : measurement to cluster ID association
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            CAMERA_CLUSTERS.nClusters = uint16(0);
            CAMERA_CLUSTERS.ClusterSizes(:) = uint16(0);
            CAMERA_CLUSTERS.ClusterCenters(:) = single(0);
            CAMERA_CLUSTERS.ClusterCovariance(:) = single(0);
            CAMERA_CLUSTERS.ClusterIDs(:) = uint16(0);
            CAMERA_CLUSTERS.ClustIDAssig(:) = uint16(0);
            sizeClust = uint16(0); ClusterID = uint16(0);
            pxIdx = 1; pyIdx = 2;  % Constant used as arrayIndex
            nMeas = CAMERAMeasCTS.ValidCumulativeMeasCount(end); % number of valid measurements
            for i = 1:nMeas 
                if(~MANAGE_CLUSTER.measurementVisited(i))        % check if the measurement has been added to an existing cluster/expanded
                    MANAGE_CLUSTER.measurementVisited(i) = true; % form a new cluster and 'tag' it as added
                    ClusterID = ClusterID + 1;    % create a new cluster ID
                    sizeClust = sizeClust + 1;    % update the current cluster size
                    MANAGE_CLUSTER.clusterMember(sizeClust) = i; % add the measurement 'i' to the list (used for computing the cluster center later)
                    CAMERA_CLUSTERS.nClusters = CAMERA_CLUSTERS.nClusters + 1;  % update the total number of clusters
                    CAMERA_CLUSTERS.ClusterIDs(1,CAMERA_CLUSTERS.nClusters) = ClusterID; % set the cluster ID
                    CAMERA_CLUSTERS.ClustIDAssig(i) = ClusterID; % tag the measurement 'i' with its cluster ID
                    for j = i+1:nMeas % compare meas 'i' with other unclustered measurement 'j's to identify if 'j' forms a cluster with 'i'
                        if(~MANAGE_CLUSTER.measurementVisited(j)) % compare only if 'j' is not a part of existing cluster
                            Positiondiff = norm([CAMERAMeasCTS.MeasArray(pxIdx,i) - CAMERAMeasCTS.MeasArray(pxIdx,j), ...
                                                 CAMERAMeasCTS.MeasArray(pyIdx,i) - CAMERAMeasCTS.MeasArray(pyIdx,j)]);
                            if(Positiondiff <= epsPos)
                               MANAGE_CLUSTER.measurementVisited(j) = true; % tag the measurement 'j' indicating that it is a part of a cluster
                               CAMERA_CLUSTERS.ClustIDAssig(j) = ClusterID; % tag the measurement 'j' with its cluster ID
                               sizeClust = sizeClust + 1;    % update the current cluster size
                               MANAGE_CLUSTER.clusterMember(sizeClust) = j; % add the measurement 'j' to the list (used for computing the cluster center later)
                            end %end of check   
                        end
                    end
                    for n = 1:sizeClust
                        CAMERA_CLUSTERS.ClusterCenters(:, CAMERA_CLUSTERS.nClusters) = CAMERA_CLUSTERS.ClusterCenters(:, CAMERA_CLUSTERS.nClusters) + ...
                                            [CAMERAMeasCTS.MeasArray(pxIdx ,MANAGE_CLUSTER.clusterMember(n)); ...
                                             CAMERAMeasCTS.MeasArray(pyIdx ,MANAGE_CLUSTER.clusterMember(n))]; % cluster center;
                        CAMERA_CLUSTERS.ClusterCovariance(:, :, CAMERA_CLUSTERS.nClusters) = CAMERA_CLUSTERS.ClusterCovariance(:, :, CAMERA_CLUSTERS.nClusters) + ...
                                             CAMERAMeasCTS.MeasCovariance(:, :, MANAGE_CLUSTER.clusterMember(n));
                    end
                    CAMERA_CLUSTERS.ClusterCenters(:, CAMERA_CLUSTERS.nClusters) = CAMERA_CLUSTERS.ClusterCenters(:, CAMERA_CLUSTERS.nClusters)/single(sizeClust); %Cluster center
                    CAMERA_CLUSTERS.ClusterCovariance(:, :, CAMERA_CLUSTERS.nClusters) = CAMERA_CLUSTERS.ClusterCovariance(:, :, CAMERA_CLUSTERS.nClusters)/single(sizeClust);
                    CAMERA_CLUSTERS.ClusterSizes(1, CAMERA_CLUSTERS.nClusters) = sizeClust; % cluster size
                end
                MANAGE_CLUSTER.clusterMember(:) = uint16(0); % reset the Cluster member list to zero
                sizeClust = uint16(0); % reset the size cluster list tozero
            end
        end
        % ======================================================================================================================================================
        function [MERGED_CLUSTERS] = MERGE_RADAR_CAMERA_CLUSTER(RADAR_CLUSTERS, CAMERA_CLUSTERS, MANAGE_CLUSTER, MERGED_CLUSTERS, epsPos)
            % Merge Radar and Camera Clusters
            % INPUT:  RADAR_CLUSTERS  : array of camera measurements transformed from sensor frame to ego vehicle frame
            %         CAMERA_CLUSTERS :
            %         MANAGE_CLUSTERS : A structure of arrays for maintaining information for clustering, contains the following fields
            %         MERGED_CLUSTERS :
            %         epsPos, epsVel  : clustering eps threshold for position and velocity
            % OUTPUT: MERGED_CLUSTERS    : structure of radar clusters and related information with the following fields :
            %         nClusters          : number of clusters formed
            %         ClusterSizes       : number of measurements in each of the clusters
            %         ClusterCenters     : cluster mean(s)
            %         ClusterCovariance  : covariance(s) of the cluster mean(s)
            %         ClusterIDs         : cluster ID(s)
            %         LookUpToRadClstrID : 
            %         LookUpToCamClstrID :
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MERGED_CLUSTERS.nClusters = uint16(0);
            MERGED_CLUSTERS.ClusterCenters(:) = single(0);
            MERGED_CLUSTERS.ClusterCovariance(:) = single(0);
            MERGED_CLUSTERS.ClusterIDs(:) = uint16(0);
            MERGED_CLUSTERS.LookUpToRadClstrID(:) = uint16(0);
            MERGED_CLUSTERS.LookUpToCamClstrID(:) = uint16(0);
            sizeClustCam = uint16(0); sizeClustRad = uint16(0); ClusterID = uint16(0);
            pxIdx = 1; pyIdx = 2; % Constant used as arrayIndex
            % Begin Clustering (Camera Only and Camera - Radar heterogeneous)
            for i = 1:CAMERA_CLUSTERS.nClusters
                ClusterID = ClusterID + 1;
                MERGED_CLUSTERS.nClusters = MERGED_CLUSTERS.nClusters + 1;
                sizeClustCam = sizeClustCam + 1; %reset later
                for j = 1:RADAR_CLUSTERS.nClusters
                    if(~MANAGE_CLUSTER.RadarClstAdded(j))
                        posNorm = norm( RADAR_CLUSTERS.ClusterCenters(pxIdx,j) - CAMERA_CLUSTERS.ClusterCenters(pxIdx,i) , ...
                                        RADAR_CLUSTERS.ClusterCenters(pyIdx,j) - CAMERA_CLUSTERS.ClusterCenters(pyIdx,i));
                        if(posNorm <= epsPos)
                           MANAGE_CLUSTER.RadarClstAdded(j) = true;
                           sizeClustRad = sizeClustRad + 1; %reset later
                           MANAGE_CLUSTER.RadarClstrMemberList(sizeClustRad) = j; %reset later
                           MERGED_CLUSTERS.LookUpToRadClstrID(1,j) = ClusterID;
                        end
                    end
                end
                MERGED_CLUSTERS.LookUpToCamClstrID(1,i) = ClusterID;
                MERGED_CLUSTERS.ClusterIDs(1, MERGED_CLUSTERS.nClusters) = ClusterID;
                MERGED_CLUSTERS.ClusterCenters(:,ClusterID) = MERGED_CLUSTERS.ClusterCenters(:, ClusterID) + CAMERA_CLUSTERS.ClusterCenters(:, i);
                MERGED_CLUSTERS.ClusterCovariance(:, :, ClusterID) = MERGED_CLUSTERS.ClusterCovariance(:, :, ClusterID) + CAMERA_CLUSTERS.ClusterCovariance(:, :, i);
                for idx = 1:sizeClustRad
                    MERGED_CLUSTERS.ClusterCenters(:,ClusterID) = MERGED_CLUSTERS.ClusterCenters(:, ClusterID) + RADAR_CLUSTERS.ClusterCenters(:, MANAGE_CLUSTER.RadarClstrMemberList(idx));
                    MERGED_CLUSTERS.ClusterCovariance(:, :, ClusterID) = MERGED_CLUSTERS.ClusterCovariance(:, :, ClusterID) + RADAR_CLUSTERS.ClusterCovariance(:, :, MANAGE_CLUSTER.RadarClstrMemberList(idx));
                end
                sizeCluster = sizeClustRad + sizeClustCam;
                MERGED_CLUSTERS.ClusterCenters(:,ClusterID) = MERGED_CLUSTERS.ClusterCenters(:,ClusterID)/single(sizeCluster);
                MERGED_CLUSTERS.ClusterCovariance(:, :, ClusterID) = MERGED_CLUSTERS.ClusterCovariance(:, :, ClusterID)/single(sizeCluster);
                sizeClustCam = uint16(0);
                sizeClustRad = uint16(0);
                MANAGE_CLUSTER.RadarClstrMemberList(:) = uint16(0);
            end
            % Identify Radar only Clusters
            for j = 1:RADAR_CLUSTERS.nClusters
                if(~MANAGE_CLUSTER.RadarClstAdded(j))
                    ClusterID = ClusterID + 1;
                    MERGED_CLUSTERS.nClusters = MERGED_CLUSTERS.nClusters + 1;
                    MERGED_CLUSTERS.ClusterIDs(1, MERGED_CLUSTERS.nClusters) = ClusterID;
                    MERGED_CLUSTERS.LookUpToRadClstrID(1,j) = ClusterID;
                    MERGED_CLUSTERS.ClusterCenters(:,ClusterID) = RADAR_CLUSTERS.ClusterCenters(:, j);
                    MERGED_CLUSTERS.ClusterCovariance(:, :, ClusterID) = RADAR_CLUSTERS.ClusterCovariance(:, :, j);
                end
            end
        end
        % ======================================================================================================================================================
    end
end