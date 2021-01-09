classdef VISUALIZE
    methods(Static)
        % ======================================================================================================================================================
        % display measurements
        function displayMeas(LANE_LINE_MEAS_MAT, XLimit, YLimit)
            for idx = 1:LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(1,end)
                x0 = 0; y0 = LANE_LINE_MEAS_MAT.MeasArray(1,idx); a0 = LANE_LINE_MEAS_MAT.MeasArray(2,idx);
                K0 = LANE_LINE_MEAS_MAT.MeasArray(3,idx); Khat = LANE_LINE_MEAS_MAT.MeasArray(4,idx); L = LANE_LINE_MEAS_MAT.MeasArray(5,idx);
                nPts = 100; sMax = L; sMin = 0; ds = (sMax - sMin)/(nPts - 1); s = sMin:ds:sMax;
                [x, y, ~, ~] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0, y0, a0, K0, Khat, s, L);
                plot(x, y, '.', 'color', 'r');axis equal;grid on; hold on;
                %set(gca,'XLim',XLimit); set(gca,'YLim',YLimit)
            end
            %set(gca,'XLim',XLimit); set(gca,'YLim',YLimit);
        end
        % ======================================================================================================================================================
        function displayRoad(points, color, marker, XLimit, YLimit)
            validIdx = points.BufferLastValidIdx;
            x = points.X(1,1:validIdx); y = points.Y(1,1:validIdx); 
            plot(x, y, marker, 'color', color);axis equal; grid on; hold on;
            %set(gca,'XLim',XLimit); set(gca,'YLim',YLimit);
        end
        % ======================================================================================================================================================
        function displayLane(ClothoidCoeff, color, marker, XLimit, YLimit)
            x0 = 0; y0 = ClothoidCoeff.LateralOffset; 
            a0 = ClothoidCoeff.HeadingAngle; 
            K0 = ClothoidCoeff.Curvature; 
            Khat = ClothoidCoeff.CurvatureDerivative; 
            L = ClothoidCoeff.CurveLength;
            nPts = 1000; sMax = L; sMin = 0; ds = (sMax - sMin)/(nPts - 1); s = sMin:ds:sMax;
            [x, y, ~, ~] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0, y0, a0, K0, Khat, s, L);
            plot(x, y, marker, 'color', color);axis equal;grid on; hold on;
            %set(gca,'XLim',XLimit); set(gca,'YLim',YLimit);
        end
        % ======================================================================================================================================================
        function VisualizeTrackData_AND_Lane(FUSED_TRACKS, TRACK_ESTIMATES, TRACK_ESTIMATES_RAD, TRACK_ESTIMATES_CAM, ...
                                             RADAR_MEAS_CLUSTER, RADAR_MEAS_CTS, CAMERA_MEAS_CTS, ...
                                             nNewTracks, nRadars, nCameras, gamma, TRAJECTORY_HISTORY, ...
                                             LANE_LINE_MEAS_MAT, ROAD_PROFILE, ROAD_GEOMETRY, XLimit, YLimit)
                
                figure(1);                         
                nMeasCam = CAMERA_MEAS_CTS.ValidCumulativeMeasCount(1,nCameras);
                nMeasRad = RADAR_MEAS_CTS.ValidCumulativeMeasCount(1,nRadars);
                nClstrsRad = RADAR_MEAS_CLUSTER.ValidCumulativeMeasCount(1,nRadars);
                pxRAD = single(zeros(1,TRACK_ESTIMATES_RAD.nValidTracks));
                pyRAD = single(zeros(1,TRACK_ESTIMATES_RAD.nValidTracks));
                pxCAM = single(zeros(1,TRACK_ESTIMATES_CAM.nValidTracks));
                pyCAM = single(zeros(1,TRACK_ESTIMATES_CAM.nValidTracks));
                pxFUS = single(zeros(1,TRACK_ESTIMATES.nValidTracks));
                pyFUS = single(zeros(1,TRACK_ESTIMATES.nValidTracks));
                nPts = 100;
                level = sqrt(50);
                StateCovIndex = [1,2,4,5];
                
                % Fused New Tracks
                newTrack_PXPY = single(zeros(2, 100));
                for idx = 1:nNewTracks
                    newTrack_PXPY(1,idx) = FUSED_TRACKS(idx).Xfus(1,1);
                    newTrack_PXPY(2,idx) = FUSED_TRACKS(idx).Xfus(3,1);
                end
                
               % RADAR Tracks
                for idx = 1:TRACK_ESTIMATES_RAD.nValidTracks
                    pxRAD(1,idx) = TRACK_ESTIMATES_RAD.TrackParam(1,idx).StateEstimate.px;
                    pyRAD(1,idx) = TRACK_ESTIMATES_RAD.TrackParam(1,idx).StateEstimate.py;
                    P = TRACK_ESTIMATES_RAD.TrackParam(1,idx).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex);
                    %S = H*P*H' + R;
                    S = P([1,4],[1,4]);
                    EllipseXY = PLOTS.sigmaEllipse2D( [pxRAD(1,idx); pyRAD(1,idx)], S, level, nPts );
                    plot(EllipseXY(1,:), EllipseXY(2,:), '-', 'color', 'k', 'markersize', 5);
                    axis equal; grid on;hold on;
                end
                plot(pxRAD,  pyRAD, '*', 'color', 'r', 'markersize', 4);
                axis equal;  grid on; hold on;
                %plot(RADAR_MEAS_CLUSTER.MeasArray(1, 1:nClstrsRad), RADAR_MEAS_CLUSTER.MeasArray(2, 1:nClstrsRad), '.', 'color', 'r', 'markersize', 5);
                plot(RADAR_MEAS_CTS.MeasArray(1, 1:nMeasRad), RADAR_MEAS_CTS.MeasArray(2, 1:nMeasRad), '.', 'color', 'r', 'markersize', 5);
                axis equal;  grid on; hold on;
                
                % CAMERA Tracks
                for idx = 1:TRACK_ESTIMATES_CAM.nValidTracks
                    pxCAM(1,idx) = TRACK_ESTIMATES_CAM.TrackParam(1,idx).StateEstimate.px;
                    pyCAM(1,idx) = TRACK_ESTIMATES_CAM.TrackParam(1,idx).StateEstimate.py;
                    P = TRACK_ESTIMATES_CAM.TrackParam(1,idx).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex);
                    %S = H*P*H' + R;
                    S = P([1,4],[1,4]);
                    EllipseXY = PLOTS.sigmaEllipse2D( [pxCAM(1,idx); pyCAM(1,idx)], S, level, nPts );
                    plot(EllipseXY(1,:), EllipseXY(2,:), '-', 'color', 'k', 'markersize', 5);
                    axis equal; grid on;hold on;
                end
                plot(pxCAM,  pyCAM, '*', 'color', 'b', 'markersize', 4); 
                axis equal; grid on; hold on;
                plot(CAMERA_MEAS_CTS.MeasArray(1, 1:nMeasCam), CAMERA_MEAS_CTS.MeasArray(2, 1:nMeasCam), '.', 'color', 'b', 'markersize', 5);
                axis equal;  grid on; hold on;
                
                % FUSED Tracks
                for idx = 1:TRACK_ESTIMATES.nValidTracks
                    pxFUS(1,idx) = TRACK_ESTIMATES.TrackParam(1,idx).StateEstimate.px;
                    pyFUS(1,idx) = TRACK_ESTIMATES.TrackParam(1,idx).StateEstimate.py;
                    P = TRACK_ESTIMATES.TrackParam(1,idx).StateEstimate.ErrCOV(StateCovIndex,StateCovIndex);
                    S = P([1,4],[1,4]);
                    EllipseXY = PLOTS.sigmaEllipse2D( [pxFUS(1,idx); pyFUS(1,idx)], S, level, nPts );
                    plot(EllipseXY(1,:), EllipseXY(2,:), '-', 'color', 'm', 'markersize', 5);
                    axis equal; grid on;hold on;
                end
                plot(pxFUS,  pyFUS, '*', 'color', 'm', 'markersize', 4); 
                axis equal;  grid on; hold on;
                
                % Trajectory History
                nPtsTV1 = TRAJECTORY_HISTORY.TRACK_HISTORY(1).BufferStartIndex;
                nPtsTV2 = TRAJECTORY_HISTORY.TRACK_HISTORY(2).BufferStartIndex;
                PX_TV1 = TRAJECTORY_HISTORY.TRACK_HISTORY(1).HistoryBufferPx(1,1:nPtsTV1);
                PY_TV1 = TRAJECTORY_HISTORY.TRACK_HISTORY(1).HistoryBufferPy(1,1:nPtsTV1);
                PX_TV2 = TRAJECTORY_HISTORY.TRACK_HISTORY(2).HistoryBufferPx(1,1:nPtsTV2);
                PY_TV2 = TRAJECTORY_HISTORY.TRACK_HISTORY(2).HistoryBufferPy(1,1:nPtsTV2);
                plot(PX_TV1, PY_TV1,'.', 'color', 'b', 'markersize', 5); axis equal;grid on; hold on;
                plot(PX_TV2, PY_TV2,'-*', 'color', 'r', 'markersize', 5); axis equal;  grid on; hold on;
               
                
                % Lane and Road Line Measurements 
                %figure(1); XLimit = [-10 100]; YLimit = [-20 20]; markerA = '.';markerB = '-';
                %displayMeas(LANE_LINE_MEAS_MAT, XLimit, YLimit);hold on;
                %displayLane(ROAD_PROFILE.EGO_LANE_CENTER_TRACK_ESTIMATION, 'b', markerA, XLimit, YLimit);hold on;
                %displayLane(ROAD_PROFILE.EGO_LANE_NEXT_NEXT_RIGHT_BOUNDARY, 'k', markerB, XLimit, YLimit);hold on;
                %displayLane(ROAD_PROFILE.EGO_LANE_NEXT_RIGHT_BOUNDARY, 'm', markerB, XLimit, YLimit);hold on;
                %displayLane(ROAD_PROFILE.EGO_LANE_RIGHT_BOUNDARY, 'k', markerB, XLimit, YLimit);hold on;
                %displayLane(ROAD_PROFILE.EGO_LANE_LEFT_BOUNDARY, 'k', markerB, XLimit, YLimit);hold on;
                %displayLane(ROAD_PROFILE.EGO_LANE_NEXT_LEFT_BOUNDARY, 'm', markerB, XLimit, YLimit);hold on;
                %displayLane(ROAD_PROFILE.EGO_LANE_NEXT_NEXT_LEFT_BOUNDARY, 'k', markerB, XLimit, YLimit);hold on;  
                %hold off;
    
                markerA = '.'; markerB = '-'; %XLimit = [-250, 200]; YLimit = [-450, 50];                                                         
                VISUALIZE.displayRoad(ROAD_GEOMETRY.EgoLaneCenter, 'b', markerA, XLimit, YLimit);hold on;
                VISUALIZE.displayRoad(ROAD_GEOMETRY.EgoLaneRightBoundary, 'r', markerB, XLimit, YLimit);hold on;
                VISUALIZE.displayRoad(ROAD_GEOMETRY.EgoLaneLeftBoundary, 'g', markerB, XLimit, YLimit);hold on;
                VISUALIZE.displayRoad(ROAD_GEOMETRY.EgoLaneNxtRightBoundary, 'k', markerB, XLimit, YLimit);hold on;
                VISUALIZE.displayRoad(ROAD_GEOMETRY.EgoLaneNxtLeftBoundary, 'm', markerB, XLimit, YLimit);hold on;
                VISUALIZE.displayRoad(ROAD_GEOMETRY.EgoLaneNxtNxtRightBoundary, 'b', markerB, XLimit, YLimit);hold on;
                VISUALIZE.displayRoad(ROAD_GEOMETRY.EgoLaneNxtNxtLeftBoundary, 'r', markerB, XLimit, YLimit);hold on;
                
                
                VISUALIZE.displayMeas(LANE_LINE_MEAS_MAT, XLimit, YLimit);hold on;
                set(gca,'XLim',XLimit); set(gca,'YLim',YLimit);
                hold off;
                drawnow
        end
        % ======================================================================================================================================================
    end
end