% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Lane Line Estimation function library , Contains the following functions :
%             : LeastSquaresFit3rdDegreePolynomial() : polynomial regression function by least squares estimation 
%             : Polynomial2Clothoid_v2()             : polynomial coefficients to clothoid parameters
%             : CoordinatesClothidToCartesian()      : clothoid to cartesian coordinates by tayor approximation
%             : ConstantCurvRateModel()              : Line Track state prediction by constant curvature motion model
%             : ClothoidCoeffMeasModel()             : set measurement model for line measurements
%             : update()                             : kalman filter state update
%             : LANE_LINE_STRUCTURE()                : function to convert line measuremnet array to line array of strucures
%             : ComputeLaneWidth()                   : compute lane width
%             : LANE_LINE_MODEL()                    : function to compute road profile/road geometry and ego motion compensated road geometry history
%             : LANE_GEOMETRY()                      : function to compute adjacent lane line geometry from ego lane center line geometry
%             : EGO_LANE_HYPOTHESIS()                : compute ego lane center line clothoid parameters from camera line measurements
%             : GenerateLaneHypothesis()             : compute adjacent line clothoid parameters from ego lane center line
%             : GenerateEgoLaneHypothesis()          : compute ego lane center line clothoid parameters from ego lane boundary measurements
%             : EGO_LANE_PREDICTION()                : line prediction
%             : EGO_LANE_UPDATE()                    : line kalman filter update
%             : EGO_LANE_TRACK_TO_TRACK_FUSION_v2()  : Fusion of multiple ego lane center line hypotheisied tracks
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef LANE_ESTIMATION
    methods(Static)
        % ======================================================================================================================================================
        function PolyCoeff = LeastSquaresFit3rdDegreePolynomial(Xin, Yin)
            Y = Yin(:);  % create a column vector 
            X = zeros(length(Xin), 4);
            for i = 1:length(Xin)
                X(i,1) = 1; X(i,2) = Xin(i)^1; X(i,3) = Xin(i)^2; X(i,4) = Xin(i)^3;
            end
            PolyCoeff = (pinv(X'*X))*(X'*Y); %output coefficients
        end
        % ======================================================================================================================================================
        function [x0, y0, phi0, K0, K_hat, L] = Polynomial2Clothoid_v2(a0, a1, a2, a3, xInit, xEnd, nPts)
            % Compute the clothoid coefficients from polynomial coefficients
            % INPUTS : a0, a1, a2, a3 : coefficients of a 3rd order polynomial
            %          Xmax : upper limit of the polynomial
            %             L : Length of the polynomial
            % OUTPUTS :  X0 : 
            %            Y0 : 
            %           phi : 
            %             K :
            %         K_hat : 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            dx = (xEnd - xInit)/(nPts - 1); 
            X = xInit:dx:xEnd;
            Y = a0 + a1.*X + a2.*X.^2 + a3.*X.^3;
            dS = zeros(1,length(Y)); dS(1,1) = 0;
            dS(1,2:end) = sqrt((X(1,2:end) - X(1,1:end-1)).^2 + (Y(1,2:end) - Y(1,1:end-1)).^2);
            S = cumsum(dS);
            y_hat = a1 + 2*a2*X + 3*a3*X.^2;           
            y_hat_hat = 2*a2 + 6*a3*X;
            phi = atan(y_hat);                               
            K = ( y_hat_hat )./( (1 + y_hat.^2).^(3/2) );    
            Khats = (K(1,2:end) - K(1,1))./S(1,2:end);
            wts = dS(1,2:end)/S(1,end);
            x0 = xInit; 
            y0 = Y(1,1); 
            phi0 = phi(1,1);
            K0 = K(1,1);
            K_hat = wts * Khats';
            L = S(1,end);
        end
        % ======================================================================================================================================================
        function [Xs, Ys, phi_s, Ks] = CoordinatesClothidToCartesian(x0, y0, phi0, K0, K_hat, s, L)
            Ks = K0 + K_hat.*s;
            phi_s = phi0 + K0.*s + 0.5*K_hat.*s.^2;
    
            L1 = L/2; L2 = L1^2; L3 = L1^3;
            S = sin(phi0 + K0*L1 + 0.5*K_hat*L2);
            C = cos(phi0 + K0*L1 + 0.5*K_hat*L2);
            K = K0 + K_hat*L1;
     
            X1 = S*K;
            X2 = 0.5*(C*K^2 + K_hat*S);
            X3 = C*K^3*0.1667 - K_hat*C*K*0.5;
            Xs = x0 + (C + L1*X1 - L2*X2 -     L3*X3).*s ...
                    + (  -0.5*X1 + L1*X2 + 1.5*L2*X3).*s.^2 ...
                    + (       -0.3333*X2 -     L1*X3).*s.^3 ...
                    + (                      0.25*X3).*s.^4;
     
            Y1 = C*K;
            Y2 = 0.5*(S*K^2 - K_hat*C);
            Y3 = C*K^3*0.1667 + K_hat*S*K*0.5;
            Ys = y0 + (S - L1*Y1 - L2*Y2 +     L3*Y3).*s ...
                    + (   0.5*Y1 + L1*Y2 - 1.5*L2*Y3).*s.^2 ...
                    + (       -0.3333*Y2 +     L1*Y3).*s.^3 ...
                    + (                  -   0.25*Y3).*s.^4;
        end
        % ======================================================================================================================================================
        function [StatePred , ProcessModel] = ConstantCurvRateModel(state, L, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dS, dYaw)
            %performs linear/nonlinear(Extended) Kalman prediction step 
            %INPUT: state: a structure with two fields:
            %           x: object state mean (dim x 1 vector) 
            %           P: object state covariance ( dim x dim matrix ) 
            %       motionmodel: a structure specifies the motion model parameters 
            %          dS: distance travelled by the ego vehicle along the arc
            %          dYaw : change in ego vehicle yaw
            %OUTPUT:state_pred: a structure with two fields:
            %           x: predicted object state mean (dim x 1 vector)
            %           P: predicted object state covariance ( dim x dim matrix )
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            ProcessModel.dim = 4; % state dimension
            ProcessModel.F = single([1, dS, 0.5*(dS^2), 0.1667*(dS^3); ...
                                     0,  1,         dS,    0.5*(dS^2); ...
                                     0,  0,          1,            dS; ...
                                     0,  0,          0,             1]);
            ProcessModel.Q = single([sigmaSq_y0, 0, 0, 0; ...
                                     0, sigmaSq_a0, 0, 0; ...
                                     0, 0, sigmaSq_K0, 0; ...
                                     0, 0, 0, sigmaSq_Khat]);
            [~, yOffset, ~, ~] =  LANE_ESTIMATION.CoordinatesClothidToCartesian(0, state.x(1,1), state.x(2,1), state.x(3,1), state.x(4,1), dS, L);                     
            ProcessModel.f(1,1) = yOffset;
            ProcessModel.f(2,1) = state.x(2,1) + state.x(3,1)*dS + 0.5*state.x(4,1)*dS*dS;
            ProcessModel.f(3,1) = state.x(3,1) + state.x(4,1)*dS;
            ProcessModel.f(4,1) = state.x(4,1);
            StatePred.x = ProcessModel.f;
            StatePred.P = ProcessModel.F * state.P * ProcessModel.F' + ProcessModel.Q;
            StatePred.x(1,1) = StatePred.x(1,1) + dYaw;
        end
        % ======================================================================================================================================================
        function MeasurementModel = ClothoidCoeffMeasModel(sigmaSq_y0, sigmaSq_A0, sigmaSq_K0, sigmaSq_Khat)
            % creates the measurement model for a 2D nearly constant acceleration motion model 
            % INPUT: sigmaPx, sigmaPy, sigmaVx, sigmaVy: standard deviation of measurement noise 
            % OUTPUT: obj.d: measurement dimension 
            %         obj.H: function handle return an observation matrix (4 x 6 matrix) 
            %         obj.R: measurement noise covariance (4 x 4 matrix)
            %         obj.h: function handle return a measurement (4 x 1 vector)
            % Its is assumed that the measurements are in the order (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MeasurementModel.dim = 3;
            MeasurementModel.H = @(x) single([1,0,0,0;...
                                              0,1,0,0;...
                                              0,0,1,0;...
                                              0,0,0,1]);
            MeasurementModel.R = single([sigmaSq_y0, 0,0,0;...
                                         0,sigmaSq_A0, 0,0;...
                                         0,0,sigmaSq_K0 ,0;...
                                         0,0,0,sigmaSq_Khat]);
            MeasurementModel.h = @(x) MeasurementModel.H(x) * x;
        end
        % ======================================================================================================================================================
        function state_upd = update(state_pred, z, measmodel)
            % performs linear/nonlinear (Extended) Kalman update step
            % INPUT: z: measurement (measurement dimension) x 1 vector
            %        state_pred: a structure with two fields:
            %                x: predicted object state mean (state dimension) x 1 vector 
            %                P: predicted object state covariance (state dimension) x (state dimension) matrix
            %        K: Kalman Gain
            %        measmodel: a structure specifies the measurement model parameters 
            % OUTPUT:state_upd: a structure with two fields:
            %                   x: updated object state mean (state dimension) x 1 vector                 
            %                   P: updated object state covariance (state dimension) x (state dimension) matrix 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            Hx = measmodel.H(state_pred.x);     %measurement model jacobian
            S = Hx * state_pred.P * Hx' + measmodel.R; %Innovation covariance
            S = (S + S')/2;                     %Make sure matrix S is positive definite
            K = (state_pred.P * Hx')/S;         %kalman gain
            state_upd.x = state_pred.x + K*(z - measmodel.h(state_pred.x));  %State update
            state_upd.P = (eye(size(state_pred.x,1)) - K*Hx)*state_pred.P;   %Covariance update
        end
        % ======================================================================================================================================================
        function LANE_TRACK_BOUNDARY = LANE_LINE_STRUCTURE(LANE_TRACK_BOUNDARY_in, LANE_LINE_MEAS_MAT, measCov)
            LANE_TRACK_BOUNDARY = LANE_TRACK_BOUNDARY_in;
            nLineMeas = LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(end);
            for i = 1:nLineMeas
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.LateralOffset = LANE_LINE_MEAS_MAT.MeasArray(1,i);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.HeadingAngle = LANE_LINE_MEAS_MAT.MeasArray(2,i);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.Curvature = LANE_LINE_MEAS_MAT.MeasArray(3,i);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.CurvatureDerivative = LANE_LINE_MEAS_MAT.MeasArray(4,i);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.CurveLength = LANE_LINE_MEAS_MAT.MeasArray(5,i);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.ErrCOV = measCov;
            end
            LANE_TRACK_BOUNDARY.nValidTracks = nLineMeas;
        end
        % ======================================================================================================================================================
        function[Width, RoadWidth] = ComputeLaneWidth(LANE_TRACK_BOUNDARY)
            L1 = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurveLength;
            L2 = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurveLength;
            if(L1 >= L2)
                x0_main   = 0;
                y0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.LateralOffset;
                a0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.HeadingAngle;
                k0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.Curvature;
                khat_main = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurvatureDerivative;
                L_main    = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurveLength;
         
                x0_other   = 0;
                y0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.LateralOffset;
                a0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.HeadingAngle;
                k0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.Curvature;
                khat_other = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurvatureDerivative;
                L_other    = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurveLength;
            else
                x0_main   = 0;
                y0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.LateralOffset;
                a0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.HeadingAngle;
                k0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.Curvature;
                khat_main = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurvatureDerivative;
                L_main    = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurveLength;
         
                x0_other   = 0;
                y0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.LateralOffset;
                a0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.HeadingAngle;
                k0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.Curvature;
                khat_other = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurvatureDerivative;
                L_other    = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurveLength;
            end
            nn = 100; dL = L_other/(nn-1); s = 0:dL:L_other;
            other_phi_s = a0_other + abs(k0_other).*s + 0.5*abs(khat_other).*s.^2;
            main_curv_s = ( sqrt(k0_main^2 -2*abs(khat_main).*(a0_main - other_phi_s)) - abs(k0_main) )/abs(khat_main);
      
            [xMain, yMain, a0Main, ~] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0_main, y0_main, a0_main, k0_main, khat_main, main_curv_s, L_main);
            [xOther, yOther, a0Other, ~] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0_other, y0_other, a0_other, k0_other, khat_other, s, L_other);
            RoadWidth = single(size(main_curv_s));
            X1 = single(size(main_curv_s)); Y1 = single(size(main_curv_s));
            X2 = single(size(main_curv_s)); Y2 = single(size(main_curv_s));
            for i = 1:length(main_curv_s)
                x1 = xMain(i);  y1 = yMain(i);
                x2 = xOther(i); y2 = yOther(i);
                %d = sqrt((x1 - x2)^2 + (y1 - y2)^2);
                m1 = tan(a0Main(i));
                c1 = y1 - m1*x1;
                m2 = -(1/tan(a0Other(i)));
                c2 = y2 - m2*x2;
                x = (c1 - c2)/(m2 - m1);
                y = m1*x + c1;
                RoadWidth(i) = sqrt((x - x2)^2 + (y - y2)^2);
                X1(i) = x; Y1(i) = y; X2(i) = x2; Y2(i) = y2;
            end
            RoadWidth = real(RoadWidth);
            Width = sum(RoadWidth)/length(RoadWidth);
        end
        % ======================================================================================================================================================
        function EgoLaneCenter = LANE_LINE_MODEL(EgoLaneCenter_in, EGO_LANE_CENTER, EGO_CAN_BUS, dT)
            EgoLaneCenter = EgoLaneCenter_in;
            if(EgoLaneCenter.validCurvLenFront == 0 && EgoLaneCenter.validCurvLenRear == 0 && EGO_LANE_CENTER.CurveLength == 0)
               return;
            end
            % ego vehicle
            HistoryLimit = 2000000;
            DEG2RAD = single(pi/180); nPts = 100;
            dyaw  = (-1) * DEG2RAD * EGO_CAN_BUS.yawRate * dT;
            vEgo  = sqrt( (EGO_CAN_BUS.vx)^2 + (EGO_CAN_BUS.vy)^2 ); dS = vEgo * dT;
            vxEgo = vEgo*cos(dyaw); vyEgo = vEgo*sin(dyaw); dXEgo = [-vxEgo*dT ; -vyEgo*dT];
            yawCompMat = [ cos(dyaw), -sin(dyaw); sin(dyaw), cos(dyaw)];
            % ego lane clothoid parameters
            x0 = 0; y0 = EGO_LANE_CENTER.LateralOffset; phi0 = EGO_LANE_CENTER.HeadingAngle; 
            K0 = EGO_LANE_CENTER.Curvature; Khat = EGO_LANE_CENTER.CurvatureDerivative; L = EGO_LANE_CENTER.CurveLength;
            % generate points for the first time
            if(EgoLaneCenter.validCurvLenFront == 0 && EgoLaneCenter.validCurvLenRear == 0 && EGO_LANE_CENTER.CurveLength ~= 0)
               sMax = EGO_LANE_CENTER.CurveLength; sMin = 0;
               ds = (sMax - sMin)/(nPts - 1); s = sMax:-ds:sMin;
               [x, y, phi, K] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0, y0, phi0, K0, Khat, s, L);
               EgoLaneCenter.X(1,1:nPts) = x;
               EgoLaneCenter.Y(1,1:nPts) = y;
               EgoLaneCenter.arcLength(1,1:nPts) = s;
               EgoLaneCenter.HeadingAngle(1,1:nPts) = phi;
               EgoLaneCenter.Curvature(1,1:nPts) = K;
               EgoLaneCenter.CurvatureDerivative(1,1:nPts) = Khat;
               EgoLaneCenter.validCurvLenFront = sMax;
               EgoLaneCenter.validCurvLenRear = sMin;
               EgoLaneCenter.BufferLastValidIdx = nPts;
            else % update the buffer
               lastBuffIdx = EgoLaneCenter.BufferLastValidIdx;     % last occupied index of the history
               prev_s = EgoLaneCenter.arcLength(1,1:lastBuffIdx);  % arc lengths from the previous cycle
               curr_s = prev_s - dS;                               % compensate the arc lengths for the ego vehicle movement dS
               idxBack = find(curr_s(1,1:lastBuffIdx) < 0);        % list of index whose arc lengths are negetive
               idxFrnt = find(curr_s(1,1:lastBuffIdx) >= 0);       % list of index whose arc lengths are positive
               sMax = EGO_LANE_CENTER.CurveLength;                 % maximum arc length
               sMin = curr_s(1,idxFrnt(1,end));                    % minimum positive arc length
               sMax = sMax + sMin;                                 % extended maximum arc length
               ds = (sMax - sMin)/(nPts - 1); s = sMax:-ds:sMin;  % generate new points 
               [x, y, phi, K] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0, y0, phi0, K0, Khat, s, L);
               nPtsBack = length(idxBack);                         % number of history points
               nPtsTobeRemoved = 0;
               if(nPtsBack > HistoryLimit)                         % if the number of points exceeds a threshold then remove the 'oldest' points
                  nPtsTobeRemoved = nPtsBack - HistoryLimit;
               end
               % extract the relevant points for history 
               idxHistory = find(curr_s(1,1:(lastBuffIdx-nPtsTobeRemoved)) < 0);             % History that needs to be maintained 
               Xprev = EgoLaneCenter.X(1,idxHistory); Yprev = EgoLaneCenter.Y(1,idxHistory); % x, y prev
               Sprev = EgoLaneCenter.arcLength(1,idxHistory);                                % arc lengths prev
               phiPrev = EgoLaneCenter.HeadingAngle(1,idxHistory);                           % phi prev
               kPrev = EgoLaneCenter.Curvature(1,idxHistory);                                % K prev
               khatPrev = EgoLaneCenter.CurvatureDerivative(1,idxHistory);                   % k hat prev
               % ego motion compensation to generate history
               XX = [Xprev ; Yprev];
               xHist = yawCompMat * XX + dXEgo;
               sHist = Sprev - dS;
               phiHist = phiPrev + dyaw;
               kHist = kPrev;
               khatHist = khatPrev;
               % update the data inside the buffer (front)
               EgoLaneCenter.X(1,1:nPts) = x;
               EgoLaneCenter.Y(1,1:nPts) = y;
               EgoLaneCenter.arcLength(1,1:nPts) = s;
               EgoLaneCenter.HeadingAngle(1,1:nPts) = phi;
               EgoLaneCenter.Curvature(1,1:nPts) = K;
               EgoLaneCenter.CurvatureDerivative(1,1:nPts) = Khat;
               EgoLaneCenter.validCurvLenFront = sMax;
               % update the data inside the buffer (rear)
               nPtsHistory = length(idxHistory);
               idxStart = nPts + 1; idxEnd = nPts + nPtsHistory;
               EgoLaneCenter.X(1,idxStart:idxEnd) = xHist(1,:);
               EgoLaneCenter.Y(1,idxStart:idxEnd) = xHist(2,:);
               EgoLaneCenter.arcLength(1,idxStart:idxEnd)    = sHist;
               EgoLaneCenter.HeadingAngle(1,idxStart:idxEnd) = phiHist;
               EgoLaneCenter.Curvature(1,idxStart:idxEnd)    = kHist;
               EgoLaneCenter.CurvatureDerivative(1,idxStart:idxEnd) = khatHist;
               EgoLaneCenter.validCurvLenRear = min(sHist);
               EgoLaneCenter.BufferLastValidIdx = idxEnd;
            end
        end
        % ======================================================================================================================================================
        function LaneBoundaryCoeff = LANE_GEOMETRY(LaneBoundaryCoeff_in, EGO_LANE_CENTER, d)
            LaneBoundaryCoeff = LaneBoundaryCoeff_in;
            if(EGO_LANE_CENTER.CurveLength == 0)
               return;
            end
            [~, y0, phi0, K0, K_hat, L] = LANE_ESTIMATION.GenerateLaneHypothesis(EGO_LANE_CENTER, d);
            LaneBoundaryCoeff.LateralOffset = y0;
            LaneBoundaryCoeff.HeadingAngle = phi0;
            LaneBoundaryCoeff.Curvature = K0;
            LaneBoundaryCoeff.CurvatureDerivative = K_hat;
            LaneBoundaryCoeff.CurveLength = L;
        end
        % ======================================================================================================================================================
        function EGO_LANE_HYPOTHESIS = EGO_LANE_HYPOTHESIS(EGO_LANE_HYPOTHESIS_in, LANE_BOUNDARY, LaneHalfWidth)
            EGO_LANE_HYPOTHESIS = EGO_LANE_HYPOTHESIS_in;
            if(LANE_BOUNDARY.nValidTracks == 0)
               return;
            end
            for i = 1:LANE_BOUNDARY.nValidTracks
                [~, y0, phi0, K0, K_hat, L] = LANE_ESTIMATION.GenerateEgoLaneHypothesis(LANE_BOUNDARY.LaneTrackParam(i).ClothoidParam, LaneHalfWidth);
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset = y0;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle = phi0;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature = K0;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative = K_hat;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength = L;
            end
            EGO_LANE_HYPOTHESIS.nValidTracks = LANE_BOUNDARY.nValidTracks;
        end
        % ======================================================================================================================================================
        function [x0, y0, phi0, K0, K_hat, L] = GenerateLaneHypothesis(a, d)
            x0 = 0; y0 = a.LateralOffset;                         % extract the init x and y
            phi0 = a.HeadingAngle; K0 = a.Curvature;              % extract init heading and curvature
            Khat = a.CurvatureDerivative; L = a.CurveLength;      % extract the curvature rate and curve length
            nPts = 100; sMax = L; sMin = 0; ds = (sMax - sMin)/(nPts - 1); s = sMin:ds:sMax;  % generate 'nPts' equally spaced arc along the curve
            alphaSegment = phi0 + K0.*s + 0.5*Khat.*s.^2;         % compute the heading at those 'nPts' positions along the curve
            slopes = -1./alphaSegment;                            % compute the slope of the line perpendicular to heading
            [xSeg, ySeg, ~, ~] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0, y0, phi0, K0, Khat, s, L); % generate x,y points along the curve
            X = xSeg - d*sqrt(1./(1+slopes.^2));       % compute the X, Y coordinates at the end of the perpendiculars
            Y = ySeg - d*abs(slopes).*sqrt(1./(1+slopes.^2));
            C = LANE_ESTIMATION.LeastSquaresFit3rdDegreePolynomial(X, Y);   % least squares fitting of a 3rd order polynomial
            [x0, y0, phi0, K0, K_hat, L] = LANE_ESTIMATION.Polynomial2Clothoid_v2(C(1), C(2), C(3), C(4), 0, max(X), 10000);
        end
        % ======================================================================================================================================================
        function [x0, y0, phi0, K0, K_hat, L] = GenerateEgoLaneHypothesis(a, d)
            x0 = 0; y0 = a.LateralOffset;                         % extract the init x and y
            phi0 = a.HeadingAngle; K0 = a.Curvature;              % extract init heading and curvature
            Khat = a.CurvatureDerivative; L = a.CurveLength;      % extract the curvature rate and curve length
            nPts = 100; sMax = L; sMin = 0; ds = (sMax - sMin)/(nPts - 1); s = sMin:ds:sMax;  % generate 'nPts' equally spaced arc along the curve
            alphaSegment = phi0 + K0.*s + 0.5*Khat.*s.^2;         % compute the heading at those 'nPts' positions along the curve
            slopes = -1./alphaSegment;                            % compute the slope of the line perpendicular to heading
            [xSeg, ySeg, ~, ~] = LANE_ESTIMATION.CoordinatesClothidToCartesian(x0, y0, phi0, K0, Khat, s, L); % generate x,y points along the curve
            X = xSeg - (sign(y0)*d)*sqrt(1./(1+slopes.^2));       % compute the X, Y coordinates at the end of the perpendiculars
            Y = ySeg - (sign(y0)*d).*abs(slopes).*sqrt(1./(1+slopes.^2));
            C = LANE_ESTIMATION.LeastSquaresFit3rdDegreePolynomial(X, Y);   % least squares fitting of a 3rd order polynomial
            [x0, y0, phi0, K0, K_hat, L] = LANE_ESTIMATION.Polynomial2Clothoid_v2(C(1), C(2), C(3), C(4), 0, max(X), 10000);
        end
        % ======================================================================================================================================================
        function EGO_LANE_CENTER = EGO_LANE_PREDICTION(EGO_LANE_CENTER_in, EGO_CAN_BUS, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dT)
            EGO_LANE_CENTER = EGO_LANE_CENTER_in;
            if(EGO_LANE_CENTER.CurveLength == 0)
               return;
            end
            DEG2RAD = single(pi/180);
            vEgo  = sqrt( (EGO_CAN_BUS.vx)^2 + (EGO_CAN_BUS.vy)^2 );
            dYaw  = (-1) * DEG2RAD * EGO_CAN_BUS.yawRate * dT;
            dS = vEgo * dT;
            state.x = [EGO_LANE_CENTER.LateralOffset; ...
                       EGO_LANE_CENTER.HeadingAngle; ... 
                       EGO_LANE_CENTER.Curvature; ...
                       EGO_LANE_CENTER.CurvatureDerivative];
            state.P  = EGO_LANE_CENTER.ErrCOV;
            curvLen = EGO_LANE_CENTER.CurveLength;
            [state, ~] = LANE_ESTIMATION.ConstantCurvRateModel(state, curvLen, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dS, dYaw);  % prediction and ego yaw rate compensation
            EGO_LANE_CENTER.LateralOffset = state.x(1,1);
            EGO_LANE_CENTER.HeadingAngle = state.x(2,1);
            EGO_LANE_CENTER.Curvature = state.x(3,1);
            EGO_LANE_CENTER.CurvatureDerivative = state.x(4,1);
            EGO_LANE_CENTER.ErrCOV = state.P;
            EGO_LANE_CENTER.CurveLength = EGO_LANE_CENTER.CurveLength - dS;
        end
        % ======================================================================================================================================================
        function EGO_LANE_HYPOTHESIS = EGO_LANE_UPDATE(EGO_LANE_HYPOTHESIS_in, EGO_LANE_CENTER, measmodel)
            EGO_LANE_HYPOTHESIS = EGO_LANE_HYPOTHESIS_in;
            if(EGO_LANE_CENTER.CurveLength == 0)
               return;
            end
           if(EGO_LANE_CENTER.CurveLength ~= 0 && EGO_LANE_HYPOTHESIS.nValidTracks == 0)
              for i = EGO_LANE_HYPOTHESIS.nValidTracks
                  EGO_LANE_HYPOTHESIS.ClothoidParam(1,i) = EGO_LANE_CENTER;
              end
              return;
           end
           X.x = [EGO_LANE_CENTER.LateralOffset; ...
                  EGO_LANE_CENTER.HeadingAngle; ...
                  EGO_LANE_CENTER.Curvature; ...
                  EGO_LANE_CENTER.CurvatureDerivative];
           X.P = EGO_LANE_CENTER.ErrCOV;
           for i = EGO_LANE_HYPOTHESIS.nValidTracks
               Z = [EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset; ...
                    EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle; ...
                    EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature; ...
                    EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative];
               StateUpdate = LANE_ESTIMATION.update(X, Z, measmodel);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset = StateUpdate.x(1,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle = StateUpdate.x(2,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature = StateUpdate.x(3,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative = StateUpdate.x(4,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).ErrCOV = StateUpdate.P;
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength = max(EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength, EGO_LANE_CENTER.CurveLength);
           end
        end
        % ======================================================================================================================================================
        function EGO_LANE_CENTER = EGO_LANE_TRACK_TO_TRACK_FUSION_v2(EGO_LANE_CENTER_in, EGO_LANE_HYPOTHESIS)
               EGO_LANE_CENTER = EGO_LANE_CENTER_in;
               if(EGO_LANE_HYPOTHESIS.nValidTracks == 0)
                  return;
               end
               if(EGO_LANE_HYPOTHESIS.nValidTracks == 1)
                  EGO_LANE_CENTER = EGO_LANE_HYPOTHESIS.ClothoidParam(1,1);
               else
                  X = single(zeros(5,EGO_LANE_HYPOTHESIS.nValidTracks)); 
                  P = single(zeros(4,4,EGO_LANE_HYPOTHESIS.nValidTracks));
                  for i = 1:EGO_LANE_HYPOTHESIS.nValidTracks
                      X(1,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset;
                      X(2,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle;
                      X(3,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature;
                      X(4,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative;
                      X(5,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength;
                      P(:,:,i) = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).ErrCOV;
                  end
                  Xfus = single(zeros(5,1)); Pfus = single(zeros(4,4));
                  Lmax = max(X(5,:)); W = X(5,:)/sum(X(5,:));
                  for i = 1:EGO_LANE_HYPOTHESIS.nValidTracks
                      Xfus = Xfus + W(1,i).*X(:,i);
                      Pfus = Pfus + W(1,i).*(reshape(P(:,:,i),[4,4]));
                  end
                  EGO_LANE_CENTER.LateralOffset       = Xfus(1,1);
                  EGO_LANE_CENTER.HeadingAngle        = Xfus(2,1);
                  EGO_LANE_CENTER.Curvature           = Xfus(3,1);
                  EGO_LANE_CENTER.CurvatureDerivative = Xfus(4,1);
                  EGO_LANE_CENTER.CurveLength         = Lmax;
                  %EGO_LANE_CENTER.CurveLength         = 100;
                  EGO_LANE_CENTER.ErrCOV              = Pfus;
               end
        end
        % ======================================================================================================================================================
    end
end