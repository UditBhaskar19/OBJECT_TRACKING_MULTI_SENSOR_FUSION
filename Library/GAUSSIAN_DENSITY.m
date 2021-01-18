% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Gaussian density manipulation functions, contains the following functions : 
%             : predict()                       : kalman filter prediction linear/Jacobian 
%             : InovationCovAndKalmanGain()     : Innovation covariance and kalman gain computation
%             : update()                        : kalman filter state update
%             : momentMatching()                : Approximate a Gaussian mixture density as a single Gaussian using moment matching (2 moments)
%             : WeightedAverage()               : Approximate a Gaussian mixture density as a single Gaussian using moment matching (1 moment)
%             : CovarianceIntersection()        : Covariance intersection of gaussian densities for track to track fusion
%             : mixtureReduction()              : uses a greedy merging method to reduce the number of Gaussian components for a Gaussian mixture density 
%             : normalizeLogWeights()           : Normalize the weights in log scale
%             : prune()                         : prunes hypotheses with small weights
%             : cap()                           : keeps M hypotheses with the highest weights and discard the rest 
%             : merge()                         : merges hypotheses within small Mahalanobis distance
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef GAUSSIAN_DENSITY
    methods(Static)
        % ======================================================================================================================================================
        function StatePred = predict(state, motionmodel)
            %performs linear/nonlinear(Extended) Kalman prediction step 
            %INPUT: state: a structure with two fields:
            %           x: object state mean (dim x 1 vector) 
            %           P: object state covariance ( dim x dim matrix ) 
            %       motionmodel: a structure specifies the motion model parameters 
            %OUTPUT:state_pred: a structure with two fields:
            %           x: predicted object state mean (dim x 1 vector)
            %           P: predicted object state covariance ( dim x dim matrix )
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            StatePred.x = motionmodel.f(state.x);
            StatePred.P = motionmodel.F(state.x) * state.P * motionmodel.F(state.x)' + motionmodel.Q;
        end
        % ======================================================================================================================================================
        function [S, K] = InovationCovAndKalmanGain(statePred, measmodel, R)
            % Computes the innovation covariance and Kalman Gain 
            % INPUT: state: a structure with two fields:
            %            x: object state mean (dim x 1 vector) 
            %            P: object state covariance ( dim x dim matrix )
            %        z: measurement
            %        R: measurement noise covariance
            %        measmodel: a structure specifies the measurement model parameters 
            % OUTPUT: S: Innovation Covariance
            %         K: Kalman Gain
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            H = measmodel.H(statePred.x); %Measurement model Jacobian
            S = H * statePred.P * H' + R; %Innovation covariance
            S = (S + S')/2;               %Make sure matrix S is positive definite
            K = (statePred.P * H')/S;     %kalman gain
        end
        % ======================================================================================================================================================
        function state_upd = update(state_pred, z, K, measmodel)
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
            Hx = measmodel.H(state_pred.x); %measurement model jacobian
            state_upd.x = state_pred.x + K*(z - measmodel.h(state_pred.x));  %State update
            state_upd.P = (eye(size(state_pred.x,1)) - K*Hx)*state_pred.P;   %Covariance update
        end
        % ======================================================================================================================================================
        function [x, P] = momentMatching(weights, X)
            % Approximate a Gaussian mixture density as a single Gaussian using moment matching 
            % INPUT: weights: normalised weight of Gaussian components in logarithm domain (number of Gaussians) x 1 vector 
            %              X: structure array of size (number of Gaussian components x 1), each structure has two fields 
            %           mean: means of Gaussian components (variable dimension) x 1 vector 
            %              P: variances of Gaussian components (variable dimension) x (variable dimension) matrix  
            % OUTPUT:  state: a structure with two fields:
            %              x: approximated mean (variable dimension) x 1 vector 
            %              P: approximated covariance (variable dimension) x (variable dimension) matrix 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nComponents = length(weights);
            if nComponents == 1
                x = X(1,1).x;
                P = X(1,1).P;
                return;
            end
            mixtureMean = 0; Paverage = 0; meanSpread = 0;
            w = exp(weights);     % convert normalized weights from log scale to linear scale (exp(w))
            for idx = 1:nComponents    % compute the weighted average of the means (in the gaussian mixture)
                mixtureMean = mixtureMean + w(1,idx) * X(1,idx).x;
            end
            for idx = 1:nComponents                        % compute weighted average covariance and spread of the mean
                Paverage = Paverage + w(1,idx) * X(1,idx).P; % compute weighted average covariance
                meanSpread = meanSpread + w(1,idx).*( ( mixtureMean - X(1,idx).x )*( mixtureMean - X(1,idx).x )' ); % spread of the mean
            end
            x = mixtureMean;
            P = Paverage + meanSpread;
        end
        % ======================================================================================================================================================
        function [x, P] = WeightedAverage(weights, X)
            % Approximate a Gaussian mixture density as a single Gaussian using weighted average 
            % INPUT: weights: normalised weight of Gaussian components in logarithm domain (number of Gaussians) x 1 vector 
            %              X: structure array of size (number of Gaussian components x 1), each structure has two fields 
            %           mean: means of Gaussian components (variable dimension) x 1 vector 
            %              P: variances of Gaussian components (variable dimension) x (variable dimension) matrix  
            % OUTPUT:  state: a structure with two fields:
            %              x: approximated mean (variable dimension) x 1 vector 
            %              P: approximated covariance (variable dimension) x (variable dimension) matrix 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nComponents = length(weights);
            if nComponents == 1
                x = X(1,1).x;
                P = X(1,1).P;
                return;
            end
            mixtureMean = 0; Paverage = 0;
            w = exp(weights);     % convert normalized weights from log scale to linear scale (exp(w))
            for idx = 1:nComponents    % compute the weighted average of the means (in the gaussian mixture)
                mixtureMean = mixtureMean + w(1,idx) * X(1,idx).x;
            end
            for idx = 1:nComponents                        % compute weighted average covariance and spread of the mean
                Paverage = Paverage + w(1,idx) * X(1,idx).P; % compute weighted average covariance
            end
            x = mixtureMean;
            P = Paverage;
        end
        % ======================================================================================================================================================
        function [X, P] = CovarianceIntersection(Xr, W)
            % Fusion of Gaussian Distributions by Covarience Intersection method
            % INPUT : W  : normalised weight of Gaussian components (row vector : 1 x N )
            %       : Xr : structure array of size (1 x N), each structure has two fields
            %          x : mean
            %          P : covariance
            % OUTPUT : X : mean vector by covariance intersection
            %        : P : covariance intersection matrix
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            if(length(W) == 1)
                X = Xr(1,1).x; P = Xr(1,1).P;
                return;
            end
            dim = size(Xr(1,1).x, 1); 
            Pavginv = 0; temp = 0; I = eye(dim);
            for i = 1:length(W)
                Pavginv = Pavginv + W(1,i).*((Xr(1,i).P)\I);
                temp = temp + W(1,i).*((Xr(1,i).P\I) * Xr(1,i).x);
            end
            P = Pavginv\I;
            X = P*temp;
        end
        % ======================================================================================================================================================
        function [w_hat, X_hat, nCluster] = mixtureReduction(w, X, nComponents, threshold)
            %  uses a greedy merging method to reduce the number of Gaussian components for a Gaussian mixture density 
            %  INPUT: w: normalised weight of Gaussian components in logarithmic scale (number of Gaussians) x 1 vector 
            %         X: structure array of size (number of Gaussian components x 1), each structure has two fields 
            %      mean: means of Gaussian components (variable dimension) x (number of Gaussians) matrix 
            %         P: variances of Gaussian components (variable dimension) x (variable dimension) x (number of Gaussians) matrix  
            % threshold: merging threshold 
            % OUTPUT: w_hat: normalised weight of Gaussian components in logarithmic scale after merging (number of Gaussians) x 1 vector  
            %         X_hat: structure array of size (number of Gaussian components after merging x 1), each structure has two fields  
            %             x: means of Gaussian components (variable dimension) x (number of Gaussians after merging) matrix  
            %             P: variances of Gaussian components (variable dimension) x (variable dimension) x (number of Gaussians after merging) matrix 
            %      nCluster: number of merged components
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            if nComponents == 1       % if the number of components is 1 no merging is needed
               X_hat(1) = X(1);
               w_hat = w;
               return;
            end
            X_hat = X; w_hat = w;      % initialization
            indexSet = 1:nComponents;  % Index set of unvisited components
            nElements = 0;             % number of elements in the cluster
            nCluster = 0;              % number of cluster
            indexSetCluster = zeros(1,nComponents); % list of index set which are grouped (clustered) together
            while ~isempty(indexSet)   % continue if the set of index is not empty (due some componenets are not visited for mixture reduction)
                [~, maxIdx] = max(w);  % unvisited component with the highest weight 
                for index = indexSet   % Find other similar components in the sense of small Mahalanobis distance
                    dX = X(index).mean - X(maxIdx).mean;
                    mahalanobisDist = diag(dX.'*(X(maxIdx).P\dX));
                    if mahalanobisDist <= threshold 
                       nElements = nElements + 1;
                       indexSetCluster(nElements) = index;
                    end
                end
                % normalize log weights  and  Gaussian mixture approximation
                nCluster = nCluster + 1;
                [tempWeights, w_hat(nCluster)] = STATE_ESTIMATOR.normalizeLogWeights(w(indexSetCluster(1:nElements)));
                [X_hat(nCluster).mean, X_hat(nCluster).P] = STATE_ESTIMATOR.momentMatching(tempWeights, X(indexSetCluster(1:nElements)));
                indexSet = setdiff(indexSet,indexSetCluster(1:nElements)); %Remove indices of merged components from index set
                w(indexSetCluster(1:nElements)) = log(eps);
                nElements = 0;
            end 
        end
        % ======================================================================================================================================================
        function [LogWeights, sumLogWeights] = normalizeLogWeights(LogWeights)
            % Normalize the weights in log scale
            % INPUT  :    LogWeights: log weights, e.g., log likelihoods
            % OUTPUT :    LogWeights: log of the normalized weights
            %        : sumLogWeights: log of the sum of the non-normalized weights
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            if length(LogWeights) == 1
               sumLogWeights = LogWeights;
               LogWeights = LogWeights - sumLogWeights;
               return;
            end
            [logWeights_aux, Index] = sort(LogWeights, 'descend');
            sumLogWeights = max(logWeights_aux) + log(1 + sum(exp(LogWeights(Index(2:end)) - max(logWeights_aux))));
            LogWeights = LogWeights - sumLogWeights;   % normalize
        end
        % ======================================================================================================================================================
        function [hypothesesWeight, multiHypotheses] = prune(hypothesesWeight, multiHypotheses, threshold)
            % prunes hypotheses with small weights
            % INPUT: hypothesesWeight: the weights of different hypotheses in logarithmic scale :(number of hypotheses) x 1 vector
            %        multiHypotheses : (number of hypotheses) x 1 structure
            %        threshold       : hypotheses with weights smaller than this threshold will be discarded scalar in logarithmic scale
            % OUTPUT:hypothesesWeight: hypotheses weights after pruning in logarithmic scale (number of hypotheses after pruning) x 1 vector   
            %        multiHypotheses : (number of hypotheses after pruning) x 1 structure  
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            n = length(hypothesesWeight);
            hypoIdx = single(zeros(1,n));
            count = 0;
            for idx = 1:n
                if hypothesesWeight(idx) >= threshold
                   count = count + 1;
                   hypoIdx(1,count) =  hypothesesWeight(idx,1);
                end
            end
            hypothesesWeight = hypothesesWeight(hypoIdx(1,1:count));
            multiHypotheses = multiHypotheses(hypoIdx(1,1:count));
        end
        % ======================================================================================================================================================
        function [hypothesesWeight, multiHypotheses] = cap(hypothesesWeight, multiHypotheses, M)
            % keeps M hypotheses with the highest weights and discard the rest 
            % INPUT: hypothesesWeight : the weights of different hypotheses in logarithmic scale (number of hypotheses) x 1 vector  
            %        multiHypotheses  : (number of hypotheses) x 1 structure
            %        M                : only keep M hypotheses scalar
            % OUTPUT: hypothesesWeight: hypotheses weights after capping in logarithmic scale (number of hypotheses after capping) x 1 vector 
            %         multiHypotheses : (number of hypotheses after capping) x 1 structure 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            [hypothesesWeight, Index] = sort(hypothesesWeight,'descend');
            hypothesesWeight = hypothesesWeight(1:M);
            multiHypotheses = multiHypotheses(Index(1:M));
        end
        % ======================================================================================================================================================
        function [hypothesesWeight,multiHypotheses] = merge(hypothesesWeight,multiHypotheses,threshold)
            % merges hypotheses within small Mahalanobis distance
            % INPUT: hypothesesWeight: the weights of different hypotheses in logarithmic scale (number of hypotheses) x 1 vector  
            %        multiHypotheses : (number of hypotheses) x 1 structure
            %        threshold       : merging threshold
            % OUTPUT: hypothesesWeight: hypotheses weights after merging in logarithmic scale (number of hypotheses after merging) x 1 vector  
            %          multiHypotheses: (number of hypotheses after merging) x 1 structure 
            [hypothesesWeight,multiHypotheses] = GAUSSIAN_DENSITY.mixtureReduction(hypothesesWeight,multiHypotheses,threshold);
        end
        % ======================================================================================================================================================
    end
end