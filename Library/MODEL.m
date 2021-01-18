% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Filter Parameters Set functions, Contains the follwing :
%             : cvmodel()             : constant velocity model
%             : camodel()             : constant acceleration model
%             : cvmeasmodelPxVxPyVy() : position and velocity linear observation model for constant velocity process model
%             : cvmeasmodelPxPy()     : position linear observation model for constant velocity process model
%             : cameasmodelPxVxPyVy() : position, velocity linear observation model for constant acceleration process model
%             : cameasmodelPxPy()     : position linear observation model for constant acceleration process model
%             : measmodelEye2D()      : identity matrix as an observation model
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef MODEL
    methods(Static)
        % ======================================================================================================================================================
        function ProcessModel = cvmodel(dT, sigma)
            % creates a 2D nearly constant velocity model 
            % INPUT: dT: sampling time 
            %        sigma: standard deviation of motion noise
            % OUTPUT:obj.d: object state dimension
            %        obj.F: function handle return a motion transition matrix (4 x 4 matrix) 
            %        obj.Q: motion noise covariance (4 x 4 matrix)
            %        obj.f: function handle return state prediction --- (4 x 1 vector) 
            % NOTE:  the motion model (and correspondingly the covariance also) assumes that the state vector consists of the order : (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            ProcessModel.type = 'Linear';   % to indicate it is a linear process model
            ProcessModel.name = 'cv';       % to indicate it is a constant velocity motion model     
            ProcessModel.dim = 4; % state dimension
            ProcessModel.F = @(x) single([1, dT,  0,0;...   % process model
                                          0,  1,  0,0;...
                                          0,0,    1, dT; ...
                                          0,0,    0,  1]);
            ProcessModel.Q = single(sigma^2 * [dT^4/4,  dT^3/2,  0,0; ... % process noise covariance
                                               dT^3/2,  dT^2,    0,0; ...
                                               0,0,     dT^4/4,  dT^3/2; ...
                                               0,0,     dT^3/2,  dT^2]);
            ProcessModel.f = @(x) ProcessModel.F(x) * x;
        end
        % ======================================================================================================================================================
        function ProcessModel = camodel(dT, sigma)
            % creates a 2D nearly constant acceleration model 
            % INPUT: dT: sampling time 
            %        sigma: standard deviation of motion noise
            % OUTPUT:obj.d: object state dimension
            %        obj.F: function handle return a motion transition matrix (6 x 6 matrix) 
            %        obj.Q: motion noise covariance (6 x 6 matrix)
            %        obj.f: function handle return state prediction --- (6 x 1 vector) 
            % NOTE:  the motion model (and correspondingly the covariance also) assumes that the state vector consists of the order : (px, vx, py, vy) 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            ProcessModel.type = 'Linear';   % to indicate it is a linear process model
            ProcessModel.name = 'ca';       % to indicate it is a constant acceleration motion model 
            ProcessModel.dim = 6; % state dimension
            ProcessModel.F = @(x) single([1, dT, 0.5*dT*dT, 0,0,0; ...
                                          0, 1,  dT,        0,0,0; ...
                                          0, 0,  1,         0,0,0; ...
                                          0,0,0,            1, dT, 0.5*dT*dT; ...
                                          0,0,0,            0, 1,  dT; ...
                                          0,0,0,            0, 0,  1]);
            ProcessModel.Q = single(sigma^2*[dT^5/20,    dT^4/8,    dT^3/6,    0,0,0; ...
                                             dT^4/8 ,    dT^3/3,    dT^2/2,    0,0,0; ...
                                             dT^3/6 ,    dT^2/2,    dT,        0,0,0; ...
                                             0,0,0,      dT^5/20,   dT^4/8,    dT^3/6; ... 
                                             0,0,0,      dT^4/8 ,   dT^3/3,    dT^2/2; ...
                                             0,0,0,      dT^3/6 ,   dT^2/2,    dT]);
            ProcessModel.f = @(x) ProcessModel.F(x) * x;
        end
        % ======================================================================================================================================================
        function MeasurementModel = cvmeasmodelPxVxPyVy()
            % creates the measurement model for a 2D nearly constant velocity motion model 
            % INPUT: sigmaPx, sigmaPy, sigmaVx, sigmaVy: standard deviation of measurement noise 
            % OUTPUT: obj.d: measurement dimension 
            %         obj.H: function handle return an observation matrix (4 x 4 matrix) 
            %         obj.R: measurement noise covariance (4 x 4 matrix)
            %         obj.h: function handle return a measurement (4 x 1 vector)
            % Its is assumed that the measurements are in the order (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MeasurementModel.dim = 4;
            MeasurementModel.H = @(x) single([1,0,0,0;...
                                              0,1,0,0;...
                                              0,0,1,0;...
                                              0,0,0,1]);
            %MeasurementModel.R = single([sigmaPx^2, 0,0,0;...
            %                             0 ,sigmaVx^2,0,0;...
            %                             0,0, sigmaPy^2,0;...
            %                             0,0,0, sigmaVy^2]);
            MeasurementModel.h = @(x) MeasurementModel.H(x) * x;
        end
        % ======================================================================================================================================================
        function MeasurementModel = cvmeasmodelPxPy()
            % creates the measurement model for a 2D nearly constant velocity motion model 
            % INPUT: sigmaPx, sigmaPy, sigmaVx, sigmaVy: standard deviation of measurement noise 
            % OUTPUT: obj.d: measurement dimension 
            %         obj.H: function handle return an observation matrix (4 x 4 matrix) 
            %         obj.R: measurement noise covariance (4 x 4 matrix)
            %         obj.h: function handle return a measurement (4 x 1 vector)
            % Its is assumed that the measurements are in the order (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MeasurementModel.dim = 2;
            MeasurementModel.H = @(x) single([1,0,0,0;...
                                              0,0,1,0]);
            %MeasurementModel.R = single([sigmaPx^2, 0,0,0;...
            %                             0 ,sigmaVx^2,0,0;...
            %                             0,0, sigmaPy^2,0;...
            %                             0,0,0, sigmaVy^2]);
            MeasurementModel.h = @(x) MeasurementModel.H(x) * x;
        end
        % ======================================================================================================================================================
        function MeasurementModel = cameasmodelPxVxPyVy()
            % creates the measurement model for a 2D nearly constant acceleration motion model 
            % INPUT: sigmaPx, sigmaPy, sigmaVx, sigmaVy: standard deviation of measurement noise 
            % OUTPUT: obj.d: measurement dimension 
            %         obj.H: function handle return an observation matrix (4 x 6 matrix) 
            %         obj.R: measurement noise covariance (4 x 4 matrix)
            %         obj.h: function handle return a measurement (4 x 1 vector)
            % Its is assumed that the measurements are in the order (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MeasurementModel.dim = 4;
            MeasurementModel.H = @(x) single([1,0,0,0,0,0;...
                                              0,1,0,0,0,0;...
                                              0,0,1,0,0,0;...
                                              0,0,0,1,0,0]);
            %MeasurementModel.R = single([sigmaPx^2, 0,0,0;...
            %                             0 ,sigmaVx^2,0,0;...
            %                             0,0, sigmaPy^2,0;...
            %                             0,0,0, sigmaVy^2]);
            MeasurementModel.h = @(x) MeasurementModel.H(x) * x;
        end
        % ======================================================================================================================================================
        function MeasurementModel = cameasmodelPxPy()
            % creates the measurement model for a 2D nearly constant acceleration motion model 
            % INPUT: sigmaPx, sigmaPy, sigmaVx, sigmaVy: standard deviation of measurement noise 
            % OUTPUT: obj.d: measurement dimension 
            %         obj.H: function handle return an observation matrix (4 x 6 matrix) 
            %         obj.R: measurement noise covariance (4 x 4 matrix)
            %         obj.h: function handle return a measurement (4 x 1 vector)
            % Its is assumed that the measurements are in the order (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MeasurementModel.dim = 2;
            MeasurementModel.H = @(x) single([1,0,0,0,0,0;...
                                              0,0,0,1,0,0]);
            %MeasurementModel.R = single([sigmaPx^2, 0,0,0;...
            %                             0 ,sigmaVx^2,0,0;...
            %                             0,0, sigmaPy^2,0;...
            %                             0,0,0, sigmaVy^2]);
            MeasurementModel.h = @(x) MeasurementModel.H(x) * x;
        end
        % ======================================================================================================================================================
        function MeasurementModel = measmodelEye2D(dim)
            % creates the measurement model for a 2D nearly constant acceleration motion model 
            % INPUT: sigmaPx, sigmaPy, sigmaVx, sigmaVy: standard deviation of measurement noise 
            % OUTPUT: obj.d: measurement dimension 
            %         obj.H: function handle return an observation matrix (4 x 6 matrix) 
            %         obj.R: measurement noise covariance (4 x 4 matrix)
            %         obj.h: function handle return a measurement (4 x 1 vector)
            % Its is assumed that the measurements are in the order (px, vx, py, vy)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MeasurementModel.dim = dim;
            MeasurementModel.H = @(x) single(eye(dim));
            MeasurementModel.h = @(x) MeasurementModel.H(x) * x;
        end
        % ======================================================================================================================================================
    end
end