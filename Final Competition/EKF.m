function [mu_t, sigma_t] = EKF(g, h, R_t, Q_t, mu_t_1, sigma_t_1, u_t, z_t, Gjac, Hjac)
% Inputs: 
%       g           g(mu_(t-1), u_t) predicts the robot's current pose by
%                   the control u_t and previous pose mu_(t-1)
%       h           h(mu_t_bar) predicts the measurement by the mu_t_bar,
%                   an output from g(mu_(t-1), u_t)
%       R_t         a covariance matrix of dynamic noises (uncertainty)
%       Q_t         a covariance matrix of the sensor measurement noises
%       mu_t_1      mu_(t-1) is the mean of the robot's pose in previous
%                   state
%       sigma_t_1   sigma_(t-1) is the covariance matrix representing where 
%                   the robot could be in previous state
%       u_t         [distance; angleTurned] the control vector at time t 
%       z_t         K by 4 measurement vector at time t
%       Gjac        a function that returns G_t ->
%                   the Jacobian matrix that corresponds to A_t and B_t in KF
%       Hjac        a function that returns H_t
%                   the Jacobian matrix that corresponds to C_t in KF
%
% Outputs:
%       mu_t        the mean of robot's current pose
%       sigma_t     the covariance matrix of the robot in current state
%
% EKF Algorithm:
%       Predict     mu_t_bar = g(mu_(t-1), u_t)
%                   sigma_t_bar = G_t * sigma_(t-1) * (G_t).' + R_t
%       Kalman gain K_t = sigma_t_bar * (H_t).' * (H_t * sigma_t_bar *
%                   (H_t).' + Q_t)^(-1)
%       Update      mu_t = mu_t_bar + K_t*(z_t - h(mu_t_bar))
%                   sigma_t = (I - K_t * H_t) * sigma_t_bar


% Get the dimension of Q to help this function identify whether it is
% working with GPS or depth measurement
[row, col] = size(Q_t);

% Get the output from Gjac
G_t = Gjac(mu_t_1, u_t);

% Prediction
mu_t_bar = g(mu_t_1, u_t(1), u_t(2));
sigma_t_bar = G_t * sigma_t_1 * (G_t).' + R_t;

% Get the output from Hjac
H_t = Hjac(mu_t_bar);
% Expected measurement by h(mu_t_bar)
z_t_bar_h = h(mu_t_bar);

% Return the NAN status of each sensor measurement and store it in a rowNum
% by 1 matrix
nanCondition = ~isnan(z_t);
numBeaconMeasures = sum(nanCondition(1:length(nanCondition) - 9));

% Get the dimension of measurement data to decide the number of iterations
[rowNum, colNum] = size(z_t);

% Declare an array that counts the number of ~NAN in each row
count = sum(nanCondition == 1, 'all');

if count == 0
    mu_t = mu_t_bar;
    sigma_t = sigma_t_bar;

else

    % Initialize mu_t and sigma_t
    mu_t = mu_t_bar;
    sigma_t = sigma_t_bar;

    k_dimension = count;
    index = 1;
    
    % Initialize H_t matrix
    H_t_updated = zeros(k_dimension, 3);
    % Initialize Q matrix
    Q_t_updated = zeros(k_dimension);

    z_t_bar_h_updated = [];
    z_t_updated = [];
    
    for j = 1 : rowNum
        if nanCondition(j, :)
            % Select the scaling whose corresponding sensorData is not NAN
            H_t_updated(index, :) = H_t(j, :);
            Q_t_updated(index, index) = Q_t(j, j);

            % We update only when:
            % 1. This is for GPS, where Q_t is a 3 by 3 matrix
            % 2. There is a small discrepency between measurement and
            % prediction. In this case, I set a boundary of +- 20%.
            if row == 3 || ... 
                j > numBeaconMeasures && (z_t_bar_h(j)/z_t(j) >= 0.82 && ...
                z_t_bar_h(j)/z_t(j) <=1.2 && ...
                ...% based on sensor_range
                z_t_bar_h(j) >= 0.175 && ...
                z_t_bar_h(j) <=10)

                z_t_bar_h_updated = [z_t_bar_h_updated; z_t_bar_h(j)];
                z_t_updated = [z_t_updated; z_t(j)];
            
            % By setting z_t_bar_h_updated (prediction to z_t (measurement) 
            % here, we are actually omitting this piece of data together,
            % since in mu_t update part, z_t_updated(j) -
            % z_t_bar_h_updated(j) = 0, which means that we don't change
            % mu_t here.
             else
                 z_t_bar_h_updated = [z_t_bar_h_updated; z_t(j)];
                 z_t_updated = [z_t_updated; z_t(j)];
             
             end
        index = index + 1;
        end
    end
    

    % Kalman gain
    K_t = sigma_t_bar * (H_t_updated).' * (H_t_updated * sigma_t_bar * (H_t_updated).' + Q_t_updated)^(-1);
    
    % Update
    if ~isempty(z_t_bar_h_updated) 
        mu_t = mu_t_bar + K_t * (z_t_updated - z_t_bar_h_updated);
        sigma_t = (eye(length(mu_t_1)) - K_t * H_t_updated) * sigma_t_bar;  
    end
end