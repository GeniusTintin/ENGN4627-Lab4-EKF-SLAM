% my EKF algorithm
% the script to perform my EKF SLAM
close all
clear
clc


addpath("../simulator/");
% start the robot with one simple landmark
% lm = [[1.5;1.5],[3.5;1.5]];
%  lm = [[1.5;1.5]];
pb = piBotSim("Floor_course.jpg");

% show the figure
figure
trail_axes = gca();
hold on
title("EKF-SLAM estiamtion")
xlim(trail_axes, [0 5]);
ylim(trail_axes, [0 5]);
axis(trail_axes, "manual");
grid on
grid minor
% initial pose
x = 1; y = 1; theta = 0;
pb.place([x;y],theta);
% timestamp
dt = 0.1;
% state vector xi
state_vector = [x;y;theta];
% covariance matrix (have the same length of the state vector)
Sigma = eye(3) * 0.01; 
% input noise covariance
R = eye(2) * 0.04;
% measurement noise covariance
Q = 0.02;
% tells which element of the measurement corresponds to which id
state_ids = []; 

% pose estimation
xiHatSave = [];

% direct integration result
Int = [x;y;theta];
integration = [];

while true
    
    img = pb.getCamera();
    
    % follow line
    [u, q, void] = line_control(img, 2.0, pb);
    if void
        break
    end
    [wl, wr] = inverse_kinematics(u, q);
    pb.setVelocity(wl, wr);

% ##########prediction##########
    % predict procedure:
    % Propagate the state
    % Integrate the state_vector given the velocity
    % Propagate the covariance
    % function [xiHat,Int,Sigma] = ekf_prediction(xiHat,Int,Sigma,R,dt,u,q)
    [state_vector,Int,Sigma] = ekf_prediction(state_vector,Int,Sigma,R,dt,u,q);
    
% ##########Deal with measurements##########
    % adding measurement procedure:
    % Take a measurement
    % Expand the state vector (if new landmark observed)
    % Expand the covariance matrix (if new landmark obversed)
    % function [xiHat, Sigma] = ekf_expansion(xiHat, Sigma, lms, ids, state_ids, R)
    [lms, ids] = pb.measureLandmarks();
    
    if ~isempty(ids)
        [state_vector, Sigma, state_ids] = ekf_expansion(state_vector, Sigma, lms, ids, state_ids, R);
        % ##########Update##########
        % Update procedure:
        % work out the measurement residual z - zHat
        % Get the Kalman gain
        % Apply to hte state and covariance
        % function [xiHat, Sigma] = ekf_update(xiHat, Sigma, Q, lms, ids, state_ids)
        [state_vector, Sigma] = ekf_update(state_vector, Sigma, Q, lms, ids, state_ids);
    end
    
    % normalise theta
    while state_vector(3) > 2 * pi
        state_vector(3) = state_vector(3) - 2 * pi;
    end
    while state_vector(3) < 0
        state_vector(3) = state_vector(3) + 2 * pi;
    end
    
    xiHatSave = [xiHatSave,state_vector(1:2)];
    integration = [integration, Int];
    
% ##########Plot##########
    
    plot(xiHatSave(1,:),xiHatSave(2,:),'b-','parent',trail_axes);
    plot(integration(1,:), integration(2,:),'r-','parent',trail_axes);
    for i = 1:numel(state_ids)
        scatter(state_vector(3+2*i-1),state_vector(3+2*i),6,'MarkerFaceColor',cmap(state_ids(i))...
            ,'MarkerEdgeColor','none','parent',trail_axes);
    end

end



