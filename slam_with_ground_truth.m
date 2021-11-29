% my EKF algorithm
% the script to perform my EKF SLAM
close all
clear
clc

addpath("../simulator/");
% ground truth trajectory
t = 0:0.01:pi/2;
xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,...
    3-0.5*sin(t),2+0.5*cos(t),2-0.5*sin(t)];
yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
    2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];

figure
trail_axes = gca();

% plot the ground truth
plot(xTrace,yTrace,'g-','Parent',trail_axes); grid on; grid minor
hold on
title("EKF-SLAM vs Direct Integration")
xlim(trail_axes,[0,5]);
ylim(trail_axes,[0,5]);
axis(trail_axes,'manual');

% start the robot with one simple landmark
% lm = [[1.5;1.5],[3.5;1.5]];
%  lm = [[1.5;1.5]];
% grid landmarks
% [lmx,lmy] = meshgrid(0.5:(4/3):4.5);
% landmarks = [lmx(:)'; lmy(:)'];

% pb = piBotSim("Floor_course.jpg",landmarks);
pb = piBotSim("floor_course.jpg");

% initial pose
x = 1; y = 1; theta = 0;
pb.place([x;y],theta);
% timestamp
dt = 0.1;
dt_acc = 0;
% state vector xi
state_vector = [x;y;theta];
% covariance matrix (have the same length of the state vector)
Sigma = eye(3) * 0.1; 
% input noise covariance
R = eye(2) * [0.04,0;0,0.08];
% 0.04
% measurement noise covariance
Q = 0.05;
% 0.02
% tells which element of the measurement corresponds to which id
state_ids = []; 

% direct integration result
Int = [x;y;theta];
integration = [];

estimated_landmarks = [];
estimated_trajectory = [];

while true
    
    img = pb.getCamera();
    
    % follow line
    [u, q, void] = line_control(img, 2.0, pb);
    if void
        break
    end
    [wl, wr] = inverse_kinematics(u, q);
    pb.setVelocity(wl, wr);
    dt_acc = dt_acc + 1;
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
    
    if ~isempty(ids) && ~any(isnan(lms(1,:)))
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
    
    estimated_trajectory = [estimated_trajectory,state_vector(1:3)];
    integration = [integration, Int];
    
% ##########Plot##########
    plot(estimated_trajectory(1,:),estimated_trajectory(2,:),'b-','parent',trail_axes);
    plot(integration(1,:), integration(2,:),'r-','parent',trail_axes);
    
    for i = 1:numel(state_ids)
        color = state_ids(i);
        scatter(state_vector(3+2*i-1),state_vector(3+2*i),6,'MarkerFaceColor'...
            ,cmap(color),'MarkerEdgeColor','none','parent',trail_axes);
%         hold on
        
    end
%     hold off
    % plot ellipse
    if mod(dt_acc, 20) == 0 
        e_state = plot_ellipses(state_vector(1:2),Sigma(1:2,1:2),'b',trail_axes);
    end
%     xlim(trail_axes, [0 5]);
%     ylim(trail_axes, [0 5]);
%     axis(trail_axes, 'manual');
%     hold off
end

% compute the estimated landmarks matrix
for i = 1:(numel(state_vector) - 3)/2
    index = find(state_ids == i);
    estimated_landmarks = [estimated_landmarks, state_vector(3+2*index-1:3+2*index)];
end

for i = 1:numel(state_ids)
    color = state_ids(i);
    ei = plot_ellipses(state_vector(3+2*i-1:3+2*i),Sigma(3+2*i-1:3+2*i,3+2*i-1:3+2*i)...
        , cmap(color), trail_axes);
    text(state_vector(3+2*i-1)+0.1, state_vector(3+2*i)+0.1, cellstr(num2str(color))...
        , 'Color',cmap(color), 'FontSize', 12, 'parent', trail_axes);
end
plot(xTrace,yTrace,'g-','Parent',trail_axes)
legend('Ground Truth','EKF-SLAM','Direct Integration')
legend('Ground Truth','EKF-SLAM','Direct Integration')

pb.saveTrail();

RMSE_calcultion(estimated_landmarks, estimated_trajectory, landmarks)


