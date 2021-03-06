% my EKF algorithm
% the script to perform my EKF SLAM


addpath("../simulator/");
% ground truth trajectory
% t = 0:0.01:pi/2;
% xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,...
%     3-0.5*sin(t),2+0.5*cos(t),2-0.5*sin(t)];
% yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
%     2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];

figure
trail_axes = gca();



% initial pose
x = 0; y = 0; theta = 0;
% pb.place([x;y],theta);
% timestamp
dt = 0.1;
% state vector xi
state_vector = [x;y;theta];
% covariance matrix (have the same length of the state vector)
Sigma = eye(3) * 0.1; 
% input noise covariance
R = eye(2) * [0.05,0;0,0.1];
% 0.04
% measurement noise covariance
Q = 0.05;
% 0.02
% tells which element of the measurement corresponds to which id
state_ids = []; 

% pose estimation
% xiHatSave = [];

% direct integration result
Int = [x;y;theta];
integration = [];

% values for evaluation
estimated_landmarks =[];
estimated_trajectory =[];

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
    
%     xiHatSave = [xiHatSave,state_vector(1:2)];
    estimated_trajectory = [estimated_trajectory, state_vector(1:3)];
    integration = [integration, Int];
    
% ##########Plot##########
    % plot the ground truth
%     plot(xTrace,yTrace,'g-','Parent',trail_axes); grid on; grid minor
    
    plot(estimated_trajectory(1,:),estimated_trajectory(2,:),'b-','parent',trail_axes);
    title("EKF-SLAM")
    grid on
    grid minor
    hold on
    % plot the integrated trajectory
%     plot(integration(1,:), integration(2,:),'r-','parent',trail_axes);
    
    for i = 1:numel(state_ids)
        color = state_ids(i);
        scatter(state_vector(3+2*i-1),state_vector(3+2*i),8,'MarkerFaceColor'...
            ,cmap(color),'MarkerEdgeColor','none','parent',trail_axes);
        
        text(state_vector(3+2*i-1)+0.1, state_vector(3+2*i)+0.1, cellstr(num2str(color))...
            , 'Color',cmap(color), 'FontSize', 12, 'Parent', trail_axes);
        
        e0 = plot_ellipses(state_vector(3+2*i-1:3+2*i),Sigma(3+2*i-1:3+2*i,3+2*i-1:3+2*i)...
            ,cmap(color),trail_axes);
%         e0.EdgeColor = cmap(color);
%         e0.LineWidth = 0.5;
        
    end
    
    % plot the visiable landmarks
    for i = 1:numel(ids)
        ilm = find(state_ids == ids(i));
        plot([state_vector(1),state_vector(3+2*ilm-1)], [state_vector(2),state_vector(3+2*ilm)]...
            ,'r-','Parent',trail_axes);
    end
    
    % plot ellipse
    e1 = plot_ellipses(state_vector(1:2),Sigma(1:2,1:2),'b',trail_axes);
    xlim(trail_axes, [-5 5]);
    ylim(trail_axes, [-5 5]);
    axis(trail_axes, 'manual');
    hold off
end

% compute the estimated landmarks matrix
for i = 1:(numel(state_vector) - 3)/2
    index = find(state_ids == i);
    estimated_landmarks = [estimated_landmarks, state_vector(3+2*index-1:3+2*index)];
end

legend('Ground Truth','EKF-SLAM','Direct Integration')
legend('Ground Truth','EKF-SLAM','Direct Integration')
% pb.saveTrail();
% 
% RMSE_calcultion(estimated_landmarks, estimated_trajectory, landmarks)
