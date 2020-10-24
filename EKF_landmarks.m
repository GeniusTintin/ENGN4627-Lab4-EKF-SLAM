%% run EKF
close all
clear 
clc
lm = [[1.5;1.5]];
addpath('../simulator');
pb = piBotSim('floor_circle.jpg',lm);
pb.place([2.5;3],0)

xiInt = [2.5;3.0;0];
xiHat = [2.5; 3; 0];
dt = 0.1;
Sigma = eye(3) * 1;
Rt = eye(2) * 0.1;
Qt = eye(2) * 0.1;

xiHatSaved = [];
integration = [];

for j = 1:600
    % line following
    img = pb.getCamera();
    [u,q] = line_control(img,0.2);
    [wl,wr] = inverse_kinematics(u,q);
    pb.setVelocity(wl,wr)
    
%     measure landmarks
    [z,ids] = pb.measureLandmarks();
    
    % EKF
    
    % 1. Compute At and Bt
    At = [1 0 -dt * u * sin(xiHat(3));
          0 1 dt * u * cos(xiHat(3));
          0 0 1];
    Bt = dt .* [cos(xiHat(3)) 0;
                sin(xiHat(3)) 0;
                0 1];  
    
    % 2. Covariance prediction
    Sigma = At * Sigma * At' + Bt * Rt * Bt';
    
    % 3. State Prediction
    xiHat = xiHat + dt * [u*cos(xiHat(3));
                          u*sin(xiHat(3));
                          q];
                      
    xiInt = xiInt + dt * [u * cos(xiInt(3));
                             u * sin(xiInt(3));
                             q];
                      
    % 4. Inpute measurements
    
    if ~isempty(z)
        rotation_matrix = [cos(xiHat(3)), -sin(xiHat(3));
                           sin(xiHat(3)), cos(xiHat(3))];
%         pose_matrix = [cos(xiHat(3)), -sin(xiHat(3));
%                        sin(xiHat(3)), cos(xiHat(3))];
        zHat = rotation_matrix' * (lm - xiHat(1:2));
        Ct = [-cos(xiHat(3)), -sin(xiHat(3)), -sin(xiHat(3))*(lm(1) - xiHat(1)) + cos(xiHat(3))*(lm(2) - xiHat(2))
              sin(xiHat(3)), -cos(xiHat(3)), -cos(xiHat(3))*(lm(1) - xiHat(1)) - sin(xiHat(3))*(lm(2) - xiHat(2))];
    
    
        % 5. Kalman Gain
        Kt = Sigma * Ct' * inv(Ct * Sigma * Ct' + Qt);

        % 6. Covariance Update
        Sigma = (eye(3) - Kt * Ct) * Sigma;

        % 7. State Update
%         z = convert2inertial(xiHat, z);
        xiHat = xiHat - Kt * (zHat - z);
        
        
    
    end
    
    % Normalise theta
%     while xiHat(3) > 2 * pi
%         xiHat(3) = xiHat(3) - 2 * pi;
%     end
%         
%     while xiHat(3) < 0
%         xiHat(3) = xiHat(3) + 2 * pi;
%     end
    
    xiHatSaved = [xiHatSaved, xiHat];
    integration = [integration, xiInt];
    figure(2)
    plot(xiHatSaved(1,:),xiHatSaved(2,:),'b-');
    xlim([0 5]);
    ylim([0 5]);
end

pb.saveTrail();

%% analyse result
load('robot_trail.mat', 'simRobotTrail');

% Removing NaNs at the start
simRobotTrailRows = ~any(isnan(simRobotTrail), 1);
simRobotTrail = simRobotTrail(:, simRobotTrailRows);

% Plot the robot trail
% plot(simRobotTrail(1,:), simRobotTrail(2,:))
% xlim([0, 5])
% ylim([0, 5])
% axis equal
% 
% hold on
% plot(xiHatSaved(1,:),xiHatSaved(2,:));
% plot(integration(1,:),integration(2,:));

hold off
time = 1:size(simRobotTrail,2);

subplot(3,1,1)
plot(time, simRobotTrail(1,:));
hold on
plot(time(2:end),    xiHatSaved(1,:));
plot(time(2:end),   integration(1,:));
hold off

subplot(3,1,2)
plot(time, simRobotTrail(2,:));
hold on
plot(time(2:end),    xiHatSaved(2,:));
plot(time(2:end),   integration(2,:));
hold off

subplot(3,1,3)
plot(time, simRobotTrail(3,:));
hold on
plot(time(2:end),    xiHatSaved(3,:));
plot(time(2:end),   integration(3,:));
hold off