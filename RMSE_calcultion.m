function RMSE_calcultion(estimated_landmarks, estimated_trajectory, landmark_positions)
% estimate the RMSE and ARMSE of the robot_trail and estimated landmarks
% #inputs:
% estimated_landmarks: estimated landmarks position
% estimated_trajectory: estimated robrot trajectory over time
    assert(all(size(estimated_landmarks) == size(landmark_positions)), "Estimated and true landmarks are not the same size.");
    assert(size(estimated_trajectory,1) == 3, "Estimated trajectory does not have 3 rows.");
    % Load the true robot trajectory
    load("robot_trail.mat", "simRobotTrail");
    simRobotTrailRows = ~any(isnan(simRobotTrail), 1);
    simRobotTrail = simRobotTrail(:, simRobotTrailRows);
    
    % Match the trajectories. Note they are assumed to start at the same time.
    % The estimated trajectory MUST start as soon as the robot begins simulation.
    trajectory_length = min(size(simRobotTrail, 2), size(estimated_trajectory,2));
    simRobotTrail = simRobotTrail(:,1:trajectory_length);
    estimated_trajectory = estimated_trajectory(:,1:trajectory_length);
    
    % Compute the trajectory error
    estimated_positions = estimated_trajectory(1:2,:);
    true_positions = simRobotTrail(1:2,:);
    trajectory_armse = compute_armse(true_positions, estimated_positions);
    
    % Compute the landmark error
    landmarks_armse = compute_armse(landmark_positions, estimated_landmarks);
    
    % Print the results
    disp("Trajectory RMSE:" + num2str(trajectory_armse));
    disp("Landmark RMSE:" + num2str(landmarks_armse));
end

%% inner function for computing the armse
function armse = compute_armse(points1, points2)
% Compute the aligned RMSE between two matched sets of 2D points
n = size(points1,2);
assert(all(size(points1)==[2,n]));
assert(all(size(points2)==[2,n]));

mu1 = mean(points1,2);
mu2 = mean(points2,2);

Sig = 1/n * (points2-mu2) * (points1-mu1)';

[U,~,V] = svd(Sig);
A = eye(2);
if det(Sig) < 0
    A(2,2) = -1;
end

R = V * A * U';
x = mu1 - R * mu2;

points1_aligned = R' * (points1 - x);

armse = real(sqrt(1/n * sum((points1_aligned - points2).^2, 'all')));

end