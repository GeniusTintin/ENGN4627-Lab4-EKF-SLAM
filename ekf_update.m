function [xiHat, Sigma] = ekf_update(xiHat, Sigma, Q, lms, ids, state_ids)
% function of update after measurement of EKF
% #inputs:
% xiHat: predicted state at t+1
% Sigma: predicted covariance at t+1
% Q: covariance of measurement noise
% lms: measured landmark position
% ids: measured landmark ids corresponds with position index
% state_ids: currently stored ids
% #outputs:
% xiHat: updated state vector at t+1
% Sigma: updated state covariance matrix at t+1
    
    N = numel(ids);
    length = numel(state_ids); %number of landmarks in state
    Q = eye(2*N) * Q;
    C = []; % intialise C matrix
    z_err = []; % error matrix
    
    for i = 1:N
       z_i = lms(:,i);
       id = ids(i);
       % find the position of the measurement in the array
       index = find(state_ids == id);
       
       % compute the C matrix
       lm = xiHat(3+index*2-1: 3+index*2); % landmark position in state_vector 
       lx = lm(1); ly = lm(2);
       th = xiHat(3); % theta
       % first three non-zero terms
       C_0 = [-cos(th),-sin(th),sin(th)*(xiHat(1)-lx)-cos(th)*(xiHat(2)-ly);
              sin(th),-cos(th),cos(th)*(xiHat(1)-lx)+sin(th)*(xiHat(2)-ly)];
       % non-zero terms associate with landmark index
       C_lm = [cos(th),sin(th);
               -sin(th),cos(th)];
       % ith C matrix
       Ci = [C_0,zeros(2,2*(index-1)),C_lm,zeros(2,2*(length-index))];
       % overall constructed C matrix
       C = [C;Ci];
       
       % Compute z_err
       rotation_matrix = [cos(th), -sin(th);
                          sin(th), cos(th)];
       zHat = rotation_matrix' * (lm - xiHat(1:2));
       z_err = [z_err; zHat - z_i];
    end
       % Kalman gain
       
       K = Sigma * C' /(C * Sigma * C' + Q);
       % Update Sigma
       Sigma = (eye(numel(xiHat)) - K * C) * Sigma;
       xiHat = xiHat - K * z_err;
       
end