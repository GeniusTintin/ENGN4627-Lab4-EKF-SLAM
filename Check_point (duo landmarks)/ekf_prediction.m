function [xiHat,Int,Sigma] = ekf_prediction(xiHat, Int, Sigma, R, dt, u, q)
% function of the EKF prediction
% #inputs:
% xiHat: state vector of the robot
% Sigma: covariance matrix of the robot
% R: control input noise
% dt: timestamp
% u,q: control input
% #outputs:
% xiHat: predicted state at t+1
% Sigma: predicted covariance at t+1

    th = xiHat(3); % theta
    N = (numel(xiHat) - 3) / 2; % number of landmarks
    
    % compute At
    At = [1,0,-dt*u*sin(th);
          0,1,dt*u*cos(th);
          0,0,1];
    if N ~= 0
        At = [At,zeros(3,2*N);
              zeros(2*N,3),eye(2*N)];
    end
    % compute Bt
    Bt = dt * [cos(th), 0;
               sin(th), 0;
               0, 1];
    if N ~= 0
        Bt = [Bt;zeros(2*N,2)];
    end
    
    Sigma = At * Sigma * At' + Bt * R * Bt';
    xiHat(1:3) = xiHat(1:3) + dt *[cos(th) * u;
                                   sin(th) * u;
                                   q];
                               
    Int = Int + dt * [cos(th) * u;
                      sin(th) * u;
                      q];

end