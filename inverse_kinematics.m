function [wl, wr] = inverse_kinematics(u, q)
% Compute the left and right wheel velocities (wl, wr) required for the robot
% to achieve a forward speed u and angular speed q.
scale_parameter = 5.33e-3;
wheel_base = 0.156;

wl = (u - wheel_base/2 * q) / scale_parameter;
wr = (u + wheel_base/2 * q) / scale_parameter;

end