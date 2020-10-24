function inertial_lm = convert2inertial(pose, lm)
% convert the landmark measurement to the inertial frame
% #inputs:
% pose: robot pose
% lm: landmark position (x;y) in BFF
% #output:
% inertial_lm: landmark position (x;y) in inertial frame
    th = pose(3); %theta
    R = [cos(th),-sin(th);
         sin(th),cos(th)];
    t = pose(1:2,:);
    inertial_lm = R * lm + t;
end