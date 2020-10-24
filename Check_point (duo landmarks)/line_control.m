function [u, q] = line_control(img, u)% Calibrate the scale parameter and wheel track of the robot
addpath("../simulator/"); % Add the simulator to the MATLAB path.

    gray = rgb2gray(img);
    bin_img = ~imbinarize(gray, 0.2);

    % Find the centre of the line to follow
    imgB = bin_img(end-29:end, :); %get the last 30 rows
    [rows, cols] = find(imgB);
    
    % If you have reached the end of the line, you need to stop by breaking
    % the loop.
    if isempty(rows)||isempty(cols)
        % run an extra 3cm at the end to fully cover the line
        u = 0.1;
        [wl, wr] = inverse_kinematics(u, 0);
        pb.setVelocity([wl, wr], 0.3);
    end
    
    line_centre = (mean(cols)-400/2)/200;
    % Use the line centre to compute a velocity command
    u = u*(1 - line_centre^2);
    q = -pi*line_centre;

end

