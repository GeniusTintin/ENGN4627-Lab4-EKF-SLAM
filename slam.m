% Always begin by using addpath
addpath("../simulator")

% For testing, we can tell the simulator where we want to place our
% landmarks. Let's try a grid formation
[lmx,lmy] = meshgrid(0.5:(4/3):4.5);
landmarks = [lmx(:)'; lmy(:)'];

% Now, we can start the simulator with these landmarks.
% You can also try set your own landmarks, or, if you leave it blank, the
% simulator will generate landmarks randomly.
pb = piBotSim("floor_spiral.jpg", landmarks);
pb.showLandmarks(true); % Use this to turn on/off showing the landmarks.
pb.showViewRange(true); % Use this to turn on/off showing the pibot view.

% Your final solution should not use any of the above.
% We will set up the simulator for you and choose the landmark positions.
% Your algorithm will not know where the landmarks are initially, and
% will have to estimate their positions based on the measurements.

% Place your robot at the centre of the room
pb.place([2;2],0);

% Which landmarks can we measure? Try the measurement function
[landmarkPoints, landmarkIds] = pb.measureLandmarks()

% Now you can use these measurements in your slam system!
% You will also need to use your known input velocity to the robot.
% I strongly suggest you try some simple examples before you try to follow
% a line, e.g. drive in a straight line for a few seconds.
% This will also help to evaluate your solution!