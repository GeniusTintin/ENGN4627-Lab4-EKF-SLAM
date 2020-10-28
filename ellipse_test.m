clc
d1 = randn(2,100) + 1;

mean1 = mean(d1,2);
std1 = std(d1,[],2);

figure
frame_axes = gca();
plot(d1(1,:), d1(2,:),'r.','parent',frame_axes);
hold on

e = plot_ellipses(mean1, std1, frame_axes);
e.EdgeColor = 'r';
e.LineWidth = 1;

%%

clc
Sigma = [0.8,0.3;0.3,0.4];
[V,D] = eig(Sigma);
ang = linspace(0,2*pi);
ellipse = V*sqrt(D)*[cos(ang);sin(ang)];
plot(ellipse(1,:),ellipse(2,:))

%%
% Make an example plot
figure;
p = peaks; % Get 'peaks' data which comes with Matlab
[nY, nX] = size(peaks);
x = 1:nX; y = 1:nY;
h1 = pcolor(x, y, peaks); % Want pcolor object to remain
shading flat;
hold on; % Applies to all plotted objects
title('Wish we could hold pcolor but not lines')
% Plot several lines over top of pcolor plot
nLine = 10; % Number of lines to plot
for iLine = 1:(nLine)
    h2 = plot(rand(1,10)*nX, rand(1,10)*nY, 'ko-');
    % Want 'old' h2 to disappear when 'new' h2 plotted 
    pause(1); % Allow user to see lines being plotted
end