function e = plot_ellipses(centre, cov, color, axh)
% this function plot the error ellipse given the mean and covariance
% #input:
% centre: the [x; y] coordinate of the centre
% rads: [horizontal; vertical] 'radius' of the ellipse
% axh: is the axis handle (if empty then gca will be used)
% #output:
% e: object handle to the plotted ellipse
if nargin < 4 || isempty(axh)
    axh = gca();
end

[V,D] = eig(cov);
ang = linspace(0,2*pi);
ellipse = 0.5*V*sqrt(D)*[cos(ang);sin(ang)] + centre;
% e = rectangle(axh,'position' ,[llc(:).', wh(:).'],'Curvature',[1,1]);
e = plot(ellipse(1,:),ellipse(2,:),'Parent',axh,'Color',color);
end