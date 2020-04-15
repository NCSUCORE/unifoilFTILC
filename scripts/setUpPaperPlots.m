% calculate some things that we'll need
wyptPosVecs = lemOfGerono(Sw,basisParams);
wyptAz = atan2d(wyptPosVecs(:,2),wyptPosVecs(:,1));
wyptEl = 90-acosd(wyptPosVecs(:,3)./sqrt(sum(wyptPosVecs.^2,2)));

% Shape of the paths
h.pathShapeFig = figure('Name','PathShape',...
    'Position',[-0.9995    0.5194    0.4990    0.4019]);
h.pathShapeAx  = axes('NextPlot','add',...
    'XGrid','on','YGrid','on',...
    'DataAspectRatio',[1 1 1]);
h.nominalPathPlot = plot(pathAz*180/pi,pathEl*180/pi,....
    'DisplayName','Nominal Path','LineWidth',2,'LineStyle',':',...
    'Color',0.5*[1 1 1]);
h.achievedPath1Plot = plot(nan,nan,....
    'DisplayName','Flight Path, $j = 1$','LineWidth',1.5,'LineStyle','--',...
    'Color',0*[1 1 1]);
h.achievedPathEndPlot = plot(nan,nan,....
    'DisplayName','Flight Path, $j =$ end','LineWidth',1.5,'LineStyle','-',...
    'Color',0*[1 1 1]);
h.wayptLocScat = scatter(wyptAz,wyptEl,...
    'Marker','x','SizeData',50,'LineWidth',1.5,...
    'CData',[0 0 0],'DisplayName','Waypoint Location');
xlim(0.5*basisParams(1)*(180/pi)*[-1 1]+[-2 2])
ylim(basisParams(4)*(180/pi)+0.5*basisParams(2)*(180/pi)*[-1 1]+[-2 2])
xlabel('Azimuth, $\phi$, [deg]')
ylabel('Elevation, $\theta$, [deg]')
title('Path Shape Summary')
h.pathShapeLeg = legend;
h.pathShapeLeg.Position = [0.6899 0.6479 0.2807 0.2781];

h.performanceFig = figure('Name','PerformanceIndex',...
    'Position',[-0.4995    0.5259    0.4990    0.3954]);
h.performanceAx = axes('NextPlot','add',...
    'XGrid','on','YGrid','on');
h.performancePlot = scatter(nan,nan,...
    'Marker','o','MarkerFaceColor','flat','CData',[0 0 0],'SizeData',50);
xlim([0 numIters])
xlabel('Iteration Number, $j$')
ylabel('Performance Index, $J$')
title('Performance Vs Iteration')

h.waypointTrackingFig = figure('Name','WaypointTracking',...
    'Position',[ -0.9995    0.0380    0.4990    0.4019]);
h.waypointTrackingAx = axes('NextPlot','add',...
    'XGrid','on','YGrid','on');
h.waypointTrackingPlot = scatter(nan,nan,...
    'Marker','o','MarkerFaceColor','flat','CData',[0 0 0],'SizeData',50);
xlim([0 numIters])
xlabel('Iteration Number, $j$')
ylabel({'Waypoint Tracking','Penalty, $e_j^TQ_ue_j$'})
title('Waypoint Tracking Vs Iteration')

h.durationFig = figure('Name','Duration',...
    'Position',[0.0005    0.1056    0.4375    0.8157]);

h.economicTermAx = subplot(3,1,1,'NextPlot','add',...
    'XGrid','on','YGrid','on');
h.economicTermScat = scatter(nan,nan,...
    'Marker','o','MarkerFaceColor','flat','CData',[0 0 0],'SizeData',50);
ylabel({'Speed Incentive','Term, $S_x x_j$'})

h.meanSpeedAx = subplot(3,1,2,'NextPlot','add',...
    'XGrid','on','YGrid','on');
h.meanSpeedScat = scatter(nan,nan,...
    'Marker','o','MarkerFaceColor','flat','CData',[0 0 0],'SizeData',50);
ylabel({'Mean ','Speed, [m/s]'})

h.iterDurAx = subplot(3,1,3,'NextPlot','add',...
    'XGrid','on','YGrid','on');
h.iterDurScat = scatter(nan,nan,...
    'Marker','o','MarkerFaceColor','flat','CData',[0 0 0],'SizeData',50);
xlabel('Iteration Number, $j$')
ylabel({'Iteration','Duration, [s]'})

linkaxes([h.economicTermAx h.meanSpeedAx h.iterDurAx],'x')
xlim([0 numIters])

set(findobj('Type','Axes'),'FontSize',24)
h.pathShapeLeg.FontSize = 16
