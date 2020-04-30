close all

path   = lemOfGerono(linspace(0,1),basisParams);
pathAz = atan2d(path(:,2),path(:,1));
pathEl = 90 - acosd(path(:,3)./sqrt(sum(path.^2,2)));

wyPtPosVecs     = lemOfGerono(Sw,basisParams);
wyPtAzimuth     = atan2(wyPtPosVecs(:,2),wyPtPosVecs(:,1));
wyPtElevation   = pi/2-acos(wyPtPosVecs(:,3)./sqrt(sum(wyPtPosVecs.^2,2)));

switch getenv('computername')
    case 'vermillionlab1'
    h.pathFig = figure('Name','Summary','Position',[1921          41        1920         963]);
    case 'DESKTOP-8QI3EO0'
    h.pathFig  = figure('Name','Path','Position',[-0.9995    0.5194    0.4990    0.4019]);
    h.spedFig  = figure('Name','Speed','Position',[-0.9995    0.0556    0.4990    0.3750]);
    h.perfFig  = figure('Name','Performance','Position',[-0.4995    0.0380    0.4990    0.8833]);
end

%% Path Shape Figure
h.fltPathsAx  = axes(h.pathFig,'NextPlot','add','XGrid','on','YGrid','on');
h.nominalPath  = plot(pathAz,pathEl,...
    'LineWidth',2,'Color','r','LineStyle',':','DisplayName','Nominal Path',...
    'Parent',h.fltPathsAx);
h.wypts = scatter(wyPtAzimuth*180/pi,wyPtElevation*180/pi,'Marker','x',...
    'DisplayName','Waypoint','CData',[0 0 0],'SizeData',72,...
    'Parent',h.fltPathsAx);
h.pathPrev  = plot(nan,nan,'Color','k','DisplayName',...
    'Previous, Actual','LineStyle','-',...
    'Parent',h.fltPathsAx);
h.pathNext  = plot(nan,nan,'Color',[0 0.75 0],'DisplayName',...
    'Next, Predicted','LineStyle','-',...
    'Parent',h.fltPathsAx);
xlabel('Azimuth, [deg]')
ylabel('Elevation, [deg]')
h.fltPathLegend = legend;
h.fltPathTitle = title('');

%% Distance-Time-Velocity Figure
% Distance axes
h.distAx     = subplot(1,3,1,'Parent',h.spedFig,'NextPlot','add','XGrid','on','YGrid','on');
h.distAx.Position(1) = h.distAx.Position(1)*0.5;
h.distAx.Position(2) = h.distAx.Position(2)*1.4;
h.distAx.Position(4) = h.distAx.Position(4)*0.8;
axes(h.distAx);
yyaxis left
h.distPrev    = scatter(nan,nan);
yyaxis right
h.distNext   = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('$j$')
title({'Distance,','(Act. \& Pred.) [m]'})

% Average speeds
h.speedAx    = subplot(1,3,2,'Parent',h.spedFig,'NextPlot','add','XGrid','on','YGrid','on');
h.speedAx.Position(2) = h.speedAx.Position(2)*1.4;
h.speedAx.Position(4) = h.speedAx.Position(4)*0.8;
axes(h.speedAx);
yyaxis left
h.vAvgPrevTime = scatter(nan,nan);
yyaxis right
h.vAvgPrevPath = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('$j$')
title({'Average Speed,','($v_{avg}^{time}$ \& $v_{avg}^{path}$) [s]'})

% Time
h.timeAx  = subplot(1,3,3,'Parent',h.spedFig,'NextPlot','add','XGrid','on','YGrid','on');
h.timeAx.Position(1) = h.timeAx.Position(1)*1.1;
h.timeAx.Position(2) = h.timeAx.Position(2)*1.4;
h.timeAx.Position(4) = h.timeAx.Position(4)*0.8;
axes(h.timeAx)
h.timePrev = scatter(nan,nan);
title('Time, [s]')
xlim([1 numIters]);
xlabel('$j$')

%% Plot performance things
h.perfIndxAx = subplot(4,2,1,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.perfIndxAx)
yyaxis left
h.perfIndxPrev = scatter(nan,nan);
yyaxis right
h.perfIndxNext = scatter(nan,nan,'Marker','x');
xlabel('$j$')
title('Performance Index, $J_j$')

h.SxAx  = subplot(4,2,2,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.SxAx)
yyaxis left
h.JsxPrev = scatter(nan,nan);
yyaxis right
h.JsxNext = scatter(nan,nan,'Marker','x');
title('$S_x$ Term')

h.JuAx  = subplot(4,2,3,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.JuAx)
yyaxis left
h.JuPrev = scatter(nan,nan);
yyaxis right
h.JuNext = scatter(nan,nan,'Marker','x');
title('$Q_u$ Term')

h.JduAx = subplot(4,2,4,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.JduAx)
yyaxis left
h.JduPrev = scatter(nan,nan);
yyaxis right
h.JduNext = scatter(nan,nan,'Marker','x');
title('$Q_{\delta u}$ Term')

h.JxAx = subplot(4,2,5,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.JxAx)
yyaxis left
h.JxPrev = scatter(nan,nan);
yyaxis right
h.JxNext = scatter(nan,nan,'Marker','x');
title('$Q_{x}$ Term')

h.JdxAx = subplot(4,2,6,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.JdxAx)
yyaxis left
h.JdxPrev = scatter(nan,nan);
yyaxis right
h.JdxNext = scatter(nan,nan,'Marker','x');
title('$Q_{\delta x}$ Term')

h.JeAx  = subplot(4,2,7,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.JeAx)
yyaxis left
h.JePrev = scatter(nan,nan);
yyaxis right
h.JeNext = scatter(nan,nan,'Marker','x');
title('$Q_e$ Term')

h.JdeAx = subplot(4,2,8,'NextPlot','add','XGrid','on','YGrid','on','Parent',h.perfFig);
axes(h.JdeAx)
yyaxis left
h.JdePrev = scatter(nan,nan);
yyaxis right
h.JdeNext = scatter(nan,nan,'Marker','x');
title('$Q_{\delta e}$ Term')

set(findall(gcf,'Type','Axes'),'XLim',[1 numIters])


%%
set(findall(findobj('Type', 'figure'),'Type','Axes'),'FontSize',16)

