nRow = 3;
nCol = 4;

path   = lemOfGerono(linspace(0,1),basisParams);
pathAz = atan2d(path(:,2),path(:,1));
pathEl = 90 - acosd(path(:,3)./sqrt(sum(path.^2,2)));

if strcmpi(getenv('computername'),'vermillionlab1')
    h.fig1 = figure('Name','Summary','Position',[1921          41        1920         963]);
else
    h.fig1 = figure('Name','Summary','Position',[-1.0000    0.0370    1.0000    0.8917]);
end

%% Path Shape
h.fltPathsAx  = subplot(nRow,nCol,[1 2],'NextPlot','add','XGrid','on','YGrid','on');
h.targetPath  = plot(pathAz,pathEl,...
    'LineWidth',2,'Color','r','LineStyle',':','DisplayName','Target Path');
h.wypts = scatter(wyPtAzimuth*180/pi,wyPtElevation*180/pi,'Marker','x',...
    'DisplayName','Waypoint','CData',[0 0 0],'SizeData',72);
h.actualPath_iMin1  = plot(nan,nan,'Color','k','DisplayName',...
    '$i-1$ Actual','LineStyle','-');
h.actualPath_i      = plot(nan,nan,'Color',[0 0.75 0],'DisplayName',...
    '$i$, Actual','LineStyle','-');
h.predicPath_iPls1  = plot(nan,nan,'Color','b','DisplayName',...
    '$i+1$, Predicted','LineStyle','-');
xlabel('Azimuth, [deg]')
ylabel('Elevation, [deg]')
h.legend1 = legend;
h.title = title('');

%% Control inputs
% Wing control input
h.wingCtrlAx = subplot(nRow,nCol,nCol+1,'NextPlot','add','XGrid','on','YGrid','on');
h.uw        = plot(nan,nan,'Color','k','DisplayName','$u_{w}^{prev}$','LineStyle','-');
h.uwfb      = plot(nan,nan,'Color','b','DisplayName','$u_{w,fb}^{prev}$','LineStyle','-');
h.uwilcPrev  = plot(nan,nan,'Color','r','DisplayName','$u_{w,ff}^{prev}$','LineStyle','-');
h.uwilcNext  = plot(nan,nan,'Color','g','DisplayName','$u_{w,ff}^{next}$','LineStyle','-');
xlim([0 1])
xlabel('Path Var')
ylabel('[deg]')
h.uwLegend = legend;

% Rudder control input
h.ruddCtrlAx = subplot(nRow,nCol,nCol+2,'NextPlot','add','XGrid','on','YGrid','on');
yyaxis left
h.ur        = plot(nan,nan,'Color','k','DisplayName','$u_{r}^{prev}$','LineStyle','-');
h.urfb      = plot(nan,nan,'Color','b','DisplayName','$u_{r,fb}^{prev}$','LineStyle','-');
yyaxis right
h.urilcPrev  = plot(nan,nan,'Color','r','DisplayName','$u_{r,ff}^{prev}$','LineStyle','-');
h.urilcNext  = plot(nan,nan,'Color','g','DisplayName','$u_{r,ff}^{next}$','LineStyle','-');
xlim([0 1])
xlabel('Path Var')
ylabel('[deg]')
h.urLegend = legend;

% Distance travelled
h.distAx    = subplot(nRow,nCol,2*nCol+1,'NextPlot','add','XGrid','on','YGrid','on');
h.distAct    = scatter(nan,nan);
h.distPred   = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('Path Var')
ylabel('Dist. [m]')

% Average speed
h.distAx    = subplot(nRow,nCol,2*nCol+2,'NextPlot','add','XGrid','on','YGrid','on');
yyaxis left
h.vAvgTime = scatter(nan,nan);
ylabel('$v_{avg}^{time}$')
yyaxis right
h.vAvgPath = scatter(nan,nan);
ylabel('$v_{avg}^{path}$')
xlim([1 numIters]);
xlabel('Path Var')


%% Plot performance things
% Overall performance index
h.perfIndxAx    = subplot(nRow,nCol,nCol-1,'NextPlot','add','XGrid','on','YGrid','on');
h.perfIndxAct   = scatter(nan,nan);
h.perfIndxPred  = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('Perf. Indx.')
grid on;hold on

% Total Simulation Time
h.iterTimeAx    = subplot(nRow,nCol,nCol,'NextPlot','add','XGrid','on','YGrid','on');
h.simTime       = scatter(nan,nan);
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$T_F$ [s]')

% Linear state term
h.lxAx      = subplot(nRow,nCol,2*nCol-1,'NextPlot','add','XGrid','on','YGrid','on');
h.lxAct     = scatter(nan,nan);
h.lxPred    = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('Lin. State Term')

% Quadratic error term
h.qxAx      = subplot(nRow,nCol,nRow*nCol-1,'NextPlot','add','XGrid','on','YGrid','on');
h.qxAct     = scatter(nan,nan);
h.qxPred    = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('Quad. Error Term')

% Linear state term
h.luAx      = subplot(nRow,nCol,2*nCol,'NextPlot','add','XGrid','on','YGrid','on');
h.luAct     = scatter(nan,nan);
h.luPred    = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('Lin. Input Term')

% Quadratic state term
h.quAx      = subplot(nRow,nCol,nRow*nCol,'NextPlot','add','XGrid','on','YGrid','on');
h.quAct     = scatter(nan,nan);
h.quPred    = scatter(nan,nan,'Marker','x');
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel({'Quad. Input','Dev. Term'})

%% Path shape inset
% h.zoomAx = axes(gcf,'Units','Normalize','Position',[0.4104    0.5944    0.0990    0.1564]);
% box on
% h.targetPathZoom = plot(pathAz,pathEl,...
%     'LineWidth',2,'Color','r','LineStyle',':','DisplayName','Target Path');
% grid on;hold on
% h.actualPath_iMin1Zm  = plot(nan,nan,'Color','k','DisplayName',...
%     '$i-1$, Actual','LineStyle','-');
% h.actualPath_iZm      = plot(nan,nan,'Color',[0 0.75 0],'DisplayName',...
%     '$i$, Actual','LineStyle','-');
% h.predicPath_iPls1Zm  = plot(nan,nan,'Color','b','DisplayName',...
%     '$i+1$, Predicted','LineStyle','-');
% xlim([pathAz(1)-2 pathAz(1)+2])
% ylim([pathEl(1)-2 pathEl(1)+2])

%% Set all font sizes
set(findall(gcf,'Type','Axes'),'FontSize',16)

%% Created predicted and actual state sequences
h.fig2 = figure('Position',[ 0.0005    0.0380    0.4990    0.8833],'Name','State Sequences');
h.phiAx = subplot(5,1,1,'NextPlot','add','XGrid','on','YGrid','on');
yyaxis left
h.phiPrev = plot(nan,nan,'Color','b','LineStyle','-','DisplayName','$\phi(s)_{i+1}^{prev}$');
h.phiPred = plot(nan,nan,'Color','r','LineStyle','-','DisplayName','$\phi(s)_i^{pred}$');
ylabel('$\phi$')
yyaxis right
h.dphiPred = plot(nan,nan,'Color','k','LineStyle','-','DisplayName','$\delta\phi(s)_i^{pred}$');
legend
xlabel('s')
ylabel('$\delta \phi$')

h.thetaAx = subplot(5,1,2,'NextPlot','add','XGrid','on','YGrid','on');
yyaxis left
h.thetaPrev = plot(nan,nan,'Color','b','LineStyle','-','DisplayName','$\theta(s)_{i+1}^{prev}$');
ylabel('$\theta$')
h.thetaPred = plot(nan,nan,'Color','r','LineStyle','-','DisplayName','$\theta(s)_i^{pred}$');
yyaxis right
h.dthetaPred = plot(nan,nan,'Color','k','LineStyle','-','DisplayName','$\delta\theta(s)_i^{pred}$');
legend
xlabel('s')
ylabel('$\delta\theta$')

h.vAx = subplot(5,1,3,'NextPlot','add','XGrid','on','YGrid','on');
yyaxis left
h.vPrev = plot(nan,nan,'Color','b','LineStyle','-','DisplayName','$v(s)_{i+1}^{prev}$');
h.vPred = plot(nan,nan,'Color','r','LineStyle','-','DisplayName','$v(s)_i^{pred}$');
ylabel('$v$')
yyaxis right
h.dvPred = plot(nan,nan,'Color','k','LineStyle','-','DisplayName','$\delta v(s)_i^{pred}$');
legend
xlabel('s')
ylabel('$\delta v$')

h.psiAx = subplot(5,1,4,'NextPlot','add','XGrid','on','YGrid','on');
yyaxis left
h.psiPrev = plot(nan,nan,'Color','b','LineStyle','-','DisplayName','$\psi(s)_{i+1}^{prev}$');
h.psiPred = plot(nan,nan,'Color','r','LineStyle','-','DisplayName','$\psi(s)_i^{pred}$');
ylabel('$\psi$')
yyaxis right
h.dpsiPred = plot(nan,nan,'Color','k','LineStyle','-','DisplayName','$\delta\psi(s)_i^{pred}$');
legend
xlabel('s')
ylabel('$\delta \psi$')

h.omegaAx = subplot(5,1,5,'NextPlot','add','XGrid','on','YGrid','on');
yyaxis left
h.omegaPrev = plot(nan,nan,'Color','b','LineStyle','-','DisplayName','$\omega(s)_{i+1}^{prev}$');
h.omegaPred = plot(nan,nan,'Color','r','LineStyle','-','DisplayName','$\omega(s)_i^{pred}$');
ylabel('$\omega$')
yyaxis right
h.domegaPred = plot(nan,nan,'Color','k','LineStyle','-','DisplayName','$\delta\omega(s)_i^{pred}$');
legend
xlabel('s')
ylabel('$\delta \omega$')

set(findall(gcf,'Type','Axes'),'FontSize',16)

h.fig3 = figure('Position',[ 0.0005    0.0380    0.4990    0.883],'Name','Wing Summary');

h.uwAx  = subplot(3,1,1);
h.uw    = plot(nan,nan,'DisplayName','$u_{w}$');
grid on
hold on
h.uwOpt = plot(nan,nan,'DisplayName','$u_{w}^*$');
xlabel('Path Var.')
ylabel('[deg]')
legend 

h.gammaAx   = subplot(3,1,2);
h.gammaW    = plot(nan,nan,'DisplayName','$\gamma_{w}$');
grid on
hold on
h.gammaR    = plot(nan,nan,'DisplayName','$\gamma_{r}$');
xlabel('Path Var.')
ylabel('[deg]')
legend 

h.wingAlphaAx   = subplot(3,1,3);
h.wingAlpha     = plot(nan,nan,'DisplayName','$\alpha_{w}$');
grid on
hold on
h.wingAlphaOpt  = plot(nan,nan,'DisplayName','$\alpha_{w}^*$');
xlim([0 1])
xlabel('Path Var.')
ylabel('[deg]')
legend 

set(findall(gcf,'Type','Axes'),'FontSize',16)

%%
h.fig4 = figure('Position',[ 0.0005    0.0380    0.4990    0.883],'Name','Wing Summary');

h.fxAx      = subplot(3,1,1,'NextPlot','Add','XGrid','on','YGrid','on');
h.fxAct     = plot(nan,nan,'DisplayName','$F_{x,j}^{Act}$');
h.fxPred    = plot(nan,nan,'DisplayName','$F_{x,j+1}^{Pred}$');
legend

xlabel('Path Var')
ylabel('[N]')

set(findall(gcf,'Type','Axes'),'FontSize',16)
