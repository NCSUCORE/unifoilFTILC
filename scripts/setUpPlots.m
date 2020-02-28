%% figure 1
nRow = 3;
nCol = 4;
path   = lemOfGerono(linspace(0,1),basisParams);
pathAz = atan2d(path(:,2),path(:,1));
pathEl = 90 - acosd(path(:,3)./sqrt(sum(path.^2,2)));

h.fig1 = figure('Name','Summary','Position',[-1.0000    0.0370    1.0000    0.8917]);

% Path Shape
h.pathShapeAx  = subplot(nRow,nCol,[1 2]);
h.targetPath   = plot(pathAz,pathEl,...
    'LineWidth',2,'Color','r','LineStyle',':','DisplayName','Target Path');
grid on;hold on
h.actualPath_iMin1  = plot(nan,nan,'Color','k','DisplayName','$i-1$','LineStyle','-');
h.actualPath_i      = plot(nan,nan,'Color',[0 0.75 0],'DisplayName','$i$, Actual','LineStyle','-');
h.predicPath_iPls1  = plot(nan,nan,'Color','b','DisplayName','$i+1$, Predicted','LineStyle','-');
h.fltPathAx = gca;
xlabel('Azimuth, [deg]')
ylabel('Elevation, [deg]')
h.legend1 = legend;
h.title = title('');

% Control input
h.uAx      = subplot(nRow,nCol,nCol+1);
yyaxis left
h.uFBPrev = plot(nan,nan,'Color','k','DisplayName','$u_{FB}^{j}$');
grid on;hold on
h.uFFPrev = plot(nan,nan,'Color','b','DisplayName','$u_{FF}^{j}$');
h.uFFNext = plot(nan,nan,'Color','r','DisplayName','$u_{FF}^{j+1}$');
xlim([0 1]);
xlabel('Path Var')
ylabel('$\delta u$')
h.uAx.XAxis.TickValues =0:0.125:1;
set(gca, 'TickLabelInterpreter','latex')
h.ctrlLegend = legend;
h.uAx.XAxis.TickLabels = {'0','$\frac{1}{8}$','$\frac{1}{4}$','$\frac{3}{8}$','$\frac{1}{2}$','$\frac{5}{8}$','$\frac{3}{4}$','$\frac{7}{8}$','$1$'};

% Distance travelled
h.distAx   = subplot(nRow,nCol,(nRow-1)*nCol+1);
h.distPred = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('Predicted, [m]')
yyaxis right
h.distAct = scatter(nan,nan);
ylabel('Actual, [m]')
title('Dist. Travelled')


% Plot control input
h.delUAx   = subplot(nRow,nCol,2*nCol+3);
yyaxis left
h.delUTermW = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J_u$')
yyaxis right
ylabel('$J_u''$')
h.delUTermU = scatter(nan,nan);

%% Plot components of the performance index

% Overall performance index
h.perfIndxAx = subplot(nRow,nCol,3);
h.perfIndx   = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J$')

% Speed term in performance index, weighted and unweighted
h.speedAx   = subplot(nRow,nCol,nCol);
yyaxis left
h.speedTermW = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J_v$')
yyaxis right
ylabel('$J_v''$')
h.speedTermU = scatter(nan,nan);

% Terminal incentive term, weighted and unweighted
h.termAx = subplot(nRow,nCol,nCol+3);
yyaxis left
h.termTermW = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J_t$')
yyaxis right
ylabel('$J_t''$')
h.termTermU = scatter(nan,nan);

% Error term, weighted and unweighted
h.errAx   = subplot(nRow,nCol,nCol+4);
yyaxis left
h.errTermW = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J_e$')
yyaxis right
ylabel('$J_e''$')
h.errTermU = scatter(nan,nan);

% Mean speed and iteration time
h.meanSpeedAx   = subplot(nRow,nCol,nRow*nCol);
yyaxis left
h.meanSpeed = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel({'Avg. Speed, [m/s]'})
yyaxis right
ylabel('$T_F$ [s]')
h.simTime   = scatter(nan,nan);

% Path shape inset
h.zoomAx = axes(gcf,'Units','Normalize','Position',[0.4104    0.5944    0.0990    0.1564]);
box on
h.targetPathZoom = plot(pathAz,pathEl,...
    'LineWidth',2,'Color','r','LineStyle',':','DisplayName','Target Path');
grid on;hold on
h.actualPath_iMin1Zm  = plot(nan,nan,'Color','k','DisplayName','$i-1$, Actual','LineStyle','-');
h.actualPath_iZm      = plot(nan,nan,'Color',[0 0.75 0],'DisplayName','$i$, Actual','LineStyle','-');
h.predicPath_iPls1Zm  = plot(nan,nan,'Color','b','DisplayName','$i+1$, Predicted','LineStyle','-');
xlim([pathAz(1)-2 pathAz(1)+2])
ylim([pathEl(1)-2 pathEl(1)+2])

set(findall(h.fig1,'Type','Axes'),'FontSize',18)
