% A basic test script demonstrating how to run the unifoil model.
% All units are in radians except when otherwise stated (eg some basis
% parameters)

%% Setup
clear;close all

%% Simulation Options
T = 100; % Simulation duration

%% Kite properties
% Physical properties
baseMass        = 6184;
addedMass       = 739.6;
baseInertia     = 10000;%80302;%104850;
addedInertia    = 10000;%724530;
buoyFactor      = 1;
ARefWing        = 20;
ARefRudder      = 1.875;
ARefElevator    = 1.875;
fuselageLength  = 8;
wingOE          = 0.8;
rudderOE        = 0.8;
wingAR          = 5;
rudderAR        = 3;
wingTable       = buildAirfoilTable('wing',wingOE,wingAR);
rudderTable     = buildAirfoilTable('rudder',wingOE,wingAR);

% Initial Conditions
radius          = 100;
initSpeed       = 5.735;
initAzimuth     = 0.01*pi/180;
initElevation   = 30*pi/180;
initTwist       = -20*pi/180;%-22.5*pi/180;
initTwistRate   = 0;

%% Water properties
flowSpeed   = 1;
density     = 1000;

%% Controller parameters
% Path geometry
azimuthSweep    = 60*pi/180; % Path azimuth sweep angle, degrees
elevationSweep  = 20*pi/180; % Path elevation sweep angle, degrees
meanAzimuth     = 0*pi/180;
meanElevation   = 30*pi/180;
pathShape       = 2; % 1 for ellipse, 2 for fig 8
basisParams = [azimuthSweep, elevationSweep, meanAzimuth, meanElevation, radius, pathShape];
% Reference model
tauRef          = 0.25; % reference model time constant (s)
% Model Ref Ctrl Gains
refGain1        = 1/tauRef^2;
refGain2        = 2/tauRef;
% Pure Pursuit Controller
initPathVar     = 0;
maxLeadLength   = 0.05;
maxIntAngle     = 10*pi/180;
% Min and max alpha for the wing controller
wingAlphaPlusStall  =  10*pi/180;
wingAlphaMinusStall = -10*pi/180;
% Min and max alpha for the rudder controller
rudderAlphaPlusStall    =  6*pi/180;
rudderAlphaMinusStall   = -6*pi/180;
% Coefficients of linear and quadratic fits to CL and CD curves
[wingCLCoeffs,wingCDCoeffs]     = fitTable(wingTable,5*[-1 1]*pi/180);
[rudderCLCoeffs,rudderCDCoeffs] = fitTable(rudderTable,6*[-1 1]*pi/180);

%% Flexible Time ILC Parameters
numIters = 45;

pathStep = 1/400;
% Linear force parmeterization
FxParams = [3000 10];

% Waypoint Specification
posWayPtPathVars = [0.25 0.5 0.75 1];

% Calculate derived quantities
wyPtPosVecs     = lemOfGerono(posWayPtPathVars,basisParams);
wyPtAzimuth     = atan2(wyPtPosVecs(:,2),wyPtPosVecs(:,1));
wyPtElevation   = pi/2-acos(wyPtPosVecs(:,3)./sqrt(sum(wyPtPosVecs.^2,2)));
r               = reshape([wyPtAzimuth(:),wyPtElevation(:)]',[2*numel(posWayPtPathVars) 1]);

% Performance index components
spedWeight = 5/(2000);
wyptWeight = 0.05/(1e-4);
inptWeight = 0.025/(sum((pi/180).^2*ones(size(0:pathStep:1))));

%% Set up some plots
path        = lemOfGerono(linspace(0,1),basisParams);
pathAz = atan2d(path(:,2),path(:,1));
pathEl = 90 - acosd(path(:,3)./sqrt(sum(path.^2,2)));
h.plot  = subplot(3,3,[1 2]);
plot(pathAz,pathEl,...
    'LineWidth',2,'Color','r','LineStyle',':','DisplayName','Target Path')
grid on;hold on
% h.origin = scatter3(0,0,0,'MarkerFaceColor','k','MarkerEdgeColor','k','DisplayName','Origin');
h.fltPathAx = gca;
xlabel('Azimuth, [deg]')
ylabel('Elevation, [deg]')
% view(68,19)

h.perfIndxAx = subplot(3,3,3);
yyaxis left
h.perfIndx   = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J$')
yyaxis right
ylabel('$T_F$ [s]')
h.simTime   = scatter(nan,nan);

h.speedAx   = subplot(3,3,6);
yyaxis left
h.speedTermW = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J_v$ [m/s]')
yyaxis right
h.speedTermU = scatter(nan,nan);

h.delUAx   = subplot(3,3,8);
yyaxis left
h.delUTermW = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J_u$ [rad$^2$]')
yyaxis right
h.delUTermU = scatter(nan,nan);

h.errAx   = subplot(3,3,9);
yyaxis left
h.errTermW = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('$J_e$ [rad$^2$]')
yyaxis right
h.errTermU = scatter(nan,nan);


h.uAx      = subplot(3,3,7);
yyaxis left
h.uFBPrev = plot(nan,nan,'Color','k','DisplayName','$u_{FB}^{j}$');
grid on;hold on
h.uFFPrev = plot(nan,nan,'Color','b','DisplayName','$u_{FF}^{j}$');
yyaxis right
h.uFFNext = plot(nan,nan,'Color','r','DisplayName','$u_{FF}^{j+1}$');
xlim([0 1]);
xlabel('Path Var')
ylabel('$\delta u$ [deg]')
h.uAx.XAxis.TickValues=0:0.125:1;
set(gca, 'TickLabelInterpreter','latex')
legend
h.uAx.XAxis.TickLabels = {'0','$\frac{1}{8}$','$\frac{1}{4}$','$\frac{3}{8}$','$\frac{1}{2}$','$\frac{5}{8}$','$\frac{3}{4}$','$\frac{7}{8}$','$1$'};


h.FxErrAx   = subplot(3,3,4);
h.FxErr = scatter(nan,nan);
grid on;hold on
xlim([1 numIters]);
xlabel('Iteration Num, j')
ylabel('Fx Fit Err, [N]')

set(findall(gcf,'Type','Axes'),'FontSize',18)

%% Run the ILC algorithm
UNext = timesignal(timeseries(...
    zeros(numel(0:pathStep:1),1),...
    0:pathStep:1));
for ii = 1:numIters
    % Run the simulation

    sim('unifoil');
    
    % Post-process Data (get it into a signalcontainer object)
    tsc = signalcontainer(logsout);
    
    %% Do operations for flexible time ILC
    
    % Crop to just one lap
    psc = processTSC(tsc,pathStep);
    
    % Fit the function Fx to match the data   
    [FxParams, FxErr] = fitFxPhase(psc);
    
    % Create continuous, linear, path parmeterized model
    [Acp,Bcp] = pathLinearize(psc,basisParams,FxParams,tauRef,baseMass+addedMass);
    
    % Discretize the continuous, linear, path-domain model
    [Adp,Bdp] = disc(Acp,Bcp,pathStep);
    
    % Lift the discrete, linear, path-domain model
    [F,G] = lift(Adp,Bdp);
    
    % Build the weighting matrix for the performance index
    Psiv = stateSelectionMatrix(3,5,numel(Adp.Time));
    f    = spedWeight*ones(1,size(Psiv,1));
    posWayPtPathIdx = cnvrtPathVar2Indx(posWayPtPathVars,Adp.Time);
    PsiW = wyptSelectionMatrix([1 2],posWayPtPathIdx,5,numel(Adp.Time))*stateSelectionMatrix([1 2],5,numel(Adp.Time));
    QW = wyptWeight*diag(ones(size(PsiW,1),1));
    Qu = inptWeight*diag(ones(numel(Adp.Time),1));
    GHatj = G'*PsiW'*QW*PsiW*G;
    L0 = (Qu-GHatj)^(-1);
    Lc = L0*(1/2)*(f*Psiv*G)';
    Le = -L0*G'*PsiW'*QW;
        
    % Apply learning filters to calculate deviation in input signal
    delUData = Lc+Le*(r-PsiW*psc.stateVec.Data(:));
    delU = timesignal(timeseries(delUData,Adp.Time));
    
    % Calculate some components of the performance index
    Jv = -f*Psiv*psc.stateVec.Data(:);
    JvPrime = -ones(1,size(Psiv,1))*Psiv*psc.stateVec.Data(:);
    
    Ju = delU.Data(:)'*Qu*delU.Data(:);
    JuPrime = delU.Data(:)'*delU.Data(:);
    
    Je = (r-PsiW*psc.stateVec.Data(:))'*QW*(r-PsiW*psc.stateVec.Data(:));
    JePrime = (r-PsiW*psc.stateVec.Data(:))'*(r-PsiW*psc.stateVec.Data(:));
    
    % Resample deviation in control signal into standard domain
    UNext = UNext.resample(delU.Time);
    UNext = UNext + delU;
    UNext = UNext.rmnans;
    
    % Plot the flight path
    plot(tsc.azimuth.Data*180/pi,tsc.elevation.Data*180/pi,...
        'LineWidth',1,'Color','b','Parent',h.fltPathAx)
    
    % Plot the overall performance index
    h.perfIndx.XData = [h.perfIndx.XData ii];
    h.perfIndx.YData = [h.perfIndx.YData Jv+Ju+Je];
    h.simTime.XData = [h.simTime.XData ii];
    h.simTime.YData = [h.simTime.YData tsc.pathVar.Time(end)];
    
    % Plot the speed term
    h.speedTermW.XData = [h.speedTermW.XData ii];
    h.speedTermW.YData = [h.speedTermW.YData Jv];
    h.speedTermU.XData = [h.speedTermU.XData ii];
    h.speedTermU.YData = [h.speedTermU.YData JvPrime];
    
    % Plot the error (waypoint tracking) term
    h.errTermW.XData = [h.errTermW.XData ii];
    h.errTermW.YData = [h.errTermW.YData Je];
    h.errTermU.XData = [h.errTermU.XData ii];
    h.errTermU.YData = [h.errTermU.YData JePrime];
    
    % Plot the delta u term
    h.delUTermW.XData = [h.delUTermW.XData ii];
    h.delUTermW.YData = [h.delUTermW.YData Ju];
    h.delUTermU.XData = [h.delUTermU.XData ii];
    h.delUTermU.YData = [h.delUTermU.YData JuPrime];
        
    % Plot the control signals
    h.uFBPrev.XData = tsc.pathVar.Data;
    h.uFBPrev.YData = tsc.uFB.Data*180/pi;
    
    h.uFFPrev.XData = psc.pathVar.Data;
    h.uFFPrev.YData = psc.uFF.Data*180/pi;
    
    h.uFFNext.XData = UNext.Time;
    h.uFFNext.YData = UNext.Data*180/pi;
    
    % Plot the Fx Fit Error
    h.FxErr.XData = [h.FxErr.XData ii];
    h.FxErr.YData = [h.FxErr.YData FxErr];
    
    
    drawnow
end
