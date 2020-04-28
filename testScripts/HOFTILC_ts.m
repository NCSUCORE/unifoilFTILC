% Test script to fun flexible time ILC.

%% Setup
clear;close all

%% Simulation Options
T = 1500; % Simulation duration

%% Kite properties
% Physical properties
baseMass        = 2857;
addedMass       = 134;
baseInertia     = 24675;
addedInertia    = 23320;%724530;
buoyFactor      = 1;
ARefWing        = 10;
ARefRudder      = 2;
ARefElevator    = 3;
fuselageLength  = 8;
wingOE          = 0.8;
rudderOE        = 0.8;
wingAR          = 10;
rudderAR        = 3;
radius          = 100;
wingTable       = buildAirfoilTable('wing',wingOE,wingAR);
rudderTable     = buildAirfoilTable('rudder',wingOE,wingAR);

%% Water properties
flowSpeed   = 1;
density     = 1000;

%% Controller parameters
% Path geometry
azimuthSweep    = 60*pi/180; % Path azimuth sweep angle, degrees
elevationSweep  = 20*pi/180; % Path elevation sweep angle, degrees
meanAzimuth     = 0*pi/180;
meanElevation   = 30*pi/180;
basisParams     = [azimuthSweep, elevationSweep, meanAzimuth, meanElevation, radius];
% Reference model
tauRef          = 0.25; % reference model time constant (s)
% Model Ref Ctrl Gains
refGain1        = 1/tauRef^2;
refGain2        = 2/tauRef;
% Pure Pursuit Controller
maxLeadLength   = 0.05;
maxIntAngle     = 10*pi/180;
% Min and max alpha for the wing controller
wingAlphaPlusStall  =  30*pi/180;
wingAlphaMinusStall = -30*pi/180;
% Min and max alpha for the rudder controller
rudderAlphaPlusStall    =  6*pi/180;
rudderAlphaMinusStall   = -6*pi/180;
% Coefficients of linear and quadratic fits to CL and CD curves
wingC     = fitTable(wingTable,5*[-1 1]*pi/180);
rudderC   = fitTable(rudderTable,6*[-1 1]*pi/180);
% Wing AoA Controller Parameters
wingTargetSpeed = 3.5;
wingCtrlKp = 0.349;

%% Initial Conditions
initSpeed       = 4.15;
initAzimuth     = meanAzimuth;
initElevation   = meanElevation;
initTwist       = -33*pi/180;%-22.5*pi/180;
initTwistRate   = 0;
initPathVar     = 0;

%% Flexible Time ILC Parameters
numIters = 50;

% Number of states and control inputs
nx = 5;
nu = 2;
% Path discretization parameters
ns = 400;
Ss = linspace(0,1,ns); % Set of all discretized path variables
% Waypoint Specification
Sw = [0.25 0.5 0.75 1]; % Set of path variables at the waypoints
nw = numel(Sw); % Number of waypoints
Iw = cnvrtPathVar2Indx(Sw,Ss);

%% Performance index components

% Quadratic penalties on the input
Quw = 0;
Qur = 0;
Qu = weightMatrix([Quw Qur],ns);

% Quadratic penalties on the input deviation
Qduw = (1)/(ns*(1*pi/180)^2); % none per radian squared per path step
Qdur = (1)/(ns*(1*pi/180)^2); % none per radian squared per path step
Qdu =  weightMatrix([Qduw Qdur],ns);

% Quadratic penalties on the error
Qephi   = (1)/(nw*(0.1*pi/180)^2); %(none per rad^2 per waypoint)
Qetheta = (1)/(nw*(0.1*pi/180)^2); %(none per rad^2 per waypoint)
Qev     = 0;
Qepsi   = 0; %(none per rad^2 per waypoint)
Qeomega = 0; %(none per rad^4 per waypoint)
Qe =  wyptSelectionMatrix(Iw,nx,ns)*weightMatrix([Qephi Qetheta Qev Qepsi Qeomega],ns);

% Quadratic penalties on the error deviation
Qdephi   = 0;
Qdetheta = 0;
Qdev     = 0;
Qdepsi   = 0;
Qdeomega = 0;
Qde =  wyptSelectionMatrix(Iw,nx,ns)*weightMatrix([Qdephi Qdetheta Qdev Qdepsi Qdeomega],ns);

% Quadratic penalties on the state magnitude
Qxphi      = 0;
Qxtheta    = 0;
Qxv        = 0;
Qxpsi      = 0;
Qxomega    = 0;
Qx =  weightMatrix([Qxphi Qxtheta Qxv Qxpsi Qxomega],ns);

% Quadratic penalties on the state deviation
Qdxphi      = (1)/(ns*(1*pi/180)^2); %(none per rad^2 per path step)
Qdxtheta    = (1)/(ns*(1*pi/180)^2); %(none per rad^2 per path step)
Qdxv        = (1)/(ns*(1)^2); %(none per mps^2 per path step)
Qdxpsi      = 0;
Qdxomega    = 0;
Qdx =  weightMatrix([Qdxphi Qdxtheta Qdxv Qdxpsi Qdxomega],ns);

% Linear term on the states at the next iteration
Sxphi   = 0;
Sxtheta = 0;
Sxv     = -1/(ns*1); % none per path step per mps
Sxpsi   = 0;
Sxomega = 0;
Sx = repmat([Sxphi Sxtheta Sxv Sxpsi Sxomega],[1 ns]);

% Delta matrix (initial condition difference operator)
Delta = cell(1,ns);
Delta(:) = {zeros(nx,nx)};
Delta(1) = {eye(nx)};
Delta(end) = {-eye(nx)};
Delta = cell2mat(Delta);

% Reference Signal
[pathPosVecs,tanVec]     = lemOfGerono(Ss,basisParams);
pathAz          = atan2(pathPosVecs(:,2),pathPosVecs(:,1));
pathEl          = pi/2-acos(pathPosVecs(:,3)./sqrt(sum(pathPosVecs.^2,2)));
pathTwist       = atan2(tanVec(:,2),tanVec(:,1));
r               = [pathAz(:) pathEl(:) zeros(ns,1) pathTwist(:) zeros(ns,1)]';
r               = r(:);

% Block lifted matrices
QQu = [Qu + 2*Qdu -Qdu;-Qdu Qu + Qdu];
QQe = [Qe + 2*Qde -Qde;-Qde Qe + Qde];
QQx = [Qx + 2*Qdx -Qdx;-Qdx Qx + Qdx];
SSx = [Sx Sx];
qqdu = [-2*Qdu zeros(size(Qdu,1),ns*nu)];
qqde = [-2*Qde zeros(size(Qde,1),ns*nx)];
qqdx = [-2*Qdx zeros(size(Qdx,1),ns*nx)];
EI = [eye(nx) zeros(nx,nx*(ns-1))];
EF = [zeros(nx,nx*(ns-1)) eye(nx) ];
II = [eye(ns*nx);eye(ns*nx)];

%% Set up some plots
% setUpPlots
setUpPaperPlots

%% Run the ILC algorithm
% Set the initial ILC inputs
uwilcNext = timesignal(timeseries(zeros(size(Ss(1:end-1))),Ss(1:end-1)));
urilcNext = timesignal(timeseries(zeros(size(Ss(1:end-1))),Ss(1:end-1)));
for ii = 1:numIters
    
    % Run the simulation
    sim('unifoil');
    
    % Post-process Data (get it into a signalcontainer object)
    tsc = signalcontainer(logsout);
    xI = tsc.stateVec.getdatasamples(1);
    xF = tsc.stateVec.getdatasamples(numel(tsc.pathVar.Time(end)));
    sF = tsc.pathVar.Data(end);
    
    % Crop to just one lap
    psc = processTSC(tsc,Ss);
    
    % Create some variables from the logged data
    if ii == 1
        % Turn off wing feedback controller
        wingCtrlKp = 0;
        % Overwrite wing ILC input with results from FB controller
        uilcPrev    = [psc.uwfb.Data(:) psc.urilc.Data(:)];
        % Create variables for delta quantity signals
        duilcPrev   = nan(size(psc.uilc.Data(:)));
        dxPrev      = nan(size(psc.stateVec.Data(:)));
        dePrev      = nan(size(psc.stateVec.Data(:)));
        % Overwrite the signals in the pathsignal container
        psc.uilc = timesignal(timeseries(permute(uilcPrev,[2 3 1]),...
            psc.pathVar.Time));
    else
        % Calculate variables for delta quantity signals
        duilcPrev = psc.uilc.Data(:)     - uilcPrev;
        dxPrev    = psc.stateVec.Data(:) - xPrev;
        dePrev    = (r - psc.stateVec.Data(:)) - ePrev;
    end
    uilcPrev    = psc.uilc.Data(:);
    xPrev       = psc.stateVec.Data(:);
    ePrev       = r - xPrev;
    meanSpeed   = tsc.speed.mean;
    iterDur     = tsc.pathVar.Time(end);
    
    % Create continuous, linear, path parmeterized model
    [Acp,Bcp] = pathLinearize(psc,...
        basisParams,...
        wingC,ARefWing,tauRef,baseMass+addedMass,...
        wingCtrlKp,wingTargetSpeed,...
        density,flowSpeed);
    
    % Discretize the continuous, linear, path-domain model
    [Adp,Bdp] = disc(Acp,Bcp,Ss);
    
    % Lift the discrete, linear, path-domain model
    [F,H] = lift(Adp,Bdp);
    
    % Build the weighting matrices for the performance index
    Fhat   = F*(EF - EI);
    FF = [ Fhat ; F*EF + F*EF*Fhat - F*EI];
    GG = -[eye(ns*nx) ; F*EF+eye(ns*nx)]*H;
    HH = [H zeros(ns*nx,ns*nu); F*EF*H H];
    
    LL0 = inv(QQu + HH'*(QQe + QQx)*HH);
    LLu = -LL0*(0.5*qqdu' + HH'*(QQe + QQx)*GG);
    LLe = +LL0*HH'*(QQe*II + 0.5*qqde');
    LLx = -LL0*HH'*(QQe*FF + QQx*II + QQx*FF + 0.5*qqdx');
    LLc = -LL0*0.5*HH'*SSx';
    
    % Apply learning filters to calculate deviation in input signal
    uilcNext = LLu*uilcPrev + LLe*(r(:) - psc.stateVec.Data(:)) + LLx*psc.stateVec.Data(:) + LLc;
    uilcNext = [eye(ns*nu) zeros(ns*nu)]*uilcNext;
    
    % Break these into individual signals and put into timesignals
    uwilcNext = timesignal(timeseries(uilcNext(1:2:end),Ss));
    urilcNext = timesignal(timeseries(uilcNext(2:2:end),Ss));
    
    % Set the initial conditions for the next iteration
    initAzimuth     = xF(1);
    initElevation   = xF(2);
    initSpeed       = xF(3);
    initTwist       = xF(4);
    initTwistRate   = xF(5);
    initPathVar     = sF;
    
    % Calculate predicted signals for next iteration
    xNext = xPrev + H*(uilcNext - uilcPrev) + F*(xF-xI);
    eNext = r - xNext;
    
    % Calculate
    JuPrev = uilcPrev'*Qu*uilcPrev;
    JuNext = uilcNext'*Qu*uilcNext;
    
    JduPrev = duilcPrev'*Qdu*duilcPrev;
    JduNext = (uilcNext - uilcPrev)'*Qdu*(uilcNext - uilcPrev);
    
    JePrev = ePrev'*Qe*ePrev;
    JeNext = eNext'*Qe*eNext;
    
    JdePrev = dePrev'*Qde*dePrev;
    JdeNext = (eNext - ePrev)'*Qde*(eNext - ePrev);
    
    JdxPrev = dxPrev'*Qdx*dxPrev;
    JdxNext = (xNext - xPrev)'*Qdx*(xNext - xPrev);
    
    JsxPrev = Sx*xPrev;
    JsxNext = Sx*xNext;
    
    JPrev = JuPrev + JduPrev + JePrev + JdePrev + JdxPrev + JsxPrev;
    JNext = JuNext + JduNext + JeNext + JdeNext + JdxNext + JsxNext;
    
    % Update the plots
    %     updatePlots
    updatePaperPlots
    
    
end
%%
% savePlot(h.pathFig,'output',h.pathFig.Name)
% savePlot(h.spedFig,'output',h.spedFig.Name)
% savePlot(h.perfFig,'output',h.perfFig.Name)

savePlot(h.pathShapeFig,'output',h.pathShapeFig.Name)
savePlot(h.performanceFig,'output',h.performanceFig.Name)
savePlot(h.waypointTrackingFig,'output',h.waypointTrackingFig.Name)
savePlot(h.durationFig,'output',h.durationFig.Name)