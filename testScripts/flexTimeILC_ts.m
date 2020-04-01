% Test script to fun flexible time ILC.

%% Setup
clear;close all

%% Simulation Options
T = 1500; % Simulation duration

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
numIters = 40;

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

% Performance index components
% Weights on linear state terms
q1phi   = 0;
q1theta = 0;
q1v     = (-1/(ns*2)); % none per path step per mps
q1psi   = 0;
q1omega = 0;
q1x = [q1phi q1theta q1v q1psi q1omega];
% Weights on quadratic error terms
qe2phi   = (1)/(nw*(1*pi/180)^2); %(none per rad^2 per waypoint)
qe2theta = (1)/(nw*(1*pi/180)^2); %(none per rad^2 per waypoint)
qe2v     = 0;
qe2psi   = 0;
qe2omega = 0;
qe2x = [qe2phi qe2theta qe2v qe2psi qe2omega];
% Weights on quadratic state deviation terms
qx2phi   = (1)/(ns*(1*pi/180)^2); %(none per rad^2 per path step)
qx2theta = (1)/(ns*(1*pi/180)^2); %(none per rad^2 per path step)
qx2v     = (1)/(ns*(1)^2); %(none per mps^2 per path step)
qx2psi   = 0;
qx2omega = 0;
qx2x = [qx2phi qx2theta qx2v qx2psi qx2omega];
% Weights on linear input terms
q1uw = 0;
q1ur = 0;
q1u = [q1uw q1ur];
% Weights on quadratic input deviation terms
q2uw = (1)/(ns*(2*pi/180)^2); %none per radian squared per path step
q2ur = (1)/(ns*(2*pi/180)^2); %none per radian squared per path step
q2u = [q2uw q2ur];

% Derived quantities
pathPosVecs     = lemOfGerono(Ss,basisParams);
pathAz   = atan2(pathPosVecs(:,2),pathPosVecs(:,1));
pathEl   = pi/2-acos(pathPosVecs(:,3)./sqrt(sum(pathPosVecs.^2,2)));
wyPtPosVecs     = lemOfGerono(Sw,basisParams);
wyPtAzimuth     = atan2(wyPtPosVecs(:,2),wyPtPosVecs(:,1));
wyPtElevation   = pi/2-acos(wyPtPosVecs(:,3)./sqrt(sum(wyPtPosVecs.^2,2)));


% Build the weighting matrices for the performance index
Lx = repmat(q1x,[1 ns-1]);
Lu = repmat(q1u,[1 ns-1]);
Wx = weightMatrix(qe2x,ns-1);
Px = wyptSelectionMatrix(Iw-1,nx,ns-1);
Qe = Px*Wx;
Qu = weightMatrix(q2u,ns-1);
Qx = weightMatrix(qx2x,ns-1);
r  = [pathAz(:) pathEl(:) zeros(ns,1) zeros(ns,1) zeros(ns,1)]';
r  = r(nx+1:end)';

%% Set up some plots
setUpPlots

%% Run the ILC algorithm
% Set the initial ILC inputs
uwilcNext = timesignal(timeseries(zeros(size(Ss(1:end-1))),Ss(1:end-1)));
urilcNext = timesignal(timeseries(zeros(size(Ss(1:end-1))),Ss(1:end-1)));
for ii = 1:numIters
    
    % Run the simulation
    sim('unifoil');
    
    % Post-process Data (get it into a signalcontainer object)
    tsc = signalcontainer(logsout);
    
    % Crop to just one lap
    psc = processTSC(tsc,Ss);
    
    % Create some variables from the logged data
    if ii == 1
        wingCtrlKp = 0;
        uilcPrev = [psc.uwfb.Data(:) psc.urilc.Data(:)];
        psc.uilc = timesignal(timeseries(permute(uilcPrev,[2 3 1]),...
            psc.pathVar.Time));
    end
    uilcPrev  = psc.uilc.Data(1:end-nu)';
    
    % Create continuous, linear, path parmeterized model
    [Acp,Bcp] = pathLinearize(psc,...
        basisParams,...
        wingC,ARefWing,tauRef,baseMass+addedMass,...
        wingCtrlKp,wingTargetSpeed,...
        density,flowSpeed);
    
    % Discretize the continuous, linear, path-domain model
    [Adp,Bdp] = disc(Acp,Bcp,Ss);
    
    % Lift the discrete, linear, path-domain model
    [F,G] = lift(Adp,Bdp);
    
    % Build the weighting matrices for the performance index
    Fo = (G'*Qe*G + Qu);
    Fo = inv(0.5*(Fo'+Fo)); %Enforce symmetry
    Fc = -0.5*Fo*(G'*Lx'+Lu');
    Fe =      Fo*(G'*Qe);
    
    % Apply learning filters to calculate deviation in input signal
    uilcNext = uilcPrev + Fc + Fe*(r - psc.stateVec.Data(nx+1:end)');
    
    % Break these into individual signals and put into timesignals
    uwilcNext = timesignal(timeseries(uilcNext(1:2:end),Ss(1:end-1)));
    urilcNext = timesignal(timeseries(uilcNext(2:2:end),Ss(1:end-1)));
    
%     initSpeed       = tsc.stateVec.Data(3,1,end);
%     initAzimuth     = tsc.stateVec.Data(1,1,end);
%     initElevation   = tsc.stateVec.Data(2,1,end);
%     initTwist       = tsc.stateVec.Data(4,1,end);%-22.5*pi/180;
%     initTwistRate   = tsc.stateVec.Data(5,1,end);
    
    % Update the plots and save the results
    updatePlots
    
    savePlot(h.fig1,'output',sprintf('Summary%d',ii))
    savePlot(h.fig2,'output',sprintf('States%d',ii))
    savePlot(h.fig3,'output',sprintf('WingControl%d',ii))
    savePlot(h.fig4,'output',sprintf('Fx%d',ii))
end

