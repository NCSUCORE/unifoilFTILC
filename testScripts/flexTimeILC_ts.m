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
numIters = 20;

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
spedWeight = 4/6.6e4;
wyptWeight = 0.015/(1e-5);
inptWeight = 0.1/(sum((pi/180).^2*ones(size(0:pathStep:1))));

%% Set up some plots
setUpPlots


%% Run the ILC algorithm
UNext = timesignal(timeseries(...
    zeros(numel(0:pathStep:1),1),...
    0:pathStep:1));
for ii = 1:numIters
    % Run the simulation

    sim('unifoil');
    
    % Post-process Data (get it into a signalcontainer object)
    tsc = signalcontainer(logsout);
    
    h.actualPath_iMin1.XData = h.actualPath_i.XData;
    h.actualPath_iMin1.YData = h.actualPath_i.YData;
    h.actualPath_iMin1Zm.XData = h.actualPath_iZm.XData;
    h.actualPath_iMin1Zm.YData = h.actualPath_iZm.YData;
    
    h.actualPath_i.XData = tsc.azimuth.Data*180/pi;
    h.actualPath_i.YData = tsc.elevation.Data*180/pi;
    h.actualPath_iZm.XData = tsc.azimuth.Data*180/pi;
    h.actualPath_iZm.YData = tsc.elevation.Data*180/pi;
    
    h.title.String = sprintf('$i = %d$',ii);
    % add dsdt as a signal
    tsc.addprop('dsdt');
    tsc.dsdt = tsc.pathVar.diff;
    
    %% Do operations for flexible time ILC
    
    % Crop to just one lap
    psc = processTSC(tsc,pathStep);
    
    % Fit the function Fx to match the data   
    [FxParams, ~] = fitFxPhase(psc);
    
    % Create continuous, linear, path parmeterized model
    [Acp,Bcp] = pathLinearize(psc,basisParams,FxParams,tauRef,baseMass+addedMass);
    
    % Discretize the continuous, linear, path-domain model
    [Adp,Bdp] = disc(Acp,Bcp,pathStep);
    
    % Lift the discrete, linear, path-domain model
    [F,G] = lift(Adp,Bdp);
    
    % Build the weighting matrix for the performance index
    % Linear term
    Psiv   = spedWeight*stateSelectionMatrix(3,5,numel(Adp.Time));
    [~,ds] = psc.arcLength.diff;
    fv     = repmat(ds(:)./psc.dsdt.Data(:),[1 5])';
    fv     = fv(:)';
    % Waypoint tracking term
    posWayPtPathIdx = cnvrtPathVar2Indx(posWayPtPathVars,Adp.Time);
    PsiW = wyptSelectionMatrix([1 2],posWayPtPathIdx,5,numel(Adp.Time))*stateSelectionMatrix([1 2],5,numel(Adp.Time));
    QW = wyptWeight*diag(ones(size(PsiW,1),1));
    % Input variation term
    Qu = inptWeight*diag(ones(numel(Adp.Time),1));
    GHatj = G'*PsiW'*QW*PsiW*G;
    % Filters
    L0 = (Qu-GHatj)^(-1);
    Lc =  L0*(1/2)*(fv*Psiv*G)';
    Le = -L0*G'*PsiW'*QW;
        
    % Apply learning filters to calculate deviation in input signal
    delUData = Lc+Le*(r-PsiW*psc.stateVec.Data(:));
    delU = timesignal(timeseries(delUData,Adp.Time));

    updatePlots
    
    % Resample deviation in control signal into standard domain
    UNext = UNext.resample(delU.Time);
    UNext = UNext + delU;
    UNext = UNext.rmnans;
end

