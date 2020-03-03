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
pathShape       = 2; % 1 for ellipse, 2 for fig 8
basisParams = [azimuthSweep, elevationSweep, meanAzimuth, meanElevation, radius, pathShape];
% Reference model
tauRef          = 0.25; % reference model time constant (s)
% Model Ref Ctrl Gains
refGain1        = 1/tauRef^2;
refGain2        = 2/tauRef;
% Pure Pursuit Controller
maxLeadLength   = 0.01;
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

%% Initial Conditions
initSpeed       = 5.735;
initAzimuth     = meanAzimuth;
initElevation   = meanElevation;
initTwist       = -20*pi/180;%-22.5*pi/180;
initTwistRate   = 0;
initPathVar     = 0;

%% Flexible Time ILC Parameters
numIters = 20;

pathStep = 1/400;
% Linear force parmeterization
FxParams = [3000 10];

% Waypoint Specification
posWayPtPathVars = [0.25 0.5 0.75 1];
posWayPtPathIdx = cnvrtPathVar2Indx(posWayPtPathVars,0:pathStep:1);

% Calculate derived quantities
wyPtPosVecs     = lemOfGerono(posWayPtPathVars,basisParams);
wyPtAzimuth     = atan2(wyPtPosVecs(:,2),wyPtPosVecs(:,1));
wyPtElevation   = pi/2-acos(wyPtPosVecs(:,3)./sqrt(sum(wyPtPosVecs.^2,2)));

% Performance index components
spedWeight = 3e-1;
wyptWeight = 4.5e2;
inptWeight = 5e-2;

%% Set up some plots
setUpPlots

%% Run the ILC algorithm
 UNext = timesignal(timeseries([0 0],[0 1]));
for ii = 1:numIters
    % Run the simulation
    
    sim('unifoil');
    
    % Post-process Data (get it into a signalcontainer object)
    tsc = signalcontainer(logsout);
    
    % Update some plotting stuff
    h.actualPath_iMin1.XData = h.actualPath_i.XData;
    h.actualPath_iMin1.YData = h.actualPath_i.YData;
    h.actualPath_iMin1Zm.XData = h.actualPath_iZm.XData;
    h.actualPath_iMin1Zm.YData = h.actualPath_iZm.YData;
    h.actualPath_i.XData = tsc.azimuth.Data*180/pi;
    h.actualPath_i.YData = tsc.elevation.Data*180/pi;
    h.actualPath_iZm.XData = tsc.azimuth.Data*180/pi;
    h.actualPath_iZm.YData = tsc.elevation.Data*180/pi;
    h.title.String = sprintf('$i = %d$',ii);
    
    %% Do operations for flexible time ILC
    
    % Crop to just one lap
    psc = processTSC(tsc,0:pathStep:1);
    
    % Fit the function Fx to match the data
    [FxParams, ~] = fitFxPhase(psc);
    
    % Create continuous, linear, path parmeterized model
    [Acp,Bcp] = pathLinearize(psc,basisParams,FxParams,tauRef,baseMass+addedMass);
    
    % Discretize the continuous, linear, path-domain model
    [Adp,Bdp] = disc(Acp,Bcp,pathStep);
    % Delete the last A and B (they would give the state vec beyond end of
    % path)
    
    Adp = delsample(Adp,'Index',numel(Adp.Time));
    Bdp = delsample(Bdp,'Index',numel(Bdp.Time));
        
    % Lift the discrete, linear, path-domain model
    [F,G] = lift(Adp,Bdp);
    
    % Build the weighting matrix for the performance index
    % Linear term
    Psiv        = spedWeight*stateSelectionMatrix(3,5,numel(Adp.Time));
    wyptArrvTms = interp1(psc.pathVar.Data,psc.Time.Data,0:pathStep:1);
    % pathVar = 0 and 1 are usually outside the range, so replace NaNs
    wyptArrvTms(1)   = 0;
    wyptArrvTms(end) = psc.Time.Data(end);
    wyptArrvDts = diff(wyptArrvTms)./psc.Time.Data(end);
    fv     = repmat(wyptArrvDts(:),[1 5])';
    fv     = fv(:)';
    % Lifted, path-domain reference signal

    r = cell(numel(Adp.Time),1);
    r(:) = {zeros(5,1)};
    for jj = 1:numel(posWayPtPathIdx)
        r(posWayPtPathIdx(jj)) = {[wyPtAzimuth(jj) wyPtElevation(jj) 0 0 0]'};
    end
    r = cell2mat(r);
    % Waypoint tracking term
    PsiW    = wyptSelectionMatrix([1 2],posWayPtPathIdx,5,numel(Adp.Time));
    QW      = wyptWeight*diag(ones(size(PsiW,1),1));
    % Input variation term
    Qu      = inptWeight*diag(ones(numel(Adp.Time),1));
    GHatj   = G'*PsiW'*QW*PsiW*G;
    % Filters
    L0 = (GHatj+Qu)^(-1);
    Lc =  L0*(1/2)*G'*Psiv*fv';
    Le =  L0*G'*PsiW*QW;
    
    % Apply learning filters to calculate deviation in input signal
    delUData = Lc+Le*(r-PsiW*psc.stateVec.Data(6:end)');
    delU = timesignal(timeseries(delUData,Adp.Time));
    
    updatePlots
    
    % Resample deviation in control signal into standard domain
    if ii == 1
        UNext = timesignal(timeseries(...
            zeros(numel(Adp.Time),1),...
            Adp.Time));
    else
        UNext = UNext.resample(delU.Time);
    end
    UNext = UNext + delU;
    UNext = UNext.rmnans;
end

