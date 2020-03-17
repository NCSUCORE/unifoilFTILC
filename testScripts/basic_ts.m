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
maxLeadLength   = 0.05;
maxIntAngle     = 10*pi/180;
% Min and max alpha for the wing controller
wingAlphaPlusStall  =  10*pi/180;
wingAlphaMinusStall = -10*pi/180;
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
initSpeed       = 5.735;
initAzimuth     = meanAzimuth;
initElevation   = meanElevation;
initTwist       = -33*pi/180;
initTwistRate   = 0;
initPathVar     = 0;

sim('unifoil')
tscOpt = signalcontainer(logsout);

% wingCtrlSwitch = 2;
% sim('unifoil')
% tscSubOpt = signalcontainer(logsout);

%%
figure
tscOpt.speed.plot
%% 
% tscOpt.pathVar.Time(end)
% tscSubOpt.pathVar.Time(end)
% 
% %%
% figure
% plot(tscOpt.azimuth.Data,tscOpt.elevation.Data)
% grid on
% hold on
% plot(tscSubOpt.azimuth.Data,tscSubOpt.elevation.Data)
% 
% %%
% figure
% tscOpt.wingFx.plot
% grid on
% hold on
% tscSubOpt.wingFx.plot
% 
% %%
% figure
% tscSubOpt.wingCmdPreSat.plot
% grid on
% hold on
% tscSubOpt.wingCmd.plot
% 
% %%
% figure
% tscOpt.speed.plot
% grid on
% hold on
% tscSubOpt.speed.plot

%%
% figure
% subplot(2,1,1)
% tscOpt.FLxPred.plot
% grid on
% hold on
% tscOpt.FLxAct.plot
% 
% subplot(2,1,2)
% tscOpt.FDxPred.plot
% grid on
% hold on
% tscOpt.FDxAct.plot
