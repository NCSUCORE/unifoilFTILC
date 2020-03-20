function [Ats,Bts] = pathLinearize(tsc,...
    basisParams,...
    wingCoeffs,Aref,tauRef,mass,...
    kpw,vsp,...
    density,flowSpeed)
% The math behind this block is derived in the matlab notebook
% Reparameterization.nb.  This notebook automatically outputs .txt files
% containing matlab expressions for each element of A and B.

% Rename variables to match variable names used in mathematica
phi     = tsc.azimuth.Data(:);
theta   = tsc.elevation.Data(:);
v       = tsc.speed.Data(:);
psi     = tsc.twistAngle.Data(:);
omega   = tsc.twistRate.Data(:);
s       = tsc.pathVar.Data(:);
urfb    = tsc.urfb.Data(:);
uwfb    = tsc.uwfb.Data(:);
urff    = tsc.urff.Data(:);
uwff    = tsc.uwff.Data(:);
tau     = tauRef;
M       = mass;
rho     = density;
sigma   = flowSpeed;

phi0   = basisParams(1);
phi1   = basisParams(3);
theta0 = basisParams(2);
theta1 = basisParams(4);
r      = basisParams(5);

CL0 = wingCoeffs.CL0;
CL1 = wingCoeffs.CL1;
CD0 = wingCoeffs.CD0;
CD1 = wingCoeffs.CD1;
CD2 = wingCoeffs.CD2;


% Preallocate A and B matrices
A = zeros(5,5,numel(tsc.pathVar.Time));
B = zeros(5,2,numel(tsc.pathVar.Time));

% Find the files with the A and B matrices in them as calculated in
% Mathematica
basePath = fullfile(fileparts(which('UnifoilFTICL.prj')),'Documentation');

% Evaluate all the expressions for the elements of A
for ii = 1:5
    for jj = 1:5
        fName = sprintf('A%d%d.txt',ii,jj);
        expr  = fileread(fullfile(basePath,'MatrixElementExpressions',fName));
        try
            A(ii,jj,:) = eval(expr);
        catch
            error('Error in A%d%D expression \n %s',ii,jj,expr)
        end
    end
end

% Evaluate all the expressions for the elements of B
for ii = 1:5
    for jj = 1:2
        fName = sprintf('B%d%d.txt',ii,jj);
        expr  = fileread(fullfile(basePath,'MatrixElementExpressions',fName));
        try
            B(ii,jj,:) = eval(expr);
        catch
            error('Error in B%d%D expression \n %s',ii,jj,expr)
        end
        
    end
end

% Store everything into a timesignal for output
Ats = timesignal(timeseries(A,tsc.stateVec.Time));
Bts = timesignal(timeseries(B,tsc.stateVec.Time));
end
