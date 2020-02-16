function [tscOut,tscPath] = cleanTSC(tscIn,pathStep)
tscOut = tscIn;
sigNames = properties(tscOut);

% get all the timesignal names (basically just drop metadata)
sigNames = sigNames(cellfun(@(x)isa(tscOut.(x),'timesignal'),properties(tscOut)));

% Crop to first lap
% Find end time
idx = find(and(tscIn.pathVar.Data(1:end-1)>0.9,tscIn.pathVar.Data(2:end)<0.1),2);
switch numel(idx)
    case 0
        tStart = tscIn.pathVar.Time(1);
        tEnd   = tscIn.pathVar.Time(end);
    case 1
        if idx<0.5*numel(tscIn.pathVar.Data)
            tStart = tscIn.pathVar.Time(idx);
            tEnd   = tscIn.pathVar.Time(end-1);
        else
            tStart = tscIn.pathVar.Time(1);
            tEnd   = tscIn.pathVar.Time(idx-1);
        end
    case 2
        tStart = tscIn.pathVar.Time(idx(1));
        tEnd   = tscIn.pathVar.Time(idx(2)-1);
    otherwise
end

tscOut = tscIn.crop([tStart tEnd]);

% Remove samples corresponding to non monotonically increasing path steps
nonMonInd = find(diff(tscOut.pathVar.Data)<=0);
if any(nonMonInd)
    for ii = 1:numel(sigNames)
        tscOut.(sigNames{ii}) = tscOut.(sigNames{ii}).delsample('Index',nonMonInd);
    end
end


tscPath = tscOut;
% Remove samples corresponding to non monotonically increasing path steps
nonMonInd = find(diff(tscPath.pathVar.Data)<=0);
if any(nonMonInd)
    for ii = 1:numel(sigNames)
        tscPath.(sigNames{ii}) = tscPath.(sigNames{ii}).delsample('Index',nonMonInd);
    end
end

% Resample each signal to match the time vector of the path variable
nonPathVarSigs = sigNames(cellfun(@(x)~strcmp(x,'pathVar'),sigNames));
for ii = 1:numel(nonPathVarSigs)
    tscPath.(nonPathVarSigs{ii}) = tscPath.(nonPathVarSigs{ii}).resample(tscPath.pathVar.Time);
end

% Resample each signal to match the time vector of the path variable
nonPathVarSigs = sigNames(cellfun(@(x)~strcmp(x,'pathVar'),sigNames));
for ii = 1:numel(nonPathVarSigs)
    tscPath.(nonPathVarSigs{ii}) = tscPath.(nonPathVarSigs{ii}).resample(tscPath.pathVar.Time);
end


% Reparameterize signals
tscPath = tscOut;
for ii = 1:numel(sigNames)
    try
    tscPath.(sigNames{ii}).Time = tscPath.pathVar.Data;
    catch
       x = 1; 
    end
end

% Resample all signals to the specified path step
warning('off','MATLAB:linearinter:noextrap')
tscPath = tscPath.resample(0:pathStep:1);
warning('on','MATLAB:linearinter:noextrap')

% Interpolate NaN's
for ii = 1:numel(sigNames)
    tscPath.(sigNames{ii}) = tscPath.(sigNames{ii}).interpNaNs;
end

end