function psc = processTSC(tsc,pathStep)
% tsc = timeseries container
% psc = path-domain series container

% Resample all signals to match time vector of path variable
sigNames = tsc.getpropsexcept({'pathVar','metaData'});
for ii = 1:numel(sigNames)
    tsc.(sigNames{ii}) = tsc.(sigNames{ii}).resample(tsc.pathVar.Time);
end

% Crop to first lap
times   = tsc.lapNum.Time(tsc.lapNum.Data==1);
tsc = tsc.crop([times(1),times(end-1)]);

% Remove samples corresponding to non monotonically increasing path steps
nonMonInd = find(diff(tsc.pathVar.Data)<=0);
sigNames  = tsc.getpropsexcept('metaData');
while ~isempty(nonMonInd)
    for ii = 1:numel(sigNames)
        tsc.(sigNames{ii}) = tsc.(sigNames{ii}).delsample('Index',nonMonInd);
    end
    nonMonInd = find(diff(tsc.pathVar.Data)<=0);
end

tsc.addprop('Time');
tsc.Time = timesignal(timeseries(tsc.pathVar.Time,tsc.pathVar.Time));

% Create path-parameterized psc
psc  = signalcontainer(tsc);

% Overwrite all time vectors with path variable and resample
sigNames = psc.getpropsexcept('metaData');
for ii = 1:numel(sigNames)
    psc.(sigNames{ii}).Time = psc.pathVar.Data;
end
for ii = 1:numel(sigNames)
    psc.(sigNames{ii}) = psc.(sigNames{ii}).resample(pathStep);
%     psc.(sigNames{ii}) = psc.(sigNames{ii}).interpNaNs;
end

end