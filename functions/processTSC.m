function psc = processTSC(tsc,pathVec)
% tsc = timeseries container
% psc = path-domain series container

% Add time as a signal
tsc.addprop('Time');
tsc.Time = timesignal(timeseries(tsc.pathVar.Time,tsc.pathVar.Time));


% Resample all signals to match time vector of path variable
sigNames = tsc.getpropsexcept({'pathVar','metaData'});
for ii = 1:numel(sigNames)
    tsc.(sigNames{ii}) = tsc.(sigNames{ii}).resample(tsc.pathVar.Time);
end

% Crop to first lap + 1 time step
idx   = [find(tsc.lapNum.Data==1,1) find(tsc.lapNum.Data==2,1)];
tsc = tsc.crop([tsc.pathVar.Time(idx(1)) tsc.pathVar.Time(idx(2))]);

% Undo wrapping of path variable at last point
tsc.pathVar.Data(end) = tsc.pathVar.Data(end) + 1;

% Remove samples corresponding to non monotonically increasing path steps
nonMonInd = find(diff(tsc.pathVar.Data)<=0);
sigNames  = tsc.getpropsexcept('metaData');
while ~isempty(nonMonInd)
    for ii = 1:numel(sigNames)
        tsc.(sigNames{ii}) = tsc.(sigNames{ii}).delsample('Index',nonMonInd);
    end
    nonMonInd = find(diff(tsc.pathVar.Data)<=0);
end


% Find time stamps to resample to
tVec = interp1(tsc.pathVar.Data,tsc.pathVar.Time,pathVec);
% Resample all signals to that time vector
for ii = 1:numel(sigNames)
    tsc.(sigNames{ii}) = tsc.(sigNames{ii}).resample(tVec);
end
% Overwrite path var b/c interp 1 only gets w/in 1e-16
tsc.pathVar.Data = reshape(pathVec,size(tsc.pathVar.Data));

% Create path-parameterized psc
psc  = signalcontainer(tsc);

% Overwrite all time vectors with path variable and resample
sigNames = psc.getpropsexcept('metaData');
for ii = 1:numel(sigNames)
    psc.(sigNames{ii}).Time = psc.pathVar.Data;
end

end