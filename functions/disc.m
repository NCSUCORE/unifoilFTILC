function [Ad,Bd] = disc(Ac,Bc,steps)
Ad = Ac;
Bd = Bc;
CMat = eye(size(Ac.getdatasamples(1)));
DMat = zeros(size(Bc.getdatasamples(1)));
stepSizes = diff(steps);
for ii = 1:numel(Ac.Time)-1
    sysTmp = c2d(ss(Ac.Data(:,:,ii),Bc.Data(:,:,ii),CMat,DMat),stepSizes(ii));
    Ad.Data(:,:,ii) = sysTmp.A;
    Bd.Data(:,:,ii) = sysTmp.B;
end
end