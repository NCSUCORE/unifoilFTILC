% Calculate some components of the performance index

% Speed term
Jv      = -fv*Psiv*psc.stateVec.Data(:);
JvPrime = Jv/spedWeight;

% Terminal term
rt      = zeros(size(Psit,1),1);
rt(end-4) = -wyPtAzimuth(end);
rt(end-3) =  wyPtElevation(end);
Jt        = -fv*(Psit*(psc.stateVec.Data(:) + rt));
PsitPrime = Psit;
PsitPrime(end-4,end-4) = 1;
PsitPrime(end-3,end-3) = -1;
JtPrime   = -fv*(PsitPrime*(psc.stateVec.Data(:) + rt));

% Control deviation term
Ju      = delU.Data(:)'*Qu*delU.Data(:);
JuPrime = Ju/inptWeight;

% Waypoint tracking error term
Je      = (r-PsiW*psc.stateVec.Data(:))'*QW*(r-PsiW*psc.stateVec.Data(:));
JePrime = Je/wyptWeight;

% State trajectory predicted for next iteration
xNext = psc.stateVec.Data(:) + G *delU.Data;

% Calculate predicted and actual distances tavelled
azNext  = timesignal(timeseries(xNext(1:5:end),psc.pathVar.Time));
elNext  = timesignal(timeseries(xNext(2:5:end),psc.pathVar.Time));
dadt    = azNext.diff.Data./psc.pathVar.diff.Data;
dedt    = elNext.diff.Data./psc.pathVar.diff.Data;
dPred   = radius*trapz(psc.pathVar.Data,sqrt(dadt.^2+dedt.^2));
dadt = psc.azimuth.diff.Data./psc.pathVar.diff.Data;
dedt = psc.elevation.diff.Data./psc.pathVar.diff.Data;
dLast = radius*trapz(psc.pathVar.Data,sqrt(dadt.^2+dedt.^2));

% Update flight paths
h.predicPath_iPls1.XData = xNext(1:5:end)*180/pi;
h.predicPath_iPls1.YData = xNext(2:5:end)*180/pi;
h.predicPath_iPls1Zm.XData = xNext(1:5:end)*180/pi;
h.predicPath_iPls1Zm.YData = xNext(2:5:end)*180/pi;

% Update the overall performance index
h.perfIndx.XData = [h.perfIndx.XData ii];
h.perfIndx.YData = [h.perfIndx.YData Jv+Jt+Ju+Je];
h.simTime.XData  = [h.simTime.XData  ii];
h.simTime.YData  = [h.simTime.YData  tsc.pathVar.Time(end)];

% Update the speed term
h.speedTermW.XData = [h.speedTermW.XData ii];
h.speedTermW.YData = [h.speedTermW.YData Jv];
h.speedTermU.XData = [h.speedTermU.XData ii];
h.speedTermU.YData = [h.speedTermU.YData JvPrime];

% Update the terminal incentive term
h.termTermW.XData = [h.termTermW.XData ii];
h.termTermW.YData = [h.termTermW.YData Jt];
h.termTermU.XData = [h.termTermU.XData ii];
h.termTermU.YData = [h.termTermU.YData JtPrime];

% Update the error (waypoint tracking) term
h.errTermW.XData = [h.errTermW.XData ii];
h.errTermW.YData = [h.errTermW.YData Je];
h.errTermU.XData = [h.errTermU.XData ii];
h.errTermU.YData = [h.errTermU.YData JePrime];

% Update the delta u term
h.delUTermW.XData = [h.delUTermW.XData ii];
h.delUTermW.YData = [h.delUTermW.YData Ju];
h.delUTermU.XData = [h.delUTermU.XData ii];
h.delUTermU.YData = [h.delUTermU.YData JuPrime];

% Update the control signals
h.uFBPrev.XData = tsc.pathVar.Data;
h.uFBPrev.YData = tsc.uFB.Data*180/pi;
h.uFFPrev.XData = psc.pathVar.Data;
h.uFFPrev.YData = psc.uFF.Data*180/pi;
h.uFFNext.XData = UNext.Time;
h.uFFNext.YData = UNext.Data*180/pi;

% Update the Predicted and Actual distances
h.distPred.XData = [h.distPred.XData ii+1];
h.distPred.YData = [h.distPred.YData dPred];
h.distAct.XData  = [h.distAct.XData ii];
h.distAct.YData  = [h.distAct.YData dLast];

% Update the mean speed over the last iteration
h.meanSpeed.XData = [h.meanSpeed.XData ii];
h.meanSpeed.YData = [h.meanSpeed.YData tsc.speed.mean];

% Update the dsdt and dtds plots
h.dsdt.XData = psc.pathVar.Data(1:end-1);
h.dsdt.YData = psc.dsdt.Data(1:end-1);
h.dtds.XData = psc.pathVar.Data(1:end-1);
h.dtds.YData = 1./psc.dsdt.Data(1:end-1);


drawnow