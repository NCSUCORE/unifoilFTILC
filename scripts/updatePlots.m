% Calculate some components of the performance index

% Actual performance index components
% Speed term
JvAct  = -fv*Psiv*psc.stateVec.Data(6:end)';
% Control deviation term
JuAct  = delU.Data(:)'*Qu*delU.Data(:);
% Waypoint tracking error term
JeAct  = (r-PsiW*psc.stateVec.Data(6:end)')'*QW*(r-PsiW*psc.stateVec.Data(6:end)');

% Predicted performance over next iteration
% State trajectory predicted for next iteration
xNext  = psc.stateVec.Data(:) + [zeros(5,1) ; G *delU.Data];
% Speed term
JvPred = -fv*Psiv*xNext(6:end);
% Control deviation term
JuPred = delU.Data(:)'*Qu*delU.Data(:);
% Waypoint tracking error term
JePred = (r-PsiW*xNext(6:end))'*QW*(r-PsiW*xNext(6:end));

% Calculate predicted and actual distances tavelled
dPred = arcLength(xNext(1:5:end),xNext(2:5:end),radius);
dPred = dPred(end);
dLast = arcLength(tsc.azimuth.Data,tsc.elevation.Data,radius);
dLast = dLast(end);

% Update flight paths
h.predicPath_iPls1.XData = xNext(1:5:end)*180/pi;
h.predicPath_iPls1.YData = xNext(2:5:end)*180/pi;
h.predicPath_iPls1Zm.XData = xNext(1:5:end)*180/pi;
h.predicPath_iPls1Zm.YData = xNext(2:5:end)*180/pi;

% Update the overall performance index
h.perfIndxAct.XData = [h.perfIndxAct.XData ii];
h.perfIndxAct.YData = [h.perfIndxAct.YData JvAct+JuAct+JeAct];
h.perfIndxPred.XData = [h.perfIndxPred.XData ii+1];
h.perfIndxPred.YData = [h.perfIndxPred.YData JvPred+JuPred+JePred];

% Update the speed term
h.speedTermAct.XData = [h.speedTermAct.XData ii];
h.speedTermAct.YData = [h.speedTermAct.YData JvAct];
h.speedTermPred.XData = [h.speedTermPred.XData ii+1];
h.speedTermPred.YData = [h.speedTermPred.YData JvPred];

% Update the error (waypoint tracking) term
h.errTermAct.XData  = [h.errTermAct.XData ii];
h.errTermAct.YData  = [h.errTermAct.YData JeAct];
h.errTermPred.XData = [h.errTermPred.XData ii+1];
h.errTermPred.YData = [h.errTermPred.YData JePred];

% Update the delta u term
h.delUTermAct.XData  = [h.delUTermAct.XData ii];
h.delUTermAct.YData  = [h.delUTermAct.YData JuAct];
h.delUTermPred.XData = [h.delUTermPred.XData ii+1];
h.delUTermPred.YData = [h.delUTermPred.YData JuPred];

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
h.meanSpeedAct.XData = [h.meanSpeedAct.XData ii];
h.meanSpeedAct.YData = [h.meanSpeedAct.YData tsc.speed.mean];
% h.meanSpeedPred.XData = [h.meanSpeedPred.XData ii+1];
% h.meanSpeedPred.YData = [h.meanSpeedPred.YData mean(xNext(3:5:end)./ds)];

% Update predicted and actual speed profiles
h.speedProfPred.XData = psc.pathVar.Data;
h.speedProfPred.YData = xNext(3:5:end);
h.speedProfAct.XData = psc.speed.Time;
h.speedProfAct.YData = psc.speed.Data;

% Update simulation time plot
h.simTime.XData  = [h.simTime.XData  ii];
h.simTime.YData  = [h.simTime.YData  tsc.pathVar.Time(end)];

drawnow