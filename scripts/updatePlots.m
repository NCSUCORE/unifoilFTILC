% Actual state trajectory from previous iteration
xActPrev = psc.stateVec.Data(nx+1:end)';

% Actual values of performance index over last iteration
JlxAct  = Lx*xActPrev;
JqxAct  = (r-xActPrev)'*Qe*(r-xActPrev);
JluAct  = Lu*uFFPrev;
JquAct  = (uFFNext-uFFPrev)'*Qu*(uFFNext-uFFPrev);

% State trajectory predicted for next iteration
xNextPred   = psc.stateVec.Data(nx+1:end)' + G*(uFFNext(:)-uFFPrev(:));

JlxPred     = Lx*xNextPred;
JqxPred     = (r-xNextPred)'*Qe*(r-xNextPred);
JluPred     = Lu*uFFNext;
JquPred     = (uFFNext-uFFPrev)'*Qu*(uFFNext-uFFPrev);

% Update the overall performance index
h.perfIndxAct.XData  = [h.perfIndxAct.XData ii];
h.perfIndxAct.YData  = [h.perfIndxAct.YData JlxAct+JluAct+JqxAct+JquAct];
h.perfIndxPred.XData = [h.perfIndxPred.XData ii+1];
h.perfIndxPred.YData = [h.perfIndxPred.YData JlxPred+JluPred+JqxPred+JquPred];

% Update total simulation time plot
h.simTime.XData  = [h.simTime.XData  ii];
h.simTime.YData  = [h.simTime.YData  tsc.pathVar.Time(end)];

% Update the linear state term
h.lxAct.XData   = [h.lxAct.XData    ii];
h.lxAct.YData   = [h.lxAct.YData    JlxAct];
h.lxPred.XData  = [h.lxPred.XData   ii+1];
h.lxPred.YData  = [h.lxPred.YData   JlxPred];

% Update the quadratic state term
h.qxAct.XData  = [h.qxAct.XData ii];
h.qxAct.YData  = [h.qxAct.YData JqxAct];
h.qxPred.XData = [h.qxPred.XData ii+1];
h.qxPred.YData = [h.qxPred.YData JqxPred];

% Update the linear input term
h.luAct.XData  = [h.luAct.XData ii];
h.luAct.YData  = [h.luAct.YData JluAct];
h.luPred.XData = [h.luPred.XData ii+1];
h.luPred.YData = [h.luPred.YData JluPred];

% Update the quadratic input term
h.quAct.XData  = [h.quAct.XData ii];
h.quAct.YData  = [h.quAct.YData JquAct];
h.quPred.XData = [h.quPred.XData ii+1];
h.quPred.YData = [h.quPred.YData JquPred];

% Flight paths
h.actualPath_iMin1.XData = h.actualPath_i.XData;
h.actualPath_iMin1.YData = h.actualPath_i.YData;
h.actualPath_i.XData = psc.azimuth.Data(:)'*180/pi;
h.actualPath_i.YData = psc.elevation.Data(:)'*180/pi;
h.predicPath_iPls1.XData = xNextPred(1:nx:end)*180/pi;
h.predicPath_iPls1.YData = xNextPred(2:nx:end)*180/pi;

% Wing control input
h.uw.XData = psc.uw.Time;
h.uw.YData = psc.uw.Data*180/pi;
h.uwfb.XData = psc.uwfb.Time;
h.uwfb.YData = psc.uwfb.Data*180/pi;
h.uwffPrev.XData = psc.uwff.Time;
h.uwffPrev.YData = psc.uwff.Data*180/pi;
h.uwffNext.XData = Ss(1:end-1);
h.uwffNext.YData = uFFNext(1:nu:end)*180/pi;

% Rudder control input
h.ur.XData = psc.ur.Time;
h.ur.YData = psc.ur.Data*180/pi;
h.urfb.XData = psc.urfb.Time;
h.urfb.YData = psc.urfb.Data*180/pi;
h.urffPrev.XData = psc.urff.Time;
h.urffPrev.YData = psc.urff.Data*180/pi;
h.urffNext.XData = urFFNext.Time;
h.urffNext.YData = urFFNext.Data*180/pi;

% Update the distances
h.distPred.XData    = [h.distPred.XData ii+1];
h.distPred.YData    = [h.distPred.YData arcLength(xNextPred(1:nx:end),xNextPred(2:nx:end),radius)];
h.distAct.XData     = [h.distAct.XData ii];
h.distAct.YData     = [h.distAct.YData arcLength(tsc.azimuth.Data(:),tsc.elevation.Data(:),radius)];

% Update the average speed
h.vAvgTime.XData = [h.vAvgTime.XData ii];
h.vAvgTime.YData = [h.vAvgTime.YData tsc.speed.mean];
h.vAvgPath.XData = [h.vAvgPath.XData ii];
h.vAvgPath.YData = [h.vAvgPath.YData psc.speed.mean];

h.title.String = sprintf('Iteration %d',ii);

drawnow

% Update figure 2
h.phiPrev.XData = psc.pathVar.Data;
h.phiPrev.YData = psc.azimuth.Data*180/pi;
h.phiPred.XData = Ss(2:end);
h.phiPred.YData = xNextPred(1:nx:end)*180/pi;
h.dphiPred.XData = Ss(2:end);
h.dphiPred.YData = (xNextPred(1:nx:end)-xActPrev(1:nx:end))*180/pi;

h.thetaPrev.XData = psc.pathVar.Data;
h.thetaPrev.YData = psc.elevation.Data*180/pi;
h.thetaPred.XData = Ss(2:end);
h.thetaPred.YData = xNextPred(2:nx:end)*180/pi;
h.dthetaPred.XData = Ss(2:end);
h.dthetaPred.YData = (xNextPred(2:nx:end)-xActPrev(2:nx:end))*180/pi;

h.vPrev.XData = psc.pathVar.Data;
h.vPrev.YData = psc.speed.Data;
h.vPred.XData = Ss(2:end);
h.vPred.YData = xNextPred(3:nx:end);
h.dvPred.XData = Ss(2:end);
h.dvPred.YData = (xNextPred(3:nx:end)-xActPrev(3:nx:end));

h.psiPrev.XData = psc.pathVar.Data;
h.psiPrev.YData = psc.twistAngle.Data*180/pi;
h.psiPred.XData = Ss(2:end);
h.psiPred.YData = xNextPred(4:nx:end)*180/pi;
h.dpsiPred.XData = Ss(2:end);
h.dpsiPred.YData = (xNextPred(4:nx:end)-xActPrev(4:nx:end))*180/pi;

h.omegaPrev.XData = psc.pathVar.Data;
h.omegaPrev.YData = psc.twistRate.Data*180/pi;
h.omegaPred.XData = Ss(2:end);
h.omegaPred.YData = xNextPred(5:nx:end)*180/pi;
h.domegaPred.XData = Ss(2:end);
h.domegaPred.YData = (xNextPred(5:nx:end)-xActPrev(5:nx:end))*180/pi;

drawnow

h.uw.XData = psc.pathVar.Data;
h.uw.YData = psc.uw.Data*180/pi;
h.uwOpt.XData = psc.pathVar.Data;
h.uwOpt.YData = psc.uwOpt.Data*180/pi;


h.gammaW.XData = psc.pathVar.Data;
h.gammaW.YData = psc.wingGamma.Data*180/pi;
h.gammaR.XData = psc.pathVar.Data;
h.gammaR.YData = psc.rudderGamma.Data*180/pi;

h.wingAlpha.XData = psc.pathVar.Data;
h.wingAlpha.YData = psc.alphaWing.Data*180/pi;
h.wingAlphaOpt.XData = psc.pathVar.Data;
h.wingAlphaOpt.YData = psc.alphaWingOpt.Data*180/pi;

drawnow

h.fxAct.XData = psc.pathVar.Data;
h.fxAct.YData = psc.Fx.Data;
h.fxAct.DisplayName = sprintf('$F_{x,%d}^{Act}$',ii) ;

h.fxPred.XData = Ss(2:end-1);
h.fxPred.YData = (diff(xNextPred(3:nx:end))./diff(psc.Time.Data(1:end-1))).*(baseMass+addedMass);
h.fxPred.DisplayName = sprintf('$F_{x,%d}^{Pred}$',ii+1) ;
drawnow

