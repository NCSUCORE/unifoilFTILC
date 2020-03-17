function [FxParams,avgErr] = fitFxPhase(psc)
% Calculate parameters to fit the Fx function
FxParams.a = max(psc.Fx.Data(:)) - min(psc.Fx.Data(:)); % a in Mathematica
FxParams.b = mean(psc.twistAngle.Data(:)); % b in Mathematica
FxParams.c = max(psc.twistAngle.Data(:)) - min(psc.twistAngle.Data(:)); % c in Mathematica
FxParams.d = mean(psc.azimuth.Data(:)); % d in Mathematica
FxParams.f = max(psc.azimuth.Data(:)) - min(psc.azimuth.Data(:)); %f in Mathematica
FxParams.h = min(psc.Fx.Data(:)); % h in Mathematica

t = psc.twistAngle.Data(:);
a = psc.azimuth.Data(:);

Fx = psc.Fx.Data(:);

tNorm = (t-FxParams.b)/FxParams.c;
aNorm = (a-FxParams.d)/FxParams.f;
phase = atan(aNorm./tNorm);
shifts = linspace(-pi,pi,numel(Fx))';


FxA   = @(shft)FxParams.a*cos(phase(:)'+shft(:)).^2+FxParams.h;
FxErr = @(shft)sum((Fx(:)-FxA(shft(:))).^2,1);
errs = FxErr(shifts);
[minErr,idx] = min(errs);
avgErr = sqrt(minErr)./numel(Fx);


% sqrt(sum(FxErr(shifts(idx))))./numel(Fx)
% FxParams.g = shifts(idx);
% 
% figure('Position',[ -0.9995    0.0380    0.4990    0.8833])
% subplot(2,1,1)
% scatter3(t,a,Fx,...
%     'MarkerFaceColor','flat')
% grid on
% hold on
% scatter3(t,a,FxA(FxParams.g),...
%     'MarkerFaceColor','flat')
% xlabel('Twist')
% ylabel('Azimuth')
% zlabel('Fx')
% set(gca,'FontSize',18)
% 
% subplot(2,1,2)
% plot(shifts,sqrt(errs))
% grid on
% hold on
% xlabel('Phase Shift')
% ylabel('Err [N]')
% set(gca,'FontSize',18)

end