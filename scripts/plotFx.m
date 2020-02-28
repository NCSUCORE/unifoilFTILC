close all

t = tsc.twistAngle.Data(:);
a = tsc.azimuth.Data(:);
s = tsc.speed.Data(:);
Fx = tsc.Fx.Data(:);
tMean = mean(t);
aMean = mean(a);
sMin  = min(s);
tAmp  = max(sqrt((t-tMean).^2));
aAmp  = max(sqrt((a-aMean).^2));
sAmp  = max(s)-min(s);

tNorm = (t-tMean)/tAmp;
aNorm = (a-aMean)/aAmp;
phase = atan(aNorm./tNorm);

FxMin = min(Fx);
FxAmp = max(Fx)-min(Fx);

sA  = sAmp*cos(phase+20*pi/180).^2+sMin;
FxA = FxAmp*cos(phase+60*pi/180).^2+FxMin;


subplot(2,1,1)
colormap hot
scatter3(t,a,s,4,tsc.Fx.Data,...
    'MarkerFaceColor','flat')
colorbar
grid on
hold on
scatter3(t,a,sA,4,FxA,...
    'MarkerFaceColor','flat')
xlabel('Twist')
ylabel('Azimuth')
zlabel('Speed')
set(gca,'FontSize',18)

subplot(2,1,2)
scatter3(t,a,tsc.Fx.Data,...
    'MarkerFaceColor','flat')
grid on
hold on
scatter3(t,a,FxA,...
    'MarkerFaceColor','flat')
xlabel('Twist')
ylabel('Azimuth')
zlabel('Fx')
set(gca,'FontSize',18)