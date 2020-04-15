close all
azimuthSweep    = 90*pi/180; % Path azimuth sweep angle, degrees
elevationSweep  = 15*pi/180; % Path elevation sweep angle, degrees
meanAzimuth     = 0*pi/180;
meanElevation   = 40*pi/180;
radius          = 100;
basisParams     = [azimuthSweep, elevationSweep, meanAzimuth, meanElevation, radius];

% Reference Signal
pathPosVecs = lemOfGerono(linspace(0,1,1000),basisParams);

plot3(pathPosVecs(:,1),pathPosVecs(:,2),pathPosVecs(:,3),...
    'LineWidth',2,'Color','k')

grid off
hold on
[X,Y,Z] = sphere(100);
h.sphere = surf(radius*X,radius*Y,radius*Z,...
    'EdgeColor','none',...
    'FaceAlpha',0.5);
scatter3(0,0,0,'Marker','.','CData',[0 0 0])

xlim([0 radius])
ylim(radius*[-1 1]);
zlim([0 radius])
daspect([1 1 1])
view( -80,-14)
colormap(gray)
h.sphere.CData = ones(size(h.sphere.CData));

axis off

[posVec,tanVec] = lemOfGerono(0.55,basisParams);

plot3([0 posVec(1)],[0 posVec(2)],[0 posVec(3)],'Color','k','LineWidth',2)
plot3([0 50],[0 0],[0 0],'Color','k','LineWidth',2)
plot3([0 0],[0 -25],[0 0],'Color','k','LineWidth',2)
plot3([0 0],[0 0],[0 25],'Color','k','LineWidth',2)
plot3(posVec(1)*[1 1],posVec(2)*[1 1],posVec(3)*[1 0],'Color',0.5*[1 1 1],'LineWidth',2,'LineStyle','--')
plot3(posVec(1)*[0 1],posVec(2)*[0 1],posVec(3)*[0 0],'Color',0.5*[1 1 1],'LineWidth',2,'LineStyle','--')

circRad = sqrt(posVec(1)^2+posVec(2)^2);

s = linspace(0,2*pi,1000);
xCirc = circRad*cos(s);
yCirc = circRad*sin(s);

zCirc = posVec(3)*ones(size(xCirc));
plot3(xCirc,yCirc,zCirc,'Color',0.25*[1 1 1],'LineWidth',2,'LineStyle','--')