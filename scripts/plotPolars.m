h.polarFig = figure;

subplot(2,3,1)
plot(wingTable.alpha*180/pi,wingTable.CL)
grid on
hold on
plot(wingTable.alpha*180/pi,polyval(wingC.CLCoeffs,wingTable.alpha))

subplot(2,3,2)
plot(wingTable.alpha*180/pi,wingTable.CD)
grid on
hold on
plot(wingTable.alpha*180/pi,polyval(wingC.CDCoeffs,wingTable.alpha))

subplot(2,3,3)
plot(wingTable.alpha*180/pi,wingTable.CL./wingTable.CD)
grid on
hold on
plot(wingTable.alpha*180/pi,polyval(wingC.CLCoeffs,wingTable.alpha)./polyval(wingC.CDCoeffs,wingTable.alpha))
