
if ii == 1 
    h.achievedPath1Plot.XData = xPrev(1:nx:end)*180/pi;
    h.achievedPath1Plot.YData = xPrev(2:nx:end)*180/pi;
end

if ii == numIters
    h.achievedPathEndPlot.XData = xPrev(1:nx:end)*180/pi;
    h.achievedPathEndPlot.YData = xPrev(2:nx:end)*180/pi;
end

h.performancePlot.XData = [h.performancePlot.XData ii];
h.performancePlot.YData = [h.performancePlot.YData JPrev];

h.waypointTrackingPlot.XData = [h.waypointTrackingPlot.XData ii];
h.waypointTrackingPlot.YData = [h.waypointTrackingPlot.YData JePrev];

h.economicTermScat.XData  = [h.economicTermScat.XData ii];
h.economicTermScat.YData  = [h.economicTermScat.YData JsxPrev];

h.meanSpeedScat.XData = [h.meanSpeedScat.XData ii];
h.meanSpeedScat.YData = [h.meanSpeedScat.YData meanSpeed];

h.iterDurScat.XData = [h.iterDurScat.XData ii];
h.iterDurScat.YData = [h.iterDurScat.YData iterDur];

drawnow
