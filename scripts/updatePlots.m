

%% Path Shape
h.pathPrev.XData = xPrev(1:nx:end)*180/pi;
h.pathPrev.YData = xPrev(2:nx:end)*180/pi;
h.pathNext.XData = xNext(1:nx:end)*180/pi;
h.pathNext.YData = xNext(2:nx:end)*180/pi;

%% Distance, Time and Velocity
h.distPrev.XData = [h.distPrev.XData ii];
h.distPrev.YData = [h.distPrev.YData arcLength(xPrev(1:nx:end),xPrev(2:nx:end),radius)];
h.distNext.XData = [h.distNext.XData ii+1];
h.distNext.YData = [h.distNext.YData arcLength(xNext(1:nx:end),xNext(2:nx:end),radius)];

h.vAvgPrevTime.XData = [h.vAvgPrevTime.XData ii];
h.vAvgPrevTime.YData = [h.vAvgPrevTime.YData tsc.speed.mean];
h.vAvgPrevPath.XData = [h.vAvgPrevPath.XData ii];
h.vAvgPrevPath.YData = [h.vAvgPrevPath.YData psc.speed.mean];

h.timePrev.XData = [h.timePrev.XData ii];
h.timePrev.YData = [h.timePrev.YData tsc.pathVar.Time(end)];

%% Performance index plots
h.perfIndxPrev.XData = [h.perfIndxPrev.XData ii];
h.perfIndxPrev.YData = [h.perfIndxPrev.YData JPrev];
h.perfIndxNext.XData = [h.perfIndxNext.XData ii+1];
h.perfIndxNext.YData = [h.perfIndxNext.YData JNext];

h.JsxPrev.XData = [h.JsxPrev.XData ii];
h.JsxPrev.YData = [h.JsxPrev.YData JsxPrev];
h.JsxNext.XData = [h.JsxNext.XData ii+1];
h.JsxNext.YData = [h.JsxNext.YData JsxNext];

h.JuPrev.XData = [h.JuPrev.XData ii];
h.JuPrev.YData = [h.JuPrev.YData JuPrev];
h.JuNext.XData = [h.JuNext.XData ii+1];
h.JuNext.YData = [h.JuNext.YData JuNext];
 
h.JduPrev.XData = [h.JduPrev.XData ii];
h.JduPrev.YData = [h.JduPrev.YData JduPrev];
h.JduNext.XData = [h.JduNext.XData ii+1];
h.JduNext.YData = [h.JduNext.YData JduNext];

h.JxPrev.XData = [h.JxPrev.XData ii];
h.JxPrev.YData = [h.JxPrev.YData JxPrev];
h.JxNext.XData = [h.JxNext.XData ii+1];
h.JxNext.YData = [h.JxNext.YData JxNext];

h.JdxPrev.XData = [h.JdxPrev.XData ii];
h.JdxPrev.YData = [h.JdxPrev.YData JdxPrev];
h.JdxNext.XData = [h.JdxNext.XData ii+1];
h.JdxNext.YData = [h.JdxNext.YData JdxNext];

h.JePrev.XData = [h.JePrev.XData ii];
h.JePrev.YData = [h.JePrev.YData JePrev];
h.JeNext.XData = [h.JeNext.XData ii+1];
h.JeNext.YData = [h.JeNext.YData JeNext];

h.JdePrev.XData = [h.JdePrev.XData ii];
h.JdePrev.YData = [h.JdePrev.YData JdePrev];
h.JdeNext.XData = [h.JdeNext.XData ii+1];
h.JdeNext.YData = [h.JdeNext.YData JdeNext];





drawnow


