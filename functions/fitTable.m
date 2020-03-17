function C = fitTable(table,alphaRange)

data = [table.alpha(:) table.CL(:) table.CD(:)];

rows = and(table.alpha<=alphaRange(2),table.alpha>=alphaRange(1));

data = data(rows,:,:);
C.CDCoeffs = polyfit(data(:,1),data(:,3),2);
C.CLCoeffs = polyfit(data(:,1),data(:,2),1);
CLCoeffs = flip(C.CLCoeffs);
CDCoeffs = flip(C.CDCoeffs);
for ii = 1:numel(CLCoeffs)
    eval(sprintf('C.CL%d = CLCoeffs(%d);',ii-1,ii))
end

for ii = 1:numel(CDCoeffs)
    eval(sprintf('C.CD%d = CDCoeffs(%d);',ii-1,ii))
end


end