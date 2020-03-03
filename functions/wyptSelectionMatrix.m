function Gamma = wyptSelectionMatrix(stateIdx,wyptIdx,nStates,nSteps)
% Matrix to pick off specified state value at every time step
Gamma = cell(nSteps,nSteps);
% Set all block elements to zero sub matrics
Gamma(:) = {zeros(nStates,nStates)};
% Set block diagonal elements of block matrix to new sub matrix
gamma = zeros(1,nStates);
gamma(stateIdx) = 1;
for ii = 1:numel(wyptIdx)
    Gamma(wyptIdx(ii),wyptIdx(ii)) = {diag(gamma)};
end
% Convert cell block matrix to matrix
Gamma = cell2mat(Gamma);
end