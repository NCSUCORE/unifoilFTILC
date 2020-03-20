function Gamma = wyptSelectionMatrix(wyptIdx,nStates,nSteps)
% Preallocate cell array of input arguments for blkdiag command later
args = cell(1,nSteps);
% Make all the elements zero matrices
args(:) = {zeros(nStates,nStates)};
% Make the ones corresponding to waypoints identity matrices
args(wyptIdx) = {eye(nStates)};
% Call the blkdiag command
Gamma = blkdiag(args{:});
end