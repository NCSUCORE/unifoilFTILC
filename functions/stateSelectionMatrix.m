function Gamma = stateSelectionMatrix(indx,nStates)
% Preallocate cell array of input arguments for blkdiag command later
args = cell(1,nSteps);
% Form the diagonal of the block elements
diagEntry = zeros(1,nStates);
diagEntry(indx) = 1;
% Make the diagonal block elements
args(:) = {diag(diagEntry)};
% Call the blkdiag command
Gamma = blkdiag(args{:});
end