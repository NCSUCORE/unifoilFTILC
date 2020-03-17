function Gamma = stateSelectionMatrix(indx,nStates,nRows)
% Matrix to pick off specified state value at every time step
Gamma = cell(nRows,nRows);
% Set all block elements to zero sub matrics
Gamma(:) = {zeros(nStates,nStates)};
% Set block diagonal elements of block matrix to new sub matrix
gamma = zeros(1,nStates);
gamma(indx) = 1;
Gamma(1:nRows+1:(nRows)^2) = {diag(gamma)};
% Convert cell block matrix to matrix
Gamma = cell2mat(Gamma);
end