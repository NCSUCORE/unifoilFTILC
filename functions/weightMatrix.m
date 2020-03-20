function W = weightMatrix(weightVec,nSteps)
diagElems = cell(1,nSteps);
diagElems(:) = {diag(weightVec)};
W = blkdiag(diagElems{:});
end