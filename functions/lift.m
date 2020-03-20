function [F,G] = lift(A,B)
% function gives F and G in the (equivalent) expressions
% deltax = F*deltax0 + G*deltau
% x = x0 + F*deltax0 + G*deltau
% where x0 is the full, lifted state trajectory, (column state vector at
% every step stacked vertically) and deltax0 is the deviation in initial
% conditions between iterations so if the system has N states, deltax0 is N
% by 1.
% inputs A and B are 3D arrays of A and B matrices where the third index
% runs along the parameterization variable.

% Pull out the data
A = A.Data;
B = B.Data;

% Get the number of steps
nSteps  = size(A,3);
nStates = size(A,1);

% Get the dimensions of the sub-blocks in the block matrices
% FblkDims = size(A(:,:,1));
GblkDims = size(A(:,:,1)*B(:,:,1));
FblkDims = size(A(:,:,1));

% Preallocate F as cell array, each cell is 1 block of block matrix
F       =  cell(nSteps,1);
F(:)    =  {zeros(FblkDims)};

% Preallocate G as cell array, each cell is 1 block of block matrix
G       =  cell(nSteps-1,nSteps-1);
G(:)    = {zeros(GblkDims)};

% Set the first elements of F and G
F{1}    = A(:,:,1);
G{1,1}  = B(:,:,1);

for ii = 2:nSteps-1 % Run each row
    % Set the next element of F
    F{ii} = A(:,:,ii)*F{ii-1};
    % Set diagonal element of G
    G{ii,ii} = B(:,:,ii); 
    % Set the elements to the left of the one-below-diagonal element of G
    for jj = 1:ii-1 % run over columns
       G{ii,jj} = A(:,:,ii-1)*G{ii-1,jj};
    end
end
F = cell2mat(F);
G = cell2mat(G);
end

