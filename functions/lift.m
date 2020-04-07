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
GblkDims = size(A(:,:,1)*B(:,:,1));
FblkDims = size(A(:,:,1));

% Preallocate F as cell array, each cell is 1 block of block matrix
F       =  cell(nSteps,1);

% Preallocate G as cell array, each cell is 1 block of block matrix
G       =  cell(nSteps,nSteps);
G(:)    = {zeros(GblkDims)};

% Set the first element
F{1}    = eye(nStates);

% Create F
for ii = 2:nSteps
    % Premultiply previous row by A at current path step
   F{ii} = A(:,:,ii)*F{ii-1};
end

for ii = 2:nSteps % Run each row
    % Set below-diagonal element of G
    G{ii,ii-1} = B(:,:,ii-1); 
    % Set the elements to the left of the below-diagonal element of G
    for jj = 1:ii-2 % run over columns
       G{ii,jj} = A(:,:,ii-1)*G{ii-1,jj};
    end
end
F = cell2mat(F);
G = cell2mat(G);
end

