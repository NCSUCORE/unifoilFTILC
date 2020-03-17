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


A = A.Data;
B = B.Data;
ns = size(A,3);
FblkDims = size(A(:,:,1));
GblkDims = size(A(:,:,1)*B(:,:,1));
% Preallocate F as cell array, each cell is 1 block of block matrix
F = cell(ns,1);
F(:) = {zeros(FblkDims)};
F{1} = eye(size(F{1}));
% F{1} = A(:,:,1); % Set the first block

% Preallocate G as cell array, each cell is 1 block of block matrix
G = cell(ns,ns-1);
G(:,:) = {zeros(GblkDims)};

for ii = 2:ns % Run each row
    F{ii} = A(:,:,ii-1)*F{ii-1};
    G{ii,ii-1} = B(:,:,ii-1); % Set the lower, off-diagonal element
    for jj = 1:ii-2 % set lower half elements by multiplying with the line above
       G{ii,jj} = A(:,:,ii-1)*G{ii-1,jj};
    end
end
F = cell2mat(F);
G = cell2mat(G);
end

