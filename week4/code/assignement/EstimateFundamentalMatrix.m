function F_rank2 = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

% Building A from the following equation : (x2.T) * F * x1
A = [ x1(1)*x2(1) x1(1)*x2(2) x1(1) ... 
      x1(2)*x2(1) x1(2)*x2(2) x1(2) ...
      x2(1) x2(2) 1];
  
% Getting F from A
[u,s,v] = svd(A);
x = v(:,8);
F = reshape(x,3,3);


% SVD Decomposition of F
[u1,s1,v1] = svd(F);

%Reconstituting F using modified Diag. Matrix S
s1(end,end) = 0;
F_rank2 = u1*s1*v1';

% Normalizing 
F_rank2  = F_rank2 / norm(F_rank2);

% References :
% http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MOHR_TRIGGS/node50.html
