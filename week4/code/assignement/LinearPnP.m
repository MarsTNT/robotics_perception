function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly



% Computing P
X = [X , ones(size(X,1),1)];
A = zeros(3 * size(X,1), 12);

for i = 1 : size(X,1)
    Xt = X(i,:);
    x_temp = [x(i,:),1];
    xx = K \ x_temp';    
    j = (i-1)*3 + 1;
    A(j:j+2,:) = [0, -1, xx(2); ...
                  1, 0, -xx(1); ...
                  -xx(2), xx(1), 0] * ...
                 [Xt, zeros(1,4), zeros(1,4); ...
                  zeros(1,4), Xt, zeros(1,4); ...
                  zeros(1,4), zeros(1,4), Xt];

end
[u,s,v] = svd(A);
t = v(:,end); %/v(end,end);
t = reshape(t, 4, 3)';
%  R 
R = t(:,1:3);

% Cleaning up R and t, computing C
[u,d,v] = svd(R);

if det(u*v') > 0
    R = u*v';
    tc = t(:,4)/d(1,1);        
else
    R = -u*v';
    tc = -t(:,4)/d(1,1);
end
C = -1* (R') *tc;
