function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 1) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 1) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

% Camera Matrices
P1 = K* R1* [eye(3) -C1];
P2 = K* R2* [eye(3) -C2];

% N : Number of points
N = length(x1);
X = zeros(N,3);

% Building X processing one point at the time
for i=1:N
    x1_new = [x1(i,:) 1]';
    x2_new = [x2(i,:) 1]';
    
    % Skew correspondences
    skew1 = Vec2Skew(x1_new);
    skew2 = Vec2Skew(x2_new);
    
    % Solve    
    A = [skew1* P1; skew2* P2 ];
    [u,s,v] = svd(A);
    X(i,:) = v(1:3,end)./v(end,end);
end
