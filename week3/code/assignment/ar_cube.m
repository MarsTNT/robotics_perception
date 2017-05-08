function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course
% Completed by Kalifou Rene B. TRAORE

%%%%% Extract the pose from the homography

%% Extracting the Rotation matrix R
h1 = H(:,1);
h2 = H(:,2);
h3 = cross(h1, h2);
H_prime = [h1 h2 h3];
[U, S, V] = svd(H_prime);

detM = det(U*V'); % transpose(V));
inter = [ 1 0 0;...
          0 1 0;...
          0 0 detM ];
R = U*inter*V'; % transpose(V);

%%  Extracting the translation matrix t
t = H(:,3)/ norm( H(:,1) ); %% t = h3 / norm(h1)
t(3) = abs(t(3)); % Third column must be positive

%%%%% Project the points using the pose

% Xim = Xc /z
% with Xc  = K(RX + t)
%%
proj_points = zeros(size(render_points,1),2);
% Iterate through the rest of points
for i = 1:size(render_points,1)
    Xc = K*(R * transpose(render_points(i,:))+t); % Xci  = K(RXi + t)
    Xim = Xc / Xc(3);
    proj_points(i,:) = Xim(1:2) ;
    
end
end






