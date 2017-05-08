function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
    
    % Building ax & ay vectors for a couple of matching points (video frame/logo)     
    function [ax,ay] = decompose(p_img, p_logo)
        
        ax = [-p_img(1) -p_img(2) -1 0 0 0 ...
               p_img(1)*p_logo(1) p_img(2)*p_logo(1) p_logo(1)];
           
        ay = [0 0 0 -p_img(1) -p_img(2) -1 ...
              p_img(1)*p_logo(2) p_img(2)*p_logo(2) p_logo(2) ];
    end

l = 4; 
A = [];

for i = 1:l
    [ax, ay] = decompose( video_pts(i,:), logo_pts(i,:));
    A = [A; ax];
    A = [A; ay];
end

%size(A)
[U, S, V] = svd(A);
h = V(:,end);
%size(h)
H = reshape(h,[3,3]);
H = transpose(H);
end