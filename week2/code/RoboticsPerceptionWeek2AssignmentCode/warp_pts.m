function [ warped_pts ] = warp_pts( video_pts, logo_pts, sample_pts)
% warp_pts computes the homography that warps the points inside
% video_pts to those inside logo_pts. It then uses this
% homography to warp the points in sample_pts to points in the logo
% image
% Inputs:
%     video_pts: a 4x2 matrix of (x,y) coordinates of corners in the
%         video frame
%     logo_pts: a 4x2 matrix of (x,y) coordinates of corners in
%         the logo image
%     sample_pts: a nx2 matrix of (x,y) coordinates of points in the video
%         video that need to be warped to corresponding points in the
%         logo image
% Outputs:
%     warped_pts: a nx2 matrix of (x,y) coordinates of points obtained
%         after warping the sample_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% Complete est_homography first!
[ H ] = est_homography(video_pts, logo_pts);
% YOUR CODE HERE

    % Function to compute a warping points using the Homography matrix H
    function x_res = compute_warping_point(x,mat)
        
       x1 = (mat(1,1)*x(1) + mat(1,2)*x(2) + mat(1,3))...
           /(mat(3,1)*x(1) + mat(3,2)*x(2) + mat(3,3));
       
       x2 = (mat(2,1)*x(1) + mat(2,2)*x(2) + mat(2,3))...       
           /(mat(3,1)*x(1) + mat(3,2)*x(2) + mat(3,3));
       
       x_res = [x1 x2];
    end

N = size(sample_pts);
N = N(1);

warped_pts = [];

for i = 1:N
    x_res = compute_warping_point(sample_pts(i,:),H);
    warped_pts = [warped_pts; x_res];    
end
% testing size
%size(sample_pts)==size(warped_pts)

end

