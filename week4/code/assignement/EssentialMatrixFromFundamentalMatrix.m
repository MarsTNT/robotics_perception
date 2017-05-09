function E = EssentialMatrixFromFundamentalMatrix(F,K)
%% EssentialMatrixFromFundamentalMatrix
% Use the camera calibration matrix to esimate the Essential matrix
% Inputs:
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     F - size (3 x 3) fundamental matrix from EstimateFundamentalMatrix
% Outputs:
%     E - size (3 x 3) Essential matrix with singular values (1,1,0)

% Computing E and decomposing it
E = K'*F*K ;
[u,s,v] = svd(E);

% Reconstructing the Essential matrix using (1,1,0) singular values
s_bis = [ 1, 0, 0; 0, 1, 0; 0, 0, 0];
E = u* s_bis* v';
E = E / norm(E);



