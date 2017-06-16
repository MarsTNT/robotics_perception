function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs: 
%     X - siqze (N x 3) matrix of refined point 3D locations
    X = X0;
    [N,~] = size(X);
    for i = 1:N,
        b = [x1(i,1) x1(i,2) x2(i,1) x2(i,2) x3(i,1) x3(i,2)]';
        PR1 = K*R1 *(X(i,:) - C1 );    
        PR2 = K*R2 *(X(i,:) - C2 );
        PR3 = K*R3 *(X(i,:) - C3 );    
        
        %f_X = [u1/w1 v1/w1 u2/w2 v2/w2 u3/w3 v3/w3 ]';
        f_X = [PR1(1)/PR1(3), PR1(2)/PR1(3),...
               PR2(1)/PR2(3), PR2(2)/PR2(3),...
               PR3(1)/PR3(3), PR3(2)/PR3(3)]';
           
        J1 = Jacobian_Triangulation(C1, R1, K, X(i,:));
        J2 = Jacobian_Triangulation(C2, R2, K, X(i,:));
        J3 = Jacobian_Triangulation(C3, R3, K, X(i,:));
        J = [J1' J2' J3']';
        
        d_X = inv(J' * J) * J' * ( b - f_X);
        X(i,:) = X(i,:) + d_X';
    end
end    

function J = Jacobian_Triangulation(C, R, K, X)
     
    % For the ith camera optical center Ci and Orientation Ri
    M = K * R * (X - C);
    ui = M(1);
    vi = M(2);
    wi = M(3);
    f = K(1,1);
    px = K(1,3);
    py = K(2,3);
    
    dwi = [R(3,1) R(3,2) R(3,3)];
    dvi = [f*R(2,1) + py*R(3,1) f*R(2,2) + py*R(3,2) f*R(2,3) + py*R(3,3)];
    dui = [f*R(1,1) + px*R(3,1) f*R(1,2) + px*R(3,2) f*R(1,3) + px*R(3,3)];
    J = [ wi*dui - ui*dwi; wi*dvi - vi*dwi] ./ power(wi,2);
    
    
end
