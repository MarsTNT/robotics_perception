function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - siqze (N x 3) matrix of refined point 3D locations
    b = [x1(1) x1(2) x2(1) x2(2) x3(1) x(2)]';

    [u1; v1; w1] = KR1 *(X - C1 );
    [u2; v2; w2] = KR2 *(X - C2 );
    [u3; v3; w3] = KR3 *(X - C3 );
    f_X = [u1/w1 v1/w1 u2/w2 v2/w2 u3/w3 v3/w3 ]';
    R = [R1,R2,R3];
    C = [C1,C2,C3];
    J = Jacobian_Triangulation(C, R, K, X);
    d_X = inv(J' * J) * J' * ( b - f_X);
    
    % Put it into a loop
    X = X - d_X;
end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
end

function J = Jacobian_Triangulation(C, R, K, X)
     
    % For the ith camera optical center Ci and Orientation Ri
    [l,r] = size(C);
    Inter = zeros(l,3);
    for i=1:length(l) 
        [ui; vi; wi] = K * R(i,:) * (X - C(i,:));
        [k11 k12 k13; k21 k22 k23; k31 k32 k33] = K;
        f = k11;
        px = k13;
        py = k23;
        [r11 r12 r13; r21 r22 r23; r31 r32 r33] = r;
    
        dwi = [r31 r32 r33];
        dvi = [f*r21 + py*r31 f*r22 + py*r32 f*r23 + py*r33];
        dui = [f*r11 + px*r31 f*r1 + px*r32 f*r13 + px*r33];
        dfi = [ wi*dui - ui*dwi; wi*dvi - vi*dwi] ./ power(wi,2);
        Inter(i,:) = dfi';
    end
    J = Inter';
    
end
