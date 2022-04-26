function [M_final,position,RPY] = DirectKinematic(q)

tol = 1e-8; % tolerance
%% Verify for  input errors
if length(q) ~= 6
    disp('Error in defining q!')
    M_final = 0;
    position = 0;
    return
end

%% Define matrices for our arm
%Taken from https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/wx250s.html
M_0 = [ 1 0 0  0.458325;
        0 1 0     0
        0 0 1  0.36065;
        0 0 0     1    ];

Slist = [0     0    1       0       0       0;
         0     1    0   -0.11065    0       0;
         0     1    0   -0.36065    0    0.04975;
         1     0    0       0    0.36065    0;
         0     1    0   -0.36065    0    0.29975;
         1     0    0       0    0.36065    0     ]';

omega_joints = Slist(1:3,1:6);
v_joints = Slist(4:6,1:6);

%% Compute the POE
%Solved using the formulas from https://en.wikipedia.org/wiki/Product_of_exponentials_formula
for i = 1:6
    %Calculate the rotation matrix
    w_hat(:,:,i) = [        0            -omega_joints(3,i)       omega_joints(2,i);
                               omega_joints(3,i)          0               -omega_joints(1,i);
                              -omega_joints(2,i)   omega_joints(1,i)              0           ];
    %Calculate the rotation matrix via the Rodrigues' rotational formula
    exp_w_hat(:,:,i) = eye(3)+w_hat(:,:,i)*sin(q(i))+w_hat(:,:,i)^2*(1-cos(q(i)));
    %Calculate the translation vector
    t(:,i) = (eye(3)-exp_w_hat(:,:,i))*(cross(omega_joints(:,i),v_joints(:,i)))+...
                omega_joints(:,i)*omega_joints(:,i)'*v_joints(:,i)*q(i);
    %Composing the matrix exponential
    mat(:,:,i) = [exp_w_hat(:,:,i),t(:,i)];
    expo(:,:,i) = [mat(:,:,i); 0 0 0 1];
end
%Computing the rototraslational matrix
M_final = expo(:,:,1)*expo(:,:,2)*expo(:,:,3)*expo(:,:,4)*expo(:,:,5)*expo(:,:,6)*M_0;
%Extracting the final position of the end effector
position = M_final(1:3,4);
% Get RPY angles from rotation matrix
Y = atan(M_final(2,1)/M_final(1,1));
P = atan(-M_final(3,1)/sqrt(M_final(3,2)^2+M_final(3,3)^2));
R = atan(M_final(3,2)/M_final(3,3));
RPY = [R,P,Y];

%% Verify for output Errors
if abs(det(M_final)-1) > tol % introduced some tolerance otherwise never equal to 0
    disp('Error in the determinant of M, is not equal to 1!')
    abs(det(M_final)-1)
    M_final = 0;
    position = 0;
    return
end

end

