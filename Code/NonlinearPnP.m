function [C,R] = NonlinearPnP(X, x, K, C, R)
%% Inputs and Outputs
% X and x: Nx3 and Nx2 matrices whose row represents correspondences 
%                  between 3D and 2D points, respectively.
% K: intrinsic parameter
% C and R: for pose

opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'MaxIter', 1e3, 'Display', 'none');

%% Your code goes here
RC0 = [R C];
[RC,~] = lsqnonlin(@(RC) compute_error(RC, x, X, K), RC0,[], [], opts);
R = RC(:,1:3);
C = RC(:, 4);
end

function [f] = compute_error(RC0, x, X, K) 
   R = RC0(:,1:3);
   C = RC0(:, 4);
  % T = (R*C).* -1;  
   P = K*R*[eye(3) -C];  
   X = [X ones(size(X,1),1)];
   x_new = P*X';
   x_new = x_new./x_new(3,:);
   u = x_new(1,:);
   v = x_new(2,:);
   f = [(x(:,1) - u') ;  (x(:,2) - v')]; 
end