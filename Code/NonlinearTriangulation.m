function [X, idx] = NonlinearTriangulation(K, C1, R1, C2, R2, x1, x2, X0)
%% Notes:
% (INPUT) C1 and R1: the 1st camera pose
% (INPUT) C2 and R2: the 2nd camera pose
% (INPUT) x1 and x2: two Nx2 matrices whose row represents correspondence 
%                    between the 1st and 2nd images where N is the number 
%                    of correspondences.
% (INPUT and OUTPUT) X: Nx3 matrix whose row represents 3D triangulated point.

%% Your code goes here

P1 = K*R1*[eye(3) -1.*C1];
P2 = K*R2*[eye(3) -1.*C2];

opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'none');

X = zeros(size(X0,1),3);
idx = ones(size(X0,1),1);
for i=1:length(X0(:,1))
    [X(i,:), ~, ~, exitflag] = lsqnonlin(@(X) func_triang(X, P1, P2, x1(i,:), x2(i,:)), X0(i,:), [], [], opts);
    if(exitflag ~=1)
        idx(i) = 0;
    end    
end


end


function f = func_triang(X, P1, P2, x1, x2)

Xhom = [X 1]';

f = [(x1(1)-(P1(1,:)*Xhom)/(P1(3,:)*Xhom));
     (x1(2)-(P1(2,:)*Xhom)/(P1(3,:)*Xhom));
     (x2(1)-(P2(1,:)*Xhom)/(P2(3,:)*Xhom));
     (x2(2)-(P2(2,:)*Xhom)/(P2(3,:)*Xhom))];
end


% function [X, idx] = NonlinearTriangulation(K, C1, R1, C2, R2, x1, x2, X0)
% %% Notes:
% % (INPUT) C1 and R1: the 1st camera pose
% % (INPUT) C2 and R2: the 2nd camera pose
% % (INPUT) x1 and x2: two Nx2 matrices whose row represents correspondence
% %                    between the 1st and 2nd images where N is the number
% %                    of correspondences.
% % (INPUT and OUTPUT) X: Nx3 matrix whose row represents 3D triangulated point.
% 
% %% Your code goes here
% 
% 
% P1 = K* [R1 -R1*C1];
% P2 = K* [R2 -R2*C2];
% 
% %opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'iter');
% X = zeros(size(X0,1),3);
% 
% % X = lsqnonlin(@(X)func(P1, P2, x1, x2, X),X0);
% opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'iter')
% % [X, ~] = lsqnonlin(@(X) fun(X, P1, P2, x1, x2), X0, [], [], opts);
% idx = ones(size(X0,1),1);
% for i=1:length(X0(:,1))
%     [X(i,:), ~, ~, exitflag] = lsqnonlin(@(X) func(X, P1, P2, x1(i,:), x2(i,:)), X0(i,:), [], [], opts);
%     if(exitflag ~=1)
%         idx(i) = 0;
%     end    
% end
% 
% end
% 
% 
% 
% function fun = func(X, P1, P2, x1, x2)
% 
% 
% X_homo = [X,1];
% 
% for i = 1:length(X)
%     
%     fun1(2*i-1) =  x1(i,1) - (P1(1,:)*X_homo(i,:)')/((P1(3,:)*X_homo(i,:)'));
%     fun1(2*i) = x1(i,2) - (P1(2,:)*X_homo(i,:)')/(P1(3,:)*X_homo(i,:)');
% end
% 
% 
% for i = 1:length(X)
%     
%     fun2(2*i-1) =  x2(i,1) - (P2(1,:)*X_homo(i,:)')/(P2(3,:)*X_homo(i,:)');
%     fun2(2*i) = x2(i,2) - (P2(2,:)*X_homo(i,:)')/(P2(3,:)*X_homo(i,:)');
% end
% 
% fun = [fun1';fun2'];
% 
% end