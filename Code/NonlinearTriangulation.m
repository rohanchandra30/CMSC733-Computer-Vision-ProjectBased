function X = NonlinearTriangulation(K, C1, R1, C2, R2, x1, x2, X0)
%% Notes:
% (INPUT) C1 and R1: the 1st camera pose
% (INPUT) C2 and R2: the 2nd camera pose
% (INPUT) x1 and x2: two Nx2 matrices whose row represents correspondence 
%                    between the 1st and 2nd images where N is the number 
%                    of correspondences.
% (INPUT and OUTPUT) X: Nx3 matrix whose row represents 3D triangulated point.

%% Your code goes here


P1 = [R1,C1];
P2 = [R2,C2];


X = lsqnonlin(@(X)func(P1, P2, x1, x2, X),X0);


end



function fun = func(P1, P2, x1, x2, X)


X_homo = [X,ones(length(X),1)];

for i = 1:length(X)
    
    fun1(2*i-1) =  x1(i,1) - (P1(1,:)*X_homo(i,:)')/((P1(3,:)*X_homo(i,:)'));
    fun1(2*i) = x1(i,2) - (P1(2,:)*X_homo(i,:)')/(P1(3,:)*X_homo(i,:)');
end


for i = 1:length(X)
    
    fun2(2*i-1) =  x2(i,1) - (P2(1,:)*X_homo(i,:)')/(P2(3,:)*X_homo(i,:)');
    fun2(2*i) = x2(i,2) - (P2(2,:)*X_homo(i,:)')/(P2(3,:)*X_homo(i,:)');
end


fun = [fun1';fun2'];



end