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

X_homo = [X0,ones(length(X0),1)];

for i = 1:2*length(X0)
    
   fun1(2*i-1) =  x1(i,1) - (P1(1,:)*X_homo({i}))/((P1(3,:)*X_homo{i}));
   fun1(2*i) = x1(i,2) - (P1(2,:)*X_homo{i})/(P1(3,:)*X_homo{i});
end


for i = 1:2*length(X0)
    
   fun2(2*i-1) =  x2(i,1) - (P2(1,:)*X_homo{i})/(P2(3,:)*X_homo{i});
   fun2(2*i) = x2(i,2) - (P2(2,:)*X_homo{i})/(P2(3,:)*X_homo{i});
end


fun = [fun1;fun2];

X = lsqnonlin(fun,X0);


end

