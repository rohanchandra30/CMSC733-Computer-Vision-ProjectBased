function m = projection(j, i, rt, xyz, r0, a)
% symbolic projection function
% code automatically generated with maple

K = [a(1) a(2) a(3); 0 a(4) a(5); 0 0 1];

%% Code to fill
% Build P matrix from rt
% rt is 7 dimensional vector-the first three are camera center and the rest are quaternion

rt1 = rt;
RQ = rt1(:,4:7);
C=rt1(:,1:3);
R = Quaternion2Matrix(RQ);
P = K*R*[eye(3), -C'];

%% Get xyz
X = xyz';

%% Code to fill
% Project the 3D point to the camera to produce (u,v)
X = [X;1];

x = P*X;
x = x./x(3,:);
u = x(1,:);
v = x(2,:);
% Return reprojection
m(1) = u;
m(2) = v;



