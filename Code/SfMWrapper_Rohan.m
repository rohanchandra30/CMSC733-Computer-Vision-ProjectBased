%% Good Luck, Have fun!
clc
clear all
close all

Nimages = 6;
I1 = im2double(imread('../Data/1.jpg'));
I2 = im2double(imread('../Data/2.jpg'));
K = [568.996140852 0 643.21055941; 0 568.988362396 477.982801038; 0 0 1];

%% Part 4 Calculating point correspondences and Fundamental Matrix



if  ~exist('variables1.mat','file')
    [points, fpoints] = get_point_cell(Nimages);
    [Points,F] = get_pointsandF_after_RANSAC(points);
    save('variables1.mat');
else
    load variables1.mat
   
end


x1 = Points{1,2}(:,1:2);
x2 = Points{1,2}(:,3:4);
% dispMatchedFeatures(I1,I2,x1,x2, 'montage');

%% % Part 5 Calculating Esssential Matrix and the 4 poses of the second Camera

E = EssentialMatrixFromFundamentalMatrix(F{1,2},K);
[Cset,Rset] = ExtractCameraPose(E);


%% % Part 6 Triangulating the 3D points and optimizing them

for i = 1:4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2) ;
end

[C, R, X] = DisambiguateCameraPose(Cset, Rset, Xset);

X_opt = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2, X);

[t1, t2] = testX(X_opt, zeros(3, 1), eye(3), R, C, K);

% Display reprojection points
DisplayCorrespondence(I2, x2, t2);
DisplayCorrespondence(I1, x1, t1);

% Display point cloud and 3 camera poses
Display3D({zeros(3,1), C}, {eye(3), R}, X_opt);


%% Part 7 and 8

Cset = cell(Nimages-1, 1);
Rset = cell(Nimages-1, 1);
Rset{1,1} = eye(3);
Cset{1,1} = zeros(3,1);
Rset{2,1} = R;
Cset{2,1} = C;

for i = 2:Nimages-1
    x_n = [];
    X_n = [];
    for j = 1:length(x1)
      in = [];
      in = find(points{i-1,i+1}(:,1) == x1(j,1)  & points{i-1,i+1}(:,2) == x1(j,2));
      if ~isempty(in) 
             %fprintf('Point %d is mapping to 2 points\n', j);    
             x_n = [x_n; points{i-1,i+1}(in(1),3:4)];
             X_n = [X_n; X_opt(j,:)];
      end    
    end
    if length(x_n) < 6
        fprintf('Need atleast 6 correspondenses\n');
        continue;
    end    
    [Cnew,Rnew] = PnPRANSAC(X_n, x_n, K);
    [Cnew, Rnew] = NonlinearPnP(X_n, x_n, K, Cnew, Rnew);
    Cset{i,1} = Cnew;
    Rset{i,1} = Rnew;
    
    x1 = Points{i,i+1}(:,1:2);
    x2 = Points{i,i+1}(:,3:4);
    
    X_new = LinearTriangulation(K, Cset{i-1}, Rset{i-1}, Cset{i}, Rset{i}, x1, x2) ;
    X_new = NonlinearTriangulation(K, Cset{i-1}, Rset{i-1}, Cset{i}, Rset{i}, x1, x2, X_new);

    X = [X; X_new];
    
    %Building traj
    %traj = cell(N, 1);

    
    %V = BuildVisibilityMatrix(Nimages, X);
    %[Cset, Rset, X] = BundleAdjustment(Cset, Rset, X, K, traj, V);
end

