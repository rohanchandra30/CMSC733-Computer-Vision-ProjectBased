clc 
clear all
close all;

% load Data.mat
load Data_Rohan.mat
load datapoints.mat
K = K';
I1 = im2double(imread('../Data/EC1_corrected.jpg'));
I2 = im2double(imread('../Data/EC2_corrected.jpg'));


% Do RANSAC
fpoints = cell(length(x1),6);
for i = 1:length(x1)
    fpoints{i,1} = x1(i,1:2);
    fpoints{i,2} = x2(i,1:2);
end

[Points,indx, F] = get_pointsandF_after_RANSAC_modified(fpoints,2);

x1 = Points{1,2}(:,1:2);
x2 = Points{1,2}(:,3:4);


% figure;
% imshow(I1);hold on;
% plot(x1(:,1),x1(:,2),'r.');
% hold off;
% figure;
% imshow(I2);hold on;
% plot(x2(:,1),x2(:,2),'r.');
% hold off;
dispMatchedFeatures(I1,I2,x1,x2, 'montage');

F = EstimateFundamentalMatrix(x1, x2);
E = EssentialMatrixFromFundamentalMatrix(F,K);
[Cset,Rset] = ExtractCameraPose(E);


%% % Part 6 Triangulating the 3D points and optimizing them

for i = 1:4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2) ;
end

[C, R, X] = DisambiguateCameraPose(Cset, Rset, Xset);

[X_opt, id] = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2, X);

% X_opt = X_opt(id>0, :);
% x1 = x1(id>0, :);
% x2 = x2(id>0, :);
% indx{1,2} = indx{1,2}(id>0,:);
[t1, t2] = testX(X_opt, zeros(3, 1), eye(3), R, C, K);

% Display reprojection points
DisplayCorrespondence(I2, x2, t2);
DisplayCorrespondence(I1, x1, t1);


