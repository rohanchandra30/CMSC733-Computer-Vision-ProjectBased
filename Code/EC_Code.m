clc 
clear all
close all;

Nimages = 6;
I1 = im2double(imread('../Data/EC1.jpg'));
I1 = imrotate(I1, -90);
I2 = im2double(imread('../Data/EC2.jpg'));
I2 = imrotate(I2, -90);
I3 = im2double(imread('../Data/EC3.jpg'));
I3 = imrotate(I3, -90);
I4 = im2double(imread('../Data/EC4.jpg'));
I4 = imrotate(I4, -90);


load Data_Rohan.mat
if(~exist('datapoints.mat','file'))
    [x1, x2] = perform_sift(I2, I3);
    save('datapoints.mat', 'x1', 'x2');
else
load datapoints.mat
end
    
K = K';


% Do RANSAC
fpoints = cell(length(x1),6);
for i = 1:length(x1)
    fpoints{i,1} = x1(i,1:2);
    fpoints{i,2} = x2(i,1:2);
end

[Points,indx, F] = get_pointsandF_after_RANSAC_modified(fpoints,2);

x1 = Points{1,2}(:,1:2);
x2 = Points{1,2}(:,3:4);


% 
% figure;
% imshow(I2);hold on;
% plot(x1(:,1),x1(:,2),'r.');
% hold off;
% figure;
% imshow(I3);hold on;
% plot(x2(:,1),x2(:,2),'r.');
% hold off;
% dispMatchedFeatures(I2,I3,x1,x2, 'montage');

%% % Part 5 Calculating Esssential Matrix and the 4 poses of the second Camera

F = EstimateFundamentalMatrix(x1, x2);
E = EssentialMatrixFromFundamentalMatrix(F,K);
[Cset,Rset] = ExtractCameraPose(E);


%% % Part 6 Triangulating the 3D points and optimizing them

for i = 1:4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2) ;
end

[C, R, X] = DisambiguateCameraPose(Cset, Rset, Xset);
disp ('Now performing Non-Linear Triangulation..')
pause(4)
[X_opt, id] = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2, X);

X_opt = X_opt(id>0, :);
x1 = x1(id>0, :);
x2 = x2(id>0, :);
indx{1,2} = indx{1,2}(id>0,:);

subplot(1,2,1)
[t1, t2] = testX(X_opt, zeros(3, 1), eye(3), R, C, K);
DisplayCorrespondence(I3, x2, t2);
subplot(1,2,2)
DisplayCorrespondence(I2, x1, t1);

